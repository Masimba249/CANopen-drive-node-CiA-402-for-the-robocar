# The CiA 402 state machine

Implemented in
[`firmware/components/cia402/cia402.c`](../firmware/components/cia402/cia402.c),
tested transition by transition in
[`host_tests/test_cia402.c`](../host_tests/test_cia402.c).

```
                    ┌──────────────────────────┐
     power on  ──▶  │ NOT READY TO SWITCH ON   │
                    └───────────┬──────────────┘
                            T1  │ (automatic)
                    ┌───────────▼──────────────┐
             ┌─────▶│   SWITCH ON DISABLED     │◀─────┐◀─────┐
             │      └───────────┬──────────────┘      │      │
             │              T2  │ Shutdown            │      │
          T7 │      ┌───────────▼──────────────┐      │      │
             ├──────│   READY TO SWITCH ON     │      │      │ T15
             │      └───────────┬──────────────┘      │      │ Fault reset
             │              T3  │ Switch on       T10 │      │
         T10 │      ┌───────────▼──────────────┐      │      │
             ├──────│      SWITCHED ON         │──────┤      │
             │      └───────────┬──────────────┘      │      │
             │              T4  │ Enable operation    │      │
          T9 │      ┌───────────▼──────────────┐  T12 │      │
             └──────│   OPERATION ENABLED      │      │      │
                    └──┬────────┬───────┬──────┘      │      │
                 T5/T8 │    T11 │       │ T13         │      │
                       │   Quick│       │ fault       │      │
                       │   stop │       │             │      │
           back to ◀───┘  ┌─────▼───────────────┐     │      │
        SWITCHED ON /     │  QUICK STOP ACTIVE  │─────┘      │
        READY TO SW ON    └─────────────────────┘            │
                                  T16 ▲ (option 5..7)        │
                          ┌───────────┴─────────┐            │
                          │ FAULT REACTION      │            │
              any state ─▶│ ACTIVE              │            │
                          └───────────┬─────────┘            │
                                  T14 │ (ramp finished)      │
                          ┌───────────▼─────────┐            │
                          │       FAULT         │────────────┘
                          └─────────────────────┘
```

## Controlword (6040h) commands

Bit 2 (quick stop) is **active low**, and bit 7 (fault reset) is **edge
triggered**. Both trip people up.

| Command | Mask | Value | Typical word |
|---|---|---|---|
| Shutdown | `0x0087` | `0x0006` | `0x0006` |
| Switch on | `0x008F` | `0x0007` | `0x0007` |
| Switch on + enable operation | `0x008F` | `0x000F` | `0x000F` |
| Disable voltage | `0x0082` | `0x0000` | `0x0000` |
| Quick stop | `0x0086` | `0x0002` | `0x0002` |
| Disable operation | `0x008F` | `0x0007` | `0x0007` |
| Enable operation | `0x008F` | `0x000F` | `0x000F` |
| Fault reset | rising edge of bit 7 | | `0x0080` |

The masks matter as much as the values. `0x000E` is a *shutdown* even though
bit 3 is set, because bit 3 is "don't care" for that command. And any word with
bit 1 clear is a *disable voltage* regardless of everything else — which is
why the decoder tests it first.

Extra bits this drive honours:

- **bit 8 (halt)**: ramp to zero using 6084h but stay in OPERATION ENABLED.

## Statusword (6041h)

| State | Mask | Value |
|---|---|---|
| Not ready to switch on | `0x004F` | `0x0000` |
| Switch on disabled | `0x004F` | `0x0040` |
| Ready to switch on | `0x006F` | `0x0021` |
| Switched on | `0x006F` | `0x0023` |
| Operation enabled | `0x006F` | `0x0027` |
| Quick stop active | `0x006F` | `0x0007` |
| Fault reaction active | `0x004F` | `0x000F` |
| Fault | `0x004F` | `0x0008` |

Order matters when decoding: check the `0x4F` patterns for "not ready" and
"switch on disabled" first, then the `0x6F` ones, then the fault patterns.
`cia402_decode_status()` and `decode_state()` in `master/robocar.py` do exactly
that.

Other bits:

| Bit | Meaning here |
|---|---|
| 4 | voltage enabled — the supply is present and inside 2002h:04/05 |
| 7 | warning |
| 9 | remote — the node is NMT OPERATIONAL, so the controlword is honoured |
| 10 | target reached — inside 606Dh for at least 606Eh ms |
| 11 | internal limit active — the demand was clipped by 607Fh |
| 12 | speed (pv mode) — measured speed has been below 606Fh for 6070h ms |

## Option codes

| Object | Meaning | Default | Behaviour implemented |
|---|---|---|---|
| 605Ah | Quick stop | 2 | 1–3: ramp on 6085h then go to SWITCH ON DISABLED. 5–7: ramp, then **stay** in QUICK STOP ACTIVE so T16 back to OPERATION ENABLED is possible. |
| 605Eh | Fault reaction | 2 | 2: ramp on 6085h then FAULT. 0: coast, straight to FAULT. |
| 6007h | Abort connection | 2 | 0: nothing. 1: signal a fault. 2: quick stop. 3: disable voltage. |

6007h is applied on three different events, because on a real bus the master
can disappear in three different ways: the heartbeat it produces stops
(1016h), it drops the node out of OPERATIONAL, or the RPDO command stream goes
quiet (2002h:06). All three funnel through one function in `main.c`.

## Power stage

The bridge is energised in SWITCHED ON, OPERATION ENABLED, QUICK STOP ACTIVE
and FAULT REACTION ACTIVE, and de-energised everywhere else. Leaving
OPERATION ENABLED also clears the internal ramp, so re-enabling a drive always
starts from standstill rather than from a stale demand — a small thing that
prevents a very unpleasant surprise.

## The ramp

Profile Velocity mode, one first-order ramp per axis:

```
target = clamp(60FFh, ±607Fh)          bit 11 set if it was clipped
rate   = accelerating ? 6083h : 6084h  (6085h while quick-stopping)
step   = rate * dt_ms / 1000           remainder carried between ticks
```

Carrying the remainder matters: at a 5 ms control period and a modest
acceleration, integer truncation alone would swallow a noticeable fraction of
the ramp.
