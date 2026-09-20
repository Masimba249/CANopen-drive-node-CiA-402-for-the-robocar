/**
 * @file app_config.h
 * @brief Build-time defaults. Anything here can be overridden at runtime
 *        through object 2005h + a 1010h store, which survives a reflash of
 *        the application only if the NVS partition is preserved.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/** CANopen node-id, 1..127. */
#define APP_NODE_ID_DEFAULT      0x10
/** Bus speed. 500 kbit/s is the usual compromise for a metre-scale robot. */
#define APP_BITRATE_KBPS_DEFAULT 500

/** NVS namespace used for the persisted parameter set (object 1010h). */
#define APP_NVS_NAMESPACE  "canopen"
#define APP_NVS_KEY_PARAMS "params"

/** Bumped whenever the persisted layout changes, so old blobs are ignored. */
#define APP_PARAM_VERSION  2

#endif /* APP_CONFIG_H */
