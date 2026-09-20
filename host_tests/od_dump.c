/**
 * @file od_dump.c
 * @brief Print the object dictionary as CSV.
 *
 * Used by master/tools/check_eds.py to prove the hand-written EDS and the
 * firmware's table describe the same device. Building the real table and
 * asking it what it contains beats parsing C with a regular expression.
 *
 *   make -C host_tests od_dump && ./host_tests/build/od_dump
 */
#include <stdio.h>
#include "od_data.h"

int main(void)
{
    od_load_defaults();
    od_apply_node_id(0x10);

    if (!od_check_sorted()) {
        fprintf(stderr, "object dictionary is not sorted\n");
        return 2;
    }

    printf("index,sub,datatype,readable,writable,constant,tpdo,rpdo,size\n");
    for (uint16_t i = 0; i < robocar_od.count; i++) {
        const co_od_entry_t *e = &robocar_od.entries[i];
        printf("0x%04X,%u,0x%04X,%u,%u,%u,%u,%u,%u\n",
               e->index, e->sub, e->type,
               (e->access & CO_ACC_R) ? 1u : 0u,
               ((e->access & CO_ACC_W) && !(e->access & CO_ACC_CONST)) ? 1u : 0u,
               (e->access & CO_ACC_CONST) ? 1u : 0u,
               (e->access & CO_ACC_TPDO) ? 1u : 0u,
               (e->access & CO_ACC_RPDO) ? 1u : 0u,
               e->size);
    }
    return 0;
}
