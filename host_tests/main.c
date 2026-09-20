/**
 * @file main.c
 * @brief Host test runner for the portable half of the firmware.
 *
 *   make -C host_tests run
 */
#include "test_util.h"

int main(void)
{
    printf("Robocar CANopen drive node - host tests\n\n");

    suite_od();
    suite_sdo();
    suite_cia402();
    suite_pdo();
    suite_nmt_emcy();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
