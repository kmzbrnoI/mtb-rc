/* Diagnostics */

#include "diag.h"

/* Variables -----------------------------------------------------------------*/

error_flags_t error_flags = {0};
mtbbus_warn_flags_t mtbbus_warn_flags = {0};
mtbbus_warn_flags_t mtbbus_warn_flags_old = {0};
uint32_t uptime_seconds = 0;

/* Implementation ------------------------------------------------------------*/

void diag_update(void) {
    {
        static uint8_t uptime_counter = 0;
        uptime_counter++;
        if (uptime_counter >= 10) {
            uptime_seconds++;
            uptime_counter = 0;
        }
    }
}
