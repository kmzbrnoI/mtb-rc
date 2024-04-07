/* Diagnostics */

#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef union {
    struct {
        bool addr_zero : 1;
        bool bad_mtbbus_polarity : 1;
    } bits;
    uint8_t all;
} error_flags_t;

extern error_flags_t error_flags;

typedef union { // TODO
    struct {
        bool extrf : 1;
        bool borf : 1;
        bool wdrf : 1;
        bool jtrf : 1;
        bool missed_timer : 1;
        bool vcc_oscilating : 1;
    } bits;
    uint8_t all;
} mtbbus_warn_flags_t;

enum MtbRcDV {
    dvVersion = 0,
    dvState = 1,
    dvUptime = 2,
    dvErrors = 10,
    dvWarnings = 11,
    dvVMCU = 12,
    dvTempMCU = 13,

    dvRCLLCutoutsStarted = 32,
    dvRCLLCutoutsFinished = 33,
    dvRCLLCutoutsTimeout = 34,
    dvRCLLCutoutsDataCh1 = 35,
    dvRCLLCutoutsDataCh2 = 36,
    dvRCLLCutoutsNoReadyToParse = 37,
};

extern mtbbus_warn_flags_t mtbbus_warn_flags;
extern mtbbus_warn_flags_t mtbbus_warn_flags_old;
extern uint32_t uptime_seconds;

void diag_update(void); // called each 100ms
