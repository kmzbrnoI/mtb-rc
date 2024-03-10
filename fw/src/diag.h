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

extern mtbbus_warn_flags_t mtbbus_warn_flags;
extern mtbbus_warn_flags_t mtbbus_warn_flags_old;
