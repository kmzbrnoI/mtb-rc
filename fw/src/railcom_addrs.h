/* Tracking of detected RailCom addresses per track
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// after this timeout the detected address will be marked as inactive
#define RC_ADDR_TIMEOUT 20 // in 100ms
#define RC_TRACKS_COUNT 8
#define RC_ADDS_MAX_IN_TRACK 8 // maximum number of addresses detected in track

typedef struct {
    uint16_t addr;
    uint32_t timeout; // in 100 ms; decrementing each 100 ms; 0 = remove from list
} RCAddrInTrack;

typedef struct {
    RCAddrInTrack addrs[RC_ADDS_MAX_IN_TRACK];
    size_t count;
} RCTrack;

extern RCTrack rc_tracks[RC_TRACKS_COUNT];
extern bool rc_tracks_changed;

void rca_init(void);
void rca_update_100ms(void);

bool rca_full(size_t track);
void rca_add_or_update(size_t track, uint16_t addr);
void rca_remove(size_t track, uint16_t addr);
void rca_remove_i(size_t track, size_t i);
void rca_remove_all_in_track(size_t track);
void rca_remove_all();

bool rca_is_addr(size_t track, uint16_t addr);
int rca_addr_i(size_t track, uint16_t addr); // -1 = not found
