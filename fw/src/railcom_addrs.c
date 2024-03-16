/* Tracking of detected RailCom addresses per track */

#include "railcom_addrs.h"
#include "assert.h"

/* Local variables -----------------------------------------------------------*/

RCTrack rc_tracks[RC_TRACKS_COUNT];
bool rc_tracks_changed = false;

/* Private prototypes --------------------------------------------------------*/

/* Implementation ------------------------------------------------------------*/

void rca_init(void) {
    rca_remove_all();
    rc_tracks_changed = false;
}

void rca_update_100ms(void) {
    for (size_t track = 0; track < RC_TRACKS_COUNT; track++) {
        if (rc_tracks[track].count > 0) {
            for (size_t i = rc_tracks[track].count-1; i > 0; i--) {
                rc_tracks[track].addrs[i].timeout--;
                if (rc_tracks[track].addrs[i].timeout == 0)
                    rca_remove_i(track, i);
            }
        }
    }
}

void rca_add_or_update(size_t track, uint16_t addr) {
    assert_param(track < RC_TRACKS_COUNT);

    int i = rca_addr_i(track, addr);
    if (i >= 0) {
        rc_tracks[track].addrs[i].timeout = RC_ADDR_TIMEOUT;
    } else {
        assert_param(rc_tracks[track].count < RC_ADDS_MAX_IN_TRACK);
        rc_tracks[track].addrs[rc_tracks[track].count].addr = addr;
        rc_tracks[track].addrs[rc_tracks[track].count].timeout = RC_ADDR_TIMEOUT;
        rc_tracks[track].count++;
        rc_tracks_changed = true;
    }
}

bool rca_full(size_t track) {
    assert_param(track < RC_TRACKS_COUNT);
    return rc_tracks[track].count < RC_ADDS_MAX_IN_TRACK;
}

void rca_remove(size_t track, uint16_t addr) {
    assert_param(track < RC_TRACKS_COUNT);
    int i = rca_addr_i(track, addr);
    assert_param(i >= 0);
    rca_remove_i(track, i);
}

void rca_remove_i(size_t track, size_t i) {
    assert_param(track < RC_TRACKS_COUNT);
    assert_param(i < rc_tracks[track].count);

    for (size_t j = i; j < rc_tracks[track].count-1; j++)
        rc_tracks[track].addrs[j] = rc_tracks[track].addrs[j+1];
    rc_tracks[track].count--;
    rc_tracks_changed = true;
}

void rca_remove_all_in_track(size_t track) {
    assert_param(track < RC_TRACKS_COUNT);
    bool changed = (rc_tracks[track].count != 0);
    rc_tracks[track].count = 0;
    rc_tracks_changed |= changed;
}

void rca_remove_all() {
    bool changed = false;
    for (size_t i = 0; i < RC_TRACKS_COUNT; i++) {
        if (rc_tracks[i].count != 0)
            changed = true;
        rc_tracks[i].count = 0;
    }
    rc_tracks_changed |= changed;
}

bool rca_is_addr(size_t track, uint16_t addr) {
    assert_param(track < RC_TRACKS_COUNT);
    return rca_addr_i(track, addr) > -1;
}

int rca_addr_i(size_t track, uint16_t addr) {
    assert_param(track < RC_TRACKS_COUNT);
    for (size_t i = 0; i < rc_tracks[track].count; i++)
        if (rc_tracks[track].addrs[i].addr == addr)
            return i;
    return -1;
}
