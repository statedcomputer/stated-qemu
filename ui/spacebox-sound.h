/*
 * The sink's sound (hw/display/virtio-gpu-remote.c): how much waits to be heard, and when it is played faster to
 * work off a surplus. Kept apart so that tests/unit/test-spacebox-sound.c runs them with no sound device.
 */
#ifndef UI_SPACEBOX_SOUND_H
#define UI_SPACEBOX_SOUND_H

#include <stdbool.h>
#include <stdint.h>

/*
 * What the backend was given and has not played yet, us, by the clock: the time of what was fed, less the time since
 * playing began. The lead is only how much the backend may be given; when the sound that arrives is less, this is
 * less than the lead, and a surplus must not be read from the lead.
 */
static inline int64_t spacebox_sound_queued_us(uint64_t fed_bytes, uint64_t bytes_per_s, int64_t played_us)
{
    int64_t fed_us = (int64_t)(fed_bytes * 1000000 / bytes_per_s);

    return fed_us > played_us ? fed_us - played_us : 0;
}

/* One step of the choice to play 1% faster: on well above the target, off near it, as it was between the two. */
static inline bool spacebox_sound_fast(bool fast, int64_t held_avg_us, int64_t target_us)
{
    if (held_avg_us > target_us * 5 / 4) {
        return true;
    }
    if (held_avg_us < target_us * 21 / 20) {
        return false;
    }
    return fast;
}

#endif
