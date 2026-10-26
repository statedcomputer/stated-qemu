/*
 * Spacebox: the UI evals' probe of the sink (screen/ui/README.md of the
 * spacebox repository, "The sink's probe"). The sink's part, the control,
 * the files and the queue lines, is hw/display/virtio-gpu-remote-probe.c;
 * the presenter's part, the readback of each frame it swaps and the faults
 * that act in it, is ui/spacebox-ui-probe-gl.h, beside the presenter
 * (ui/spacebox-presenter.h).
 */

#ifndef UI_SPACEBOX_UI_PROBE_H
#define UI_SPACEBOX_UI_PROBE_H

#define SP_PROBE_REGIONS 4      /* regions of a spec, at most */
#define SP_PROBE_ROWS 4         /* rows of the strip, at most */

typedef struct SpProbeRect {
    char name[16];
    int x, y, w, h;
} SpProbeRect;

/* the spec of a probe (probe-spec.json), in the scanout's pixels */
typedef struct SpProbeSpec {
    unsigned gen;               /* one for each ui-start */
    SpProbeRect strip;          /* "strip": cell x nrows high */
    int cell, nrows;
    char rows[SP_PROBE_ROWS][16];
    int nregions;
    SpProbeRect regions[SP_PROBE_REGIONS];
} SpProbeSpec;

/* a probe runs (ui-start); read by the presenter without a lock */
extern bool spacebox_ui_probe_on;
/* the running probe's spec; false when none runs */
bool spacebox_ui_probe_spec(SpProbeSpec *spec);

/*
 * One swap, handed over by the presenter after its swap: the pixels of its
 * readback (the strip's rectangle, then each region's, each RGBA, bottom row
 * first) or, when they could not be kept, none. Times are of
 * g_get_monotonic_time() (the presenter's clock), qpc_* the
 * QueryPerformanceCounter's (Windows; 0 elsewhere).
 */
typedef struct SpProbeShot {
    unsigned gen;               /* the spec's */
    uint64_t seq;               /* the presenter's swap */
    int slot;
    int64_t t_call, t_ret;      /* the swap's call and return */
    int64_t qpc_call, qpc_ret;
    int64_t t_read;             /* the readback was done (0: lost) */
    int64_t flush_dev, flush_read;  /* its guest flush: the source's time on
                                       this clock, its read (0 = not known) */
    double sx, sy;              /* the slot's pixels per scanout pixel */
    const char *lost;           /* why its pixels are not kept, or NULL */
    int nrects;                 /* the strip, then the regions */
    SpProbeRect rects[1 + SP_PROBE_REGIONS];    /* in the slot's pixels */
    uint8_t *px;                /* g_malloc'd, or NULL */
} SpProbeShot;
/* the shot (g_new'd) is taken */
void spacebox_ui_probe_shot(SpProbeShot *shot);
/* the presenter's time in the probe at one swap, for the stop line */
void spacebox_ui_probe_cost(int64_t us);

/*
 * The faults of the harness's validation that act in the presenter
 * (ui-fault), read by it without a lock. hold: 1 the next frame submitted is
 * kept in its slot, never shown, 2 one is kept, 0 none (a kept one is let go).
 */
extern int spacebox_ui_fault_delay_ms;
extern int spacebox_ui_fault_late_ms;
extern int spacebox_ui_fault_hold;
extern int spacebox_ui_fault_corrupt_n;     /* swaps left */
extern int spacebox_ui_fault_corrupt_w, spacebox_ui_fault_corrupt_h,
           spacebox_ui_fault_corrupt_amp;
/* a fault acted: a line in probe.jsonl; more is ",\"key\":value..." or NULL */
void spacebox_ui_probe_fault(const char *name, const char *more);

/*
 * The presenter's slots by state (free, writing, ready, reading, displayed,
 * and of the ready ones those a fault keeps) and the oldest ready one's age
 * in microseconds (0 = none). Set by the presenter while it runs.
 */
#define SP_PROBE_SLOT_STATES 6
extern void (*spacebox_ui_probe_slots)(unsigned counts[SP_PROBE_SLOT_STATES],
                                       int64_t *ready_oldest_us);

#endif
