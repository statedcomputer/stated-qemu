/*
 * Spacebox: the sink's record of video frames (SPACEBOX_VIDEO_RECORD), kept
 * by the remote GPU (hw/display/virtio-gpu-remote-record.c) and written to at
 * each swap by the Mac's presenter (ui/spacebox-presenter.h). The lines are
 * described in screen/README.md of the spacebox repository ("The sink's
 * record of video frames").
 */

#ifndef UI_SPACEBOX_VIDEO_RECORD_H
#define UI_SPACEBOX_VIDEO_RECORD_H

/* the tickets a frame can carry (as virglrenderer's resources) */
#define SP_REC_TICKETS 4

/* a record is being written; read by the presenter without the lock */
extern bool spacebox_video_record_on;
/*
 * The tickets of the scanout that the guest flushes, for the frame that the
 * display gets now (set and cleared around the flush, as
 * spacebox_frame_due_us; taken by the presenter's submit).
 */
extern uint64_t spacebox_frame_tickets[SP_REC_TICKETS];
extern unsigned spacebox_frame_ntickets;
/* ... and the source's time and the read of the flush's message (the clock
 * of g_get_monotonic_time(), 0 = not known) */
extern int64_t spacebox_frame_flush_dev, spacebox_frame_flush_read;
/* the test pause: the presenter shows nothing for this many ms after its
 * next swap, once (0 = none); the presenter takes it */
extern int spacebox_present_pause_ms;

/* where the presenter reads the frame code, in the scanout's pixels */
bool spacebox_video_rect(int *x, int *y, int *w, int *h);
/* a swap: its return (g_get_monotonic_time()), the frame code read (-1 =
 * none), the frame's tickets, frames dropped since the last swap; the
 * refresh it is shown at (0 = not known), the swap's call, and the display
 * link's tick before it (its refresh and the one it prepares for), all on the
 * same clock */
void spacebox_video_record_swap(uint64_t seq, int64_t t_swap, int code,
                                bool code_ok, const uint64_t *tickets,
                                unsigned n, uint64_t coalesced,
                                int64_t read_us, int64_t t_show,
                                int64_t t_call, int64_t t_vsync,
                                int64_t t_out, int64_t t_due,
                                int64_t t_flush_dev, int64_t t_flush_read);
/* the test pause, from its start to its end */
void spacebox_video_record_paused(int ms, int64_t t0, int64_t t1);

#endif
