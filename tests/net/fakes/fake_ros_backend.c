#include "fake_ros_backend.h"
#include <string.h>

static void record(fake_ros_t *f, char op, int id) {
    if (f->len < FAKE_ROS_LOG) { f->op[f->len] = op; f->id[f->len] = id; f->len++; }
}
static bool create(void *ctx, ros_entity_id_t id, const ros_entity_desc_t *d, const ros_session_config_t *cfg) {
    fake_ros_t *f = ctx; (void)d;
    f->creates++; f->last_cfg = *cfg;
    if (f->on_blocking) f->on_blocking(f->user);
    if (id == ROS_ENT_INIT_OPTIONS && f->transport_open) f->creates_while_live_transport++;
    if ((int)id == f->fail_create_id) {
        /* A failed session open may leave the socket owned (closed in release). */
        if (id == ROS_ENT_CONTEXT) f->transport_open = true;
        return false;
    }
    if (id == ROS_ENT_CONTEXT) { f->transport_open = true; if (!f->agent_alive) return false; }
    if (id > ROS_ENT_CONTEXT && !f->agent_alive) return false;
    record(f, 'c', (int)id);
    f->live |= 1u << id;
    return true;
}
static void destroy(void *ctx, ros_entity_id_t id, const ros_entity_desc_t *d) {
    fake_ros_t *f = ctx; (void)d;
    if (!(f->live & (1u << id))) f->destroy_without_create++;
    if (f->live & (1u << id) && id > ROS_ENT_CONTEXT && !f->teardown_begun) f->destroys_before_teardown++;
    record(f, 'd', (int)id);
    f->live &= ~(1u << id);
    if (f->on_destroy) f->on_destroy(f->user, id);
}
static void begin_teardown(void *ctx) {
    fake_ros_t *f = ctx;
    if (fake_ros_count(f, 'd')) f->teardown_after_destroy++;
    f->teardown_begun = true;
    record(f, 't', -1);
}
static bool release_transport(void *ctx) {
    fake_ros_t *f = ctx;
    record(f, 'r', -1);
    if (!f->release_ok) return false;
    f->transport_open = false;
    return true;
}
static void set_generation(void *ctx, uint32_t g) { ((fake_ros_t *)ctx)->generation = g; }
static void session_started(void *ctx) { ((fake_ros_t *)ctx)->session_starts++; }
static ros_spin_result_t spin(void *ctx) {
    fake_ros_t *f = ctx; f->spins++;
    if (f->on_spin) f->on_spin(f->user);
    return f->agent_alive ? f->spin_result : ROS_SPIN_ERROR;
}
static bool ping(void *ctx, uint32_t timeout_ms) {
    fake_ros_t *f = ctx; (void)timeout_ms; f->pings++;
    if (f->on_blocking) f->on_blocking(f->user);
    return f->agent_alive;
}
static bool sync_time(void *ctx, uint32_t timeout_ms, int64_t *epoch_ns, uint64_t *mono_ms) {
    fake_ros_t *f = ctx; (void)timeout_ms; f->syncs++;
    if (f->on_blocking) f->on_blocking(f->user);
    if (!f->agent_alive || !f->sync_ok) return false;
    *epoch_ns = f->epoch_ns; *mono_ms = f->mono_ms; return true;
}
void fake_ros_reset(fake_ros_t *f) {
    memset(f, 0, sizeof *f);
    f->fail_create_id = -1; f->agent_alive = f->release_ok = f->sync_ok = true;
    f->spin_result = ROS_SPIN_OK; f->epoch_ns = 1700000000000000000ll; f->mono_ms = 1000;
}
ros_backend_t fake_ros_backend(fake_ros_t *f) {
    ros_backend_t b = {create, destroy, begin_teardown, release_transport, set_generation,
                       session_started, spin, ping, sync_time, f};
    return b;
}
unsigned fake_ros_count(const fake_ros_t *f, char op) {
    unsigned n = 0;
    for (unsigned i = 0; i < f->len; i++) n += f->op[i] == op;
    return n;
}
