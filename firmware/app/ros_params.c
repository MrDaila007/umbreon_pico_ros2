/* ros_params.c - see ros_params.h. Core-0 owner only; shared state is touched
 * only through the injectable io (copies and one atomic commit). */
#include "ros_params.h"
#include <math.h>
#include <string.h>
#include "shared_state.h"

#define NAME_I(n) {n, n, ROS_PARAM_INT, 0, false}
#define NAME_F(n) {n, n, ROS_PARAM_DOUBLE, 0, false}
#define NAME_B(n) {n, n, ROS_PARAM_BOOL, 0, false}
#define NET_I(n) {n, n, ROS_PARAM_INT, 0, true}
#define RX(id, mode) {"reflex." id "." mode, "reflex." id "." mode, ROS_PARAM_INT, 0, false}
#define RX2(id) RX(id, "manual"), RX(id, "auto")

/* One explicit table; docs/ros-interface.md lists the same 55 entries. Strings
 * (credentials, SSID) and the calibration flag are deliberately absent. */
static const ros_param_desc_t TABLE[] = {
    NAME_I("arming_mode"), NAME_I("cmd_timeout_ms"),
    NAME_F("max_speed_ms"), NAME_F("max_reverse_speed_ms"),
    NAME_F("max_throttle"), NAME_F("max_throttle_rev"),
    NAME_I("min_speed"), NAME_I("max_speed"), NAME_I("min_bspeed"),
    NAME_I("min_point"), NAME_I("max_point"),
    {"steer_trim", "neutral_point", ROS_PARAM_INT, 90, false}, /* alias: neutral_point - 90 */
    NAME_B("servo_reverse"),
    NAME_F("pid_kp"), NAME_F("pid_ki"), NAME_F("pid_kd"), NAME_F("spd_slew"),
    NAME_F("kick_pct"), NAME_I("kick_ms"), NAME_I("corner_kick_us"),
    NAME_I("reverse_brake_cmd"), NAME_I("reverse_brake_ms"),
    NAME_I("encoder_holes"), NAME_F("wheel_diam_m"), NAME_I("tach_glitch_filter_us"),
    NAME_B("bat_enabled"), NAME_F("bat_multiplier"), NAME_F("bat_low"),
    NAME_B("imu_rotate"),
    NET_I("wifi_mode"), NET_I("sta_timeout_s"), NET_I("agent_ip"), NET_I("agent_port"), NET_I("ros_domain_id"),
    RX2("battery"), RX2("collision"), RX2("speed_cap"), RX2("stall"), RX2("impact"), RX2("sensor_fault"),
    NAME_I("ttc_ms"), NAME_I("min_front_mm"), NAME_I("cap_far_mm"), NAME_I("cap_near_mm"),
    NAME_F("cap_min_ms"), NAME_I("collision_sensor_mask"),
    NAME_I("stall_min_us"), NAME_I("stall_ms"), NAME_F("impact_g"),
};
#define N_PARAMS (sizeof TABLE / sizeof TABLE[0])

size_t ros_params_count(void) { return N_PARAMS; }
const ros_param_desc_t *ros_params_desc(size_t i) { return i < N_PARAMS ? &TABLE[i] : NULL; }

const ros_param_desc_t *ros_params_find(const char *name) {
    if (!name) return NULL;
    for (size_t i = 0; i < N_PARAMS; i++)
        if (strcmp(TABLE[i].name, name) == 0) return &TABLE[i];
    return NULL;
}

bool ros_params_range(const ros_param_desc_t *d, double *min, double *max) {
    umb_settings_numeric_info_t info;
    if (!d || !min || !max || !umb_settings_numeric_info(d->key, &info)) return false;
    *min = info.min - d->bias;
    *max = info.max - d->bias;
    return true;
}

bool ros_params_read(const umb_settings_t *s, const ros_param_desc_t *d, ros_param_value_t *out) {
    if (!s || !d || !out) return false;
    *out = (ros_param_value_t){.type = d->type};
    if (d->type == ROS_PARAM_DOUBLE) return umb_settings_get_float(s, d->key, &out->d);
    int64_t v;
    if (!umb_settings_get_int(s, d->key, &v)) return false;
    if (d->type == ROS_PARAM_BOOL) out->b = v != 0;
    else out->i = v - d->bias;
    return true;
}

/* The candidate settings for `v`, validated by the core rules; type and range
 * errors are told apart from rejection of the value itself. */
static ros_param_result_t apply_value(const umb_settings_t *in, const ros_param_desc_t *d,
                                      const ros_param_value_t *v, umb_settings_t *out) {
    if (!v || v->type != d->type) return ROS_PARAM_BAD_TYPE;
    bool ok;
    switch (d->type) {
    case ROS_PARAM_DOUBLE: ok = umb_settings_set_float(in, d->key, v->d, out); break;
    case ROS_PARAM_BOOL: ok = umb_settings_set_int(in, d->key, v->b ? 1 : 0, out); break;
    default:
        /* The bias shift must not overflow before the core range check. */
        ok = (d->bias == 0 || (v->i >= INT32_MIN && v->i <= INT32_MAX)) &&
             umb_settings_set_int(in, d->key, v->i + d->bias, out);
        break;
    }
    return ok ? ROS_PARAM_OK : ROS_PARAM_BAD_VALUE;
}

bool ros_params_matches(const umb_settings_t *s, const ros_param_desc_t *d, const ros_param_value_t *v) {
    umb_settings_t candidate;
    if (!s || !d || !v || apply_value(s, d, v, &candidate) != ROS_PARAM_OK) return false;
    return umb_settings_equal(s, &candidate);
}

/* ---- shared settings io ---------------------------------------------------- */

static uint32_t shared_snapshot(umb_settings_t *out) { return shared_settings_snapshot(out); }
static bool shared_commit(const umb_settings_t *s, uint32_t revision) {
    return shared_commit_settings(s, revision, false); /* never the armed mode-only path */
}
const ros_params_io_t *ros_params_shared_io(void) {
    static const ros_params_io_t io = {shared_snapshot, shared_commit, shared_settings_write_allowed};
    return &io;
}

void ros_params_init(ros_params_t *p, const ros_params_io_t *io) {
    memset(p, 0, sizeof *p);
    p->io = io;
}

ros_param_result_t ros_params_set(ros_params_t *p, const char *name, const ros_param_value_t *v) {
    const ros_param_desc_t *d = ros_params_find(name);
    ros_param_result_t r = d ? ROS_PARAM_CONFLICT : ROS_PARAM_UNKNOWN;
    for (unsigned attempt = 0; d && attempt < ROS_PARAMS_COMMIT_ATTEMPTS; attempt++) {
        umb_settings_t current, next;
        uint32_t revision = p->io->snapshot(&current);
        r = apply_value(&current, d, v, &next);
        if (r != ROS_PARAM_OK) break;
        if (!p->io->writable()) { r = ROS_PARAM_BUSY; break; }
        if (p->io->commit(&next, revision)) { r = ROS_PARAM_OK; break; }
        /* Refused: either a gate closed meanwhile or the revision moved. */
        if (!p->io->writable()) { r = ROS_PARAM_BUSY; break; }
        p->set_conflicts++;
        r = ROS_PARAM_CONFLICT;
    }
    if (r == ROS_PARAM_OK) p->set_ok++; else p->set_rejected++;
    return r;
}

bool ros_params_on_modification(ros_params_t *p, const char *name, bool has_old, const ros_param_value_t *new_value) {
    if (p->refreshing) return true; /* our own write-back of settings into the server */
    if (!has_old || !new_value) return false; /* creation and deletion are not supported */
    return ros_params_set(p, name, new_value) == ROS_PARAM_OK;
}

/* ---- reconciliation ---------------------------------------------------------- */

void ros_params_invalidate(ros_params_t *p) {
    p->cache_valid = false;
    p->pass_active = false;
}

static void start_pass(ros_params_t *p, const umb_settings_t *s, uint32_t revision) {
    p->pass = *s;
    p->pass_rev = revision;
    p->cursor = 0;
    p->pass_active = true;
}

static bool refresh_one(ros_params_t *p, const ros_param_sink_t *sink, const ros_param_desc_t *d) {
    ros_param_value_t want, have;
    if (!ros_params_read(&p->pass, d, &want)) return false;
    if (sink->get(sink->ctx, d, &have) && have.type == want.type && ros_params_matches(&p->pass, d, &have))
        return true;
    p->refreshing = true;
    bool ok = sink->put(sink->ctx, d, &want);
    p->refreshing = false;
    if (ok) p->refresh_writes++;
    return ok;
}

bool ros_params_reconcile(ros_params_t *p, const ros_param_sink_t *sink, unsigned budget, unsigned writes) {
    umb_settings_t latest;
    uint32_t revision = p->io->snapshot(&latest);
    if (!sink || !sink->get || !sink->put) return false;
    if (p->cache_valid && revision == p->cached_rev) return true;
    if (!p->pass_active) start_pass(p, &latest, revision);
    else if (revision != p->pass_rev) { p->refresh_restarts++; start_pass(p, &latest, revision); }
    uint32_t written_before = p->refresh_writes;
    for (unsigned n = 0; n < budget && p->cursor < N_PARAMS; n++, p->cursor++) {
        if (p->refresh_writes - written_before >= writes) break;
        if (!refresh_one(p, sink, &TABLE[p->cursor])) {
            p->refresh_failures++;
            p->pass_active = false; /* retry the whole pass from a fresh snapshot */
            p->cache_valid = false;
            memset(&latest, 0, sizeof latest);
            return false;
        }
    }
    if (p->cursor < N_PARAMS) return false;
    p->pass_active = false;
    p->cached_rev = p->pass_rev;
    p->cache_valid = true;
    return true;
}
