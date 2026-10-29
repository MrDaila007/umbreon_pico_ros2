#include "umb_ap_provision.h"
umb_ap_provision_t umb_ap_provision_init(void) {
    umb_ap_provision_t s = {.stage = UMB_PROV_IDLE, .backoff_ms = UMB_PROV_FIRST_BACKOFF_MS};
    return s;
}
bool umb_ap_provision_due(umb_ap_provision_t s, bool have_password, uint32_t now_ms) {
    /* DONE is final for this boot: a provisioned device never regenerates. */
    if (have_password || s.stage == UMB_PROV_GAVE_UP || s.stage == UMB_PROV_DONE) return false;
    return s.stage != UMB_PROV_BACKOFF || (int32_t)(now_ms - s.retry_ms) >= 0;
}
static umb_ap_provision_t backoff(umb_ap_provision_t s, uint32_t now_ms) {
    s.stage = UMB_PROV_BACKOFF; s.retry_ms = now_ms + s.backoff_ms;
    s.backoff_ms = s.backoff_ms >= UMB_PROV_MAX_BACKOFF_MS / 2u ? UMB_PROV_MAX_BACKOFF_MS : s.backoff_ms * 2u;
    return s;
}
umb_ap_provision_t umb_ap_provision_result(umb_ap_provision_t s, umb_prov_result_t r, uint32_t now_ms) {
    switch (r) {
    case UMB_PROV_RESULT_OK:
        s.writes++; /* fall through */
    case UMB_PROV_RESULT_ALREADY:
        s.stage = UMB_PROV_DONE; s.backoff_ms = UMB_PROV_FIRST_BACKOFF_MS; break;
    case UMB_PROV_RESULT_BUSY:
        s.failures++; s.stage = UMB_PROV_BACKOFF; s.retry_ms = now_ms + UMB_PROV_BUSY_RETRY_MS; break;
    case UMB_PROV_RESULT_NO_ENTROPY:
        s.failures++; s = backoff(s, now_ms); break;
    case UMB_PROV_RESULT_FLASH_FAILED:
        s.failures++; s.writes++;
        if (s.writes >= UMB_PROV_MAX_WRITES) s.stage = UMB_PROV_GAVE_UP;
        else s = backoff(s, now_ms);
        break;
    default: return s;
    }
    s.last = r;
    return s;
}
