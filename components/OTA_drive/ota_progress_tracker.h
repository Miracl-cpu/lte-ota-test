#ifndef OTA_PROGRESS_TRACKER_H
#define OTA_PROGRESS_TRACKER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define OTA_PROGRESS_REPORT_INTERVAL_US (1000LL * 1000LL)
#define OTA_DOWNLOAD_STALL_TIMEOUT_US  (60LL * 1000LL * 1000LL)
#define OTA_DOWNLOAD_TOTAL_TIMEOUT_US  (15LL * 60LL * 1000LL * 1000LL)

typedef struct {
    size_t total_bytes;
    size_t received_bytes;
    int last_reported_percent;
    int64_t started_at_us;
    int64_t last_progress_at_us;
    int64_t last_report_at_us;
} ota_progress_tracker_t;

static inline ota_progress_tracker_t ota_progress_tracker_init(size_t total_bytes,
                                                               int64_t now_us) {
    return (ota_progress_tracker_t) {
        .total_bytes = total_bytes,
        .last_reported_percent = -1,
        .started_at_us = now_us,
        .last_progress_at_us = now_us,
        .last_report_at_us = now_us,
    };
}

static inline void ota_progress_tracker_observe(ota_progress_tracker_t *progress,
                                                size_t received_bytes,
                                                int64_t now_us) {
    if (progress != NULL && received_bytes > progress->received_bytes) {
        progress->received_bytes = received_bytes;
        progress->last_progress_at_us = now_us;
    }
}

static inline int ota_progress_tracker_percent(const ota_progress_tracker_t *progress) {
    if (progress == NULL || progress->total_bytes == 0) return 0;
    uint64_t percent = ((uint64_t)progress->received_bytes * 100U) /
                       progress->total_bytes;
    return percent > 100U ? 100 : (int)percent;
}

static inline const char *ota_progress_tracker_timeout(const ota_progress_tracker_t *progress,
                                                       int64_t now_us) {
    if (progress == NULL) return NULL;
    if ((now_us - progress->started_at_us) >= OTA_DOWNLOAD_TOTAL_TIMEOUT_US) {
        return "download_timeout";
    }
    if ((now_us - progress->last_progress_at_us) >= OTA_DOWNLOAD_STALL_TIMEOUT_US) {
        return "download_stalled";
    }
    return NULL;
}

static inline bool ota_progress_tracker_should_report(ota_progress_tracker_t *progress,
                                                       int64_t now_us) {
    if (progress == NULL) return false;
    int percent = ota_progress_tracker_percent(progress);
    bool first = progress->last_reported_percent < 0;
    bool completed_now = percent == 100 && progress->last_reported_percent < 100;
    bool percent_changed = percent != progress->last_reported_percent;
    bool interval_elapsed = (now_us - progress->last_report_at_us) >=
                            OTA_PROGRESS_REPORT_INTERVAL_US;
    if (first || completed_now || (percent_changed && interval_elapsed)) {
        progress->last_reported_percent = percent;
        progress->last_report_at_us = now_us;
        return true;
    }
    return false;
}

#endif /* OTA_PROGRESS_TRACKER_H */
