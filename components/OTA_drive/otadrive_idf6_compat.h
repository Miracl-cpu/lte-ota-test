#ifndef OTADRIVE_IDF6_COMPAT_H
#define OTADRIVE_IDF6_COMPAT_H

#include "esp_err.h"
#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#include "esp_rom_md5.h"

static inline void otadrive_compat_md5_init(md5_context_t *context) {
  esp_rom_md5_init(context);
}

static inline void otadrive_compat_md5_update(md5_context_t *context,
                                               const void *data,
                                               uint32_t length) {
  esp_rom_md5_update(context, data, length);
}

static inline esp_err_t otadrive_compat_md5_finish(md5_context_t *context,
                                                   uint8_t *digest) {
  esp_rom_md5_final(digest, context);
  return ESP_OK;
}

#define esp_md5_init otadrive_compat_md5_init
#define esp_md5_update otadrive_compat_md5_update
#define esp_md5_finish otadrive_compat_md5_finish
#endif

#endif
