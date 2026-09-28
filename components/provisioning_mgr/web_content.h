/**
 * @file web_content.h
 * @brief Embedded HTML/CSS/JS content for web UI
 */

#ifndef WEB_CONTENT_H
#define WEB_CONTENT_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get main configuration page HTML
 * @return Pointer to HTML string
 */
const char* web_content_get_index_html(void);

/**
 * @brief Get page content length
 * @return Length in bytes
 */
size_t web_content_get_index_html_len(void);

#ifdef __cplusplus
}
#endif

#endif // WEB_CONTENT_H
