cmake_minimum_required(VERSION 3.16)

# Reuse the LTE manager and ST7920 LCD driver from the repository.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components"
    # Reuse the repository-local ESP-MQTT component.  ESP-IDF 6 no longer
    # ships MQTT as a built-in component, and the sample project must expose
    # this checked-in copy when the component registry is unavailable.
    "${CMAKE_CURRENT_LIST_DIR}/managed_components/espressif__mqtt"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lte_lcd_project)

# OTAdrive 2.0.6 still uses the ESP-IDF 5 ROM MD5 function names.
if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    idf_component_get_property(_otadrive_lib otadrive__otadrive_esp COMPONENT_LIB)
    target_compile_options(
        ${_otadrive_lib}
        PRIVATE "-include${CMAKE_CURRENT_LIST_DIR}/components/OTA_drive/otadrive_idf6_compat.h"
    )
endif()
