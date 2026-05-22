cmake_minimum_required(VERSION 3.16.0)

# ── ESP-IDF component registration ───────────────────────────
# Place this directory in your project's components/ folder,
# or add it to EXTRA_COMPONENT_DIRS.
#
# 将此目录放入项目的 components/ 文件夹,
# 或添加到 EXTRA_COMPONENT_DIRS 中。

idf_component_register(
    SRCS
        "ina226.c"
        "hal/i2c_hal.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
)
