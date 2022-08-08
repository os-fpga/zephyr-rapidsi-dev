# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(i2c_master)

if(CTRL_SIMU_EEPROM)
zephyr_compile_options(-DTEST_I2C_EEPROM=0)
else()
zephyr_compile_options(-DTEST_I2C_EEPROM=1)
endif()
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
