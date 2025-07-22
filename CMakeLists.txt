# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(oxygen2)

target_sources(app PRIVATE src/main.c src/lcd.c src/fonts.c src/oxygen_sensor.c src/turbidity_sensor.c)



