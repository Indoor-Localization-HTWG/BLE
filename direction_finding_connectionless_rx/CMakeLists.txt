#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(direction_finding_connectionless_rx)

target_sources(app PRIVATE
  src/main.c
  src/aoa_calc.c
  src/aoa_calc.h
  src/ble.c
  src/ble.h
  src/est_pos.c
  src/est_pos.h
  src/test/aoa_calc_test.c
  src/test/aoa_calc_test.h
)
