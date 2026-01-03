# Copyright (c) 2025 Makani Science
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LED_KTD202X ktd202x.c)
