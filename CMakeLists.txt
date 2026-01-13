# Copyright (c) 2025 Makani Science
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LED_KTD202X)
  if(TARGET drivers__led)
    # Append to the existing Zephyr LED driver library
    target_sources(drivers__led PRIVATE ktd202x.c)
  else()
    # Fallback: Create our own library if the standard one doesn't exist
    zephyr_library()
    zephyr_library_sources(ktd202x.c)
  endif()
endif()
