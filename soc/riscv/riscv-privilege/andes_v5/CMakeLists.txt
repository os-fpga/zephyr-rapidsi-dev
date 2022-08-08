# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${CONFIG_SOC})

zephyr_sources(
  start.S
  soc_irq.S
)

zephyr_include_directories(
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/riscv/include
)

zephyr_sources_ifdef(CONFIG_SOC_ANDES_V5_PMA pma.c)
zephyr_sources_ifdef(CONFIG_SOC_ANDES_V5_L2_CACHE l2_cache.c)
zephyr_sources_ifdef(CONFIG_SOC_ANDES_V5_PLIC_SW plic_sw.c)

# Note: AndeStar V5 DSP needs custom Andes V5 toolchain
if(CONFIG_SOC_ANDES_V5_HWDSP)
  zephyr_cc_option(-mext-dsp)
endif()

# For awareness debugging auto-detection feature
zephyr_ld_options(-Wl,--defsym=__rtos_signature_zephyr_v2_4_0=0x0)

# Add external module
include(module.cmake)

if(CROSS_COMPILE MATCHES "newlib")
  set(TOOLCHAIN_HAS_NEWLIB ON CACHE BOOL "True if toolchain supports newlib")
else()
  set(TOOLCHAIN_HAS_NEWLIB OFF CACHE BOOL "True if toolchain supports newlib")
endif()
