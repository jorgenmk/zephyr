zephyr_compile_definitions_ifdef(CONFIG_SOC_NRF9120_MLM1 NRF9120_XXAA_MLM1)

zephyr_sources(
  soc.c
  )

zephyr_sources_ifdef(CONFIG_ARM_MPU_NRF91X mpu_regions.c)
