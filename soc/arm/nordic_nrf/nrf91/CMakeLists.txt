zephyr_compile_definitions_ifdef(CONFIG_SOC_NRF9160 NRF9160_XXAA)

zephyr_sources_ifdef(CONFIG_ARM_MPU_NRF91X mpu_regions.c)
zephyr_sources(soc.c)

