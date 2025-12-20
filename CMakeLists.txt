if(CONFIG_RADIO_CTRL)
  zephyr_library()

  set(RAL_SOURCES
      lib/drivers/radio/ral/radio_utilities.c
      lib/drivers/radio/ral/ral_sx126x.c
      lib/drivers/radio/ral/ral_sx126x_bsp.c
  )

  set(RALF_SOURCES
      lib/drivers/radio/ralf/ralf_sx126x.c
  )

  set(SX126X_SOURCES
      lib/drivers/radio/sx126x/lr_fhss_mac.c
      lib/drivers/radio/sx126x/sx126x_driver_version.c
      lib/drivers/radio/sx126x/sx126x_hal.c
      lib/drivers/radio/sx126x/sx126x_lr_fhss.c
      lib/drivers/radio/sx126x/sx126x.c
  )

  set(RADIO_SOURCES
      lib/drivers/radio/radio_ctrl.c
  )

  zephyr_library_sources_ifdef(CONFIG_RADIO_CTRL_SX1262
      ${RAL_SOURCES}
      ${RALF_SOURCES}
      ${SX126X_SOURCES}
      ${RADIO_SOURCES}
  )

  zephyr_include_directories(
      include
  )
endif()
