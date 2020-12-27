set(TARGET nrf5_sdk)

if(NOT DEFINED DEVICE_DEF)
    message(FATAL_ERROR "DEVICE_DEF is not defined. Example: NRF52832_XXAA")
endif()

add_library(${TARGET} STATIC
    # ble
    components/ble/ble_advertising/ble_advertising.c
    components/ble/ble_db_discovery/ble_db_discovery.c
    components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c
    components/ble/ble_racp/ble_racp.c
    components/ble/ble_radio_notification/ble_radio_notification.c
    components/ble/ble_services/ble_bas/ble_bas.c
    components/ble/ble_services/ble_cts_c/ble_cts_c.c
    components/ble/ble_services/ble_lbs/ble_lbs.c
    components/ble/ble_services/ble_nus/ble_nus.c
    components/ble/common/ble_advdata.c
    components/ble/common/ble_conn_params.c
    components/ble/common/ble_conn_state.c
    components/ble/common/ble_srv_common.c
    components/ble/nrf_ble_gatt/nrf_ble_gatt.c
    components/ble/nrf_ble_gq/nrf_ble_gq.c
    components/ble/nrf_ble_qwr/nrf_ble_qwr.c
    components/ble/nrf_ble_scan/nrf_ble_scan.c
    components/ble/peer_manager/auth_status_tracker.c
    components/ble/peer_manager/gatt_cache_manager.c
    components/ble/peer_manager/gatts_cache_manager.c
    components/ble/peer_manager/id_manager.c
    components/ble/peer_manager/nrf_ble_lesc.c
    components/ble/peer_manager/peer_data_storage.c
    components/ble/peer_manager/peer_database.c
    components/ble/peer_manager/peer_id.c
    components/ble/peer_manager/peer_manager_handler.c
    components/ble/peer_manager/peer_manager.c
    components/ble/peer_manager/pm_buffer.c
    components/ble/peer_manager/security_dispatcher.c
    components/ble/peer_manager/security_manager.c
    
    
    
    components/libraries/atomic/nrf_atomic.c
    
    components/libraries/util/app_error_handler_gcc.c
    components/libraries/util/app_error_weak.c
    components/libraries/util/app_error.c
    components/libraries/util/app_util_platform.c
    components/libraries/util/nrf_assert.c
    components/libraries/util/sdk_mapped_flags.c
    
    # nrfx legacy
    integration/nrfx/legacy/nrf_drv_clock.c
    #integration/nrfx/legacy/nrf_drv_power.c
    #integration/nrfx/legacy/nrf_drv_ppi.c
    #integration/nrfx/legacy/nrf_drv_rng.c
    #integration/nrfx/legacy/nrf_drv_spi.c
    #integration/nrfx/legacy/nrf_drv_spis.c
    #integration/nrfx/legacy/nrf_drv_swi.c
    #integration/nrfx/legacy/nrf_drv_twi.c
    #integration/nrfx/legacy/nrf_drv_uart.c
    
    # nrfx
    #modules/nrfx/drivers/src/nrfx_adc.c
    modules/nrfx/drivers/src/nrfx_clock.c
    #modules/nrfx/drivers/src/nrfx_comp.c
    #modules/nrfx/drivers/src/nrfx_dppi.c
    #modules/nrfx/drivers/src/nrfx_gpiote.c
    #modules/nrfx/drivers/src/nrfx_i2s.c
    #modules/nrfx/drivers/src/nrfx_lpcomp.c
    #modules/nrfx/drivers/src/nrfx_nfct.c
    #modules/nrfx/drivers/src/nrfx_nvmc.c
    #modules/nrfx/drivers/src/nrfx_pdm.c
    #modules/nrfx/drivers/src/nrfx_power.c
    #modules/nrfx/drivers/src/nrfx_ppi.c
    #modules/nrfx/drivers/src/nrfx_pwm.c
    #modules/nrfx/drivers/src/nrfx_qdec.c
    #modules/nrfx/drivers/src/nrfx_qspi.c
    #modules/nrfx/drivers/src/nrfx_rng.c
    #modules/nrfx/drivers/src/nrfx_rtc.c
    #modules/nrfx/drivers/src/nrfx_saadc.c
    #modules/nrfx/drivers/src/nrfx_spi.c
    #modules/nrfx/drivers/src/nrfx_spim.c
    #modules/nrfx/drivers/src/nrfx_spis.c
    #modules/nrfx/drivers/src/nrfx_swi.c
    #modules/nrfx/drivers/src/nrfx_systick.c
    #modules/nrfx/drivers/src/nrfx_temp.c
    #modules/nrfx/drivers/src/nrfx_timer.c
    #modules/nrfx/drivers/src/nrfx_twi.c
    #modules/nrfx/drivers/src/nrfx_twi_twim.c
    #modules/nrfx/drivers/src/nrfx_twim.c
    #modules/nrfx/drivers/src/nrfx_twis.c
    #modules/nrfx/drivers/src/nrfx_uart.c
    #modules/nrfx/drivers/src/nrfx_uarte.c
    #modules/nrfx/drivers/src/nrfx_usbd.c
    #modules/nrfx/drivers/src/nrfx_wdt.c
    #modules/nrfx/drivers/src/prs/nrfx_prs.c
    modules/nrfx/mdk/gcc_startup_nrf52.S
    modules/nrfx/mdk/system_nrf52.c
    
    # freertos
    external/freertos/portable/CMSIS/nrf52/port_cmsis_systick.c
    external/freertos/portable/CMSIS/nrf52/port_cmsis.c
    external/freertos/portable/GCC/nrf52/port.c
    external/freertos/source/croutine.c
    external/freertos/source/event_groups.c
    external/freertos/source/list.c
    external/freertos/source/queue.c
    external/freertos/source/stream_buffer.c
    external/freertos/source/tasks.c
    #external/freertos/source/timers.c
    #external/freertos/source/portable/Common/mpu_wrappers.c
    external/freertos/source/portable/MemMang/heap_4.c
)

target_include_directories(${TARGET}
    PUBLIC
        # Select nrf_soc.h depending from softdevice
        #components/drivers_nrf/nrf_soc_nosd
        components/softdevice/s132/headers
        
        components/libraries/util
        components/toolchain/cmsis/include
        external/freertos/config
        external/freertos/portable/CMSIS/nrf52
        external/freertos/portable/GCC/nrf52
        external/freertos/source/include
        modules/nrfx/mdk
        
    PRIVATE
        .
        #config/nrf52832/config
        
        components/ble/ble_db_discovery
        components/ble/ble_link_ctx_manager
        components/ble/common
        components/ble/nrf_ble_gq
        components/libraries/atomic
        components/libraries/atomic_flags
        components/libraries/balloc
        components/libraries/delay
        components/libraries/experimental_section_vars
        components/libraries/fds
        components/libraries/log
        components/libraries/log/src
        components/libraries/memobj
        components/libraries/queue
        components/libraries/strerror
        components/softdevice/common
        components/softdevice/s132/headers
        
        integration/nrfx
        integration/nrfx/legacy
        modules/nrfx
        modules/nrfx/drivers/include
        modules/nrfx/hal
)

target_compile_definitions(${TARGET}
    PUBLIC
        ${DEVICE_DEF}
    PRIVATE
        __STARTUP_CLEAR_BSS
        __START=main
)
