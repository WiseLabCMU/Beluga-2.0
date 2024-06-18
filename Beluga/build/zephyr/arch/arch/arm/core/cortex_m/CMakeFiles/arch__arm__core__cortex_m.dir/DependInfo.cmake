
# Consider dependencies only in project.
set(CMAKE_DEPENDS_IN_PROJECT_ONLY OFF)

# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  "ASM"
  )
# The set of files for implicit dependencies of each language:
set(CMAKE_DEPENDS_CHECK_ASM
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/__aeabi_read_tp.S" "/home/tom/central_and_peripheral_hr/build/zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/__aeabi_read_tp.S.obj"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/cpu_idle.S" "/home/tom/central_and_peripheral_hr/build/zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/cpu_idle.S.obj"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/exc_exit.S" "/home/tom/central_and_peripheral_hr/build/zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/exc_exit.S.obj"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/fault_s.S" "/home/tom/central_and_peripheral_hr/build/zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/fault_s.S.obj"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/isr_wrapper.S" "/home/tom/central_and_peripheral_hr/build/zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/isr_wrapper.S.obj"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/reset.S" "/home/tom/central_and_peripheral_hr/build/zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/reset.S.obj"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/swap_helper.S" "/home/tom/central_and_peripheral_hr/build/zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/swap_helper.S.obj"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/vector_table.S" "/home/tom/central_and_peripheral_hr/build/zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/vector_table.S.obj"
  )
set(CMAKE_ASM_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_ASM
  "KERNEL"
  "NRF52832_XXAA"
  "PICOLIBC_LONG_LONG_PRINTF_SCANF"
  "_FORTIFY_SOURCE=1"
  "_POSIX_C_SOURCE=200809"
  "__LINUX_ERRNO_EXTENSIONS__"
  "__PROGRAM_START"
  "__ZEPHYR_SUPERVISOR__"
  "__ZEPHYR__=1"
  )

# The include file search paths:
set(CMAKE_ASM_TARGET_INCLUDE_PATH
  "/home/tom/nordic/v2.6.1/zephyr/kernel/include"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/include"
  "/home/tom/nordic/v2.6.1/nrf/drivers/mpsl/clock_control"
  "/home/tom/nordic/v2.6.1/zephyr/include"
  "zephyr/include/generated"
  "/home/tom/nordic/v2.6.1/zephyr/soc/arm/nordic_nrf/nrf52"
  "/home/tom/nordic/v2.6.1/zephyr/soc/common/nordic_nrf/."
  "/home/tom/nordic/v2.6.1/zephyr/soc/arm/nordic_nrf/common/."
  "/home/tom/nordic/v2.6.1/zephyr/subsys/bluetooth"
  "/home/tom/nordic/v2.6.1/zephyr/subsys/settings/include"
  "/home/tom/nordic/v2.6.1/nrf/include"
  "/home/tom/nordic/v2.6.1/nrf/lib/multithreading_lock/."
  "/home/tom/nordic/v2.6.1/nrf/subsys/bluetooth/controller/."
  "/home/tom/nordic/v2.6.1/zephyr/drivers/flash"
  "/home/tom/nordic/v2.6.1/nrf/tests/include"
  "/home/tom/nordic/v2.6.1/modules/hal/cmsis/CMSIS/Core/Include"
  "/home/tom/nordic/v2.6.1/zephyr/modules/cmsis/."
  "/home/tom/nordic/v2.6.1/modules/hal/nordic/nrfx"
  "/home/tom/nordic/v2.6.1/modules/hal/nordic/nrfx/drivers/include"
  "/home/tom/nordic/v2.6.1/modules/hal/nordic/nrfx/mdk"
  "/home/tom/nordic/v2.6.1/zephyr/modules/hal_nordic/nrfx/."
  "/home/tom/nordic/v2.6.1/modules/debug/segger/SEGGER"
  "/home/tom/nordic/v2.6.1/modules/debug/segger/Config"
  "/home/tom/nordic/v2.6.1/modules/crypto/tinycrypt/lib/include"
  "/home/tom/nordic/v2.6.1/nrfxlib/mpsl/fem/common/include"
  "/home/tom/nordic/v2.6.1/nrfxlib/mpsl/fem/nrf21540_gpio/include"
  "/home/tom/nordic/v2.6.1/nrfxlib/mpsl/fem/nrf21540_gpio_spi/include"
  "/home/tom/nordic/v2.6.1/nrfxlib/mpsl/fem/simple_gpio/include"
  "/home/tom/nordic/v2.6.1/nrfxlib/mpsl/fem/include"
  "/home/tom/nordic/v2.6.1/nrfxlib/mpsl/fem/include/protocol"
  "/home/tom/nordic/v2.6.1/nrfxlib/mpsl/include"
  "/home/tom/nordic/v2.6.1/nrfxlib/mpsl/include/protocol"
  "/home/tom/nordic/v2.6.1/nrfxlib/softdevice_controller/include"
  "/home/tom/nordic/v2.6.1/zephyr/lib/libc/common/include"
  )

# The set of dependency files which are needed:
set(CMAKE_DEPENDS_DEPENDENCY_FILES
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/fault.c" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/fault.c.obj" "gcc" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/fault.c.obj.d"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/fpu.c" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/fpu.c.obj" "gcc" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/fpu.c.obj.d"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/irq_init.c" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/irq_init.c.obj" "gcc" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/irq_init.c.obj.d"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/irq_manage.c" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/irq_manage.c.obj" "gcc" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/irq_manage.c.obj.d"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/prep_c.c" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/prep_c.c.obj" "gcc" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/prep_c.c.obj.d"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/scb.c" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/scb.c.obj" "gcc" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/scb.c.obj.d"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/swap.c" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/swap.c.obj" "gcc" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/swap.c.obj.d"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/thread.c" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/thread.c.obj" "gcc" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/thread.c.obj.d"
  "/home/tom/nordic/v2.6.1/zephyr/arch/arm/core/cortex_m/thread_abort.c" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/thread_abort.c.obj" "gcc" "zephyr/arch/arch/arm/core/cortex_m/CMakeFiles/arch__arm__core__cortex_m.dir/thread_abort.c.obj.d"
  )

# Targets to which this target links.
set(CMAKE_TARGET_LINKED_INFO_FILES
  )

# Fortran module output directory.
set(CMAKE_Fortran_TARGET_MODULE_DIR "")
