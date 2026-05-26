# =============================================================================
# motorcore 根 CMakeLists.txt（用户可改；CubeMX 一般不覆盖整个文件）
# 配合：CMakePresets.json（configure/build 快捷方式）
#       cmake/gcc-arm-none-eabi.cmake（由 Preset 的 toolchainFile 加载）
#       cmake/stm32cubemx/CMakeLists.txt（CubeMX 生成，HAL/启动文件）
# =============================================================================

# cmake_minimum_required：声明本工程需要的 CMake 最低版本；版本过低会配置失败
cmake_minimum_required(VERSION 3.22)

# -----------------------------------------------------------------------------
# 一、C 语言标准（影响所有 C 源文件的编译方式）
# -----------------------------------------------------------------------------

# set(变量名 值)：定义/修改变量；后续用 ${变量名} 引用
# CMAKE_C_STANDARD：全局 C 标准，11 表示 C11
set(CMAKE_C_STANDARD 11)

# CMAKE_C_STANDARD_REQUIRED ON：找不到 C11 则报错，不静默降级
set(CMAKE_C_STANDARD_REQUIRED ON)

# CMAKE_C_EXTENSIONS ON：允许编译器扩展（如 GNU 扩展）；嵌入式里常用
set(CMAKE_C_EXTENSIONS ON)

# -----------------------------------------------------------------------------
# 二、构建类型（Debug / Release，与 CMakePresets 里的 cache 一致）
# -----------------------------------------------------------------------------

# if(条件) ... endif()：条件执行；NOT 表示取反
# CMAKE_BUILD_TYPE：单配置生成器（Makefiles）下常用；多配置（VS/Xcode）不一定靠它
# 若命令行/Preset 已传入 Release，这里不会改掉
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# -----------------------------------------------------------------------------
# 三、工程名（与 x1.ioc 里 ProjectManager.ProjectName 一致）
# -----------------------------------------------------------------------------

# 自定义变量，给后面 project / add_executable 复用；也可直接写死 x1
set(CMAKE_PROJECT_NAME x1)

# CMAKE_EXPORT_COMPILE_COMMANDS：生成 compile_commands.json，给 clangd/IDE 跳转用
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# -----------------------------------------------------------------------------
# 四、声明 CMake 工程
# -----------------------------------------------------------------------------

# project(名称 [语言...])：定义工程；${CMAKE_PROJECT_NAME} 展开为 x1
# 注意：交叉工具链由 Preset 的 toolchainFile 在 project 之前注入，此处不再 include
project(${CMAKE_PROJECT_NAME})

# message()：配置阶段在终端打印；调试用
message("Build type: " ${CMAKE_BUILD_TYPE})

# enable_language：显式启用 C 与汇编；汇编用于 startup_stm32g474xx.s
# 须在 project 之后（CubeMX 模板顺序）；语言启用后才会找 arm-none-eabi-gcc 编 .s
enable_language(C ASM)

# -----------------------------------------------------------------------------
# 五、定义最终产物：可执行文件（MCU 上跑的是 .elf，再转 hex/bin 烧录）
# -----------------------------------------------------------------------------

# add_executable(目标名 [源文件...])：创建一个“可执行”目标
# 此处只写目标名、暂不列源文件；源文件由 target_sources / 子目录稍后追加
# 链接完成后默认生成：build/Release/x1.elf（具体目录取决于 Preset 的 binaryDir）
add_executable(${CMAKE_PROJECT_NAME})

# -----------------------------------------------------------------------------
# 六、纳入 CubeMX 子工程（HAL、main、启动文件等）
# -----------------------------------------------------------------------------

# add_subdirectory(相对路径)：进入子目录，执行其中的 CMakeLists.txt
# 子目录会：把 MX_Application_Src 挂到 x1、建 STM32_Drivers 对象库、建 stm32cubemx 接口库
add_subdirectory(cmake/stm32cubemx)

# -----------------------------------------------------------------------------
# 七、目标级属性（现代 CMake 推荐：命令都带“目标名”，作用域清晰）
# 语法：target_xxx(目标名 可见性 [参数...])
# PRIVATE：仅本目标编译/链接时使用，不传递给依赖它的其他目标
# -----------------------------------------------------------------------------

# target_link_directories：额外库搜索路径（-L）；当前无第三方 .a，留空占位
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 例：${CMAKE_SOURCE_DIR}/libs
)

# target_sources：给目标 x1 增加要编译的源文件（.c / .s）
# PRIVATE：这些源只参与 x1 的编译，不导出给别的 target
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # --- 用户业务代码（CubeMX 重新生成后请检查此列表是否被清空）---
    FOC/foc_alg.c
    FOC/foc_drv.c
    others/mt6835.c
    others/mt6835_stm32_spi_port.c
    others/MT6816.c
    others/can_handler.c
    others/drv_DRV835X.c
    others/flash.c
    others/SEGGER_RTT.c
    others/SEGGER_RTT_printf.c
    test/test.c
)

# target_include_directories：头文件搜索路径（等价 gcc -I）
# 编 x1 及其 .c 时能找到 foc_drv.h、mt6835.h、arm_math.h 等
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    FOC
    others
    Drivers/CMSIS/DSP/Include
    test
)

# target_compile_definitions：预处理器宏（等价 gcc -D）
# 例：MY_DEBUG=1 ；当前为空，宏 USE_HAL_DRIVER 等由 stm32cubemx 接口库提供
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 例：MY_FEATURE_ENABLE
)

# -----------------------------------------------------------------------------
# 八、链接阶段杂项
# -----------------------------------------------------------------------------

# list(REMOVE_ITEM 列表 项)：从 CMake 内部列表删掉一项
# 避免链接错误的 libob.a（与 C++ 相关；纯 C 工程删掉多数也能编过）
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# target_link_libraries：指定链接时依赖的库或其它 CMake 目标
# stm32cubemx：INTERFACE 库，传递 HAL 头文件路径与宏
# 子目录里 x1 已链 STM32_Drivers，且 STM32_Drivers PUBLIC 依赖 stm32cubemx，故此行可删仍常保留
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# -----------------------------------------------------------------------------
# 九、链接完成后的自定义步骤（POST_BUILD）
# -----------------------------------------------------------------------------

# add_custom_command：为目标添加一条“构建规则”
# TARGET x1：挂在 x1 这个目标上
# POST_BUILD：在 x1.elf 链接成功之后执行
# COMMAND：要执行的命令；可写多条
# ${CMAKE_OBJCOPY}：在 toolchain 文件里设为 arm-none-eabi-objcopy
# $<TARGET_FILE:x1>：生成器表达式，展开为 x1.elf 的完整路径
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Building ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
)

# set_target_properties：设置目标的其它属性
# ADDITIONAL_CLEAN_FILES：执行 make clean / cmake --build --target clean 时额外删除 hex/bin
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    ADDITIONAL_CLEAN_FILES "${CMAKE_PROJECT_NAME}.hex;${CMAKE_PROJECT_NAME}.bin"
)
