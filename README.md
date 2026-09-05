# MotorCore

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

基于 STM32G474 的永磁同步电机 FOC 固件。算法核在 `FOC/`：坐标变换、SVPWM、PID、极限圆与校准；硬件通过 `Motor_DrvTypeDef` 函数指针接入。其余目录是本板参考工程，不是最小集。

## 功能

- Clark / Park 及逆变换；Park 分 `_theta` / `_sincos`，电流环先 `update_sincos`
- SVPWM、SPWM
- `dt`、机械角 unwrap、速度一阶低通、电速度（机械转速 × 极对数）
- 自实现 `my_sin` / `my_cos` 等，不依赖 `arm_math`
- 位置式 PID（积分分离可选）；句柄内预置位置 / 速度 / id / iq / 混合环
- 电流极限圆、电压极限圆（隐极 `Ld=Lq`，限 id/iq 指令）；Ud-Uq 极限圆（`u_max/2` 径向，电流环后、SVPWM 前）
- 校准：电流零点、`dir`、`phase`、极对数、电角度零点。阻塞 / 非阻塞成对；`_` 接口由外部传入 `dt` 等
- 开环：电角度位置开环（返回 1 进行中 / 0 完成）；机械量只做极对数换算

每台电机一份 `Motor_HandleTypeDef`。

## 参考硬件

| 项目 | 本板配置 |
| --- | --- |
| MCU | STM32G474CETx，Cortex-M4F，HSE 8 MHz → SYSCLK 170 MHz |
| PWM | TIM1 中心对齐，ARR=4250，约 **20 kHz**；CH1/2/3 → PA8/PA9/PA10 |
| 电流采样 | ADC1 注入组，TIM1 CH4 下降沿触发；IA/IB/IC → PA0/PA1/PA2 |
| 时基 | TIM3（PSC=300）测 `dt` |
| 主编码器 | MT6835，SPI2，CS=PB12 |
| 副编码器 | MT6816，SPI3，CS=PA15 |
| 栅驱 | DRV835X，SPI1，CS=PB0；ENABLE/PWML/STB → PB6/PB7/PB9 |
| 总线 | FDCAN1，PA11/PA12，标称 1 Mbps（FD + BRS） |
| 调试 | SWD + SEGGER RTT；TEST1/TEST2 → PB10/PB11 |

`foc_init` 里的默认电机参数（按本板改）：极对数 14、相序 `phase=3`、AB 双电阻采样 `mode_sampling=0x110`、`i_max=20 A`、`u_max=24 V`。

## 目录

```
FOC/          算法 foc_alg + 本板驱动 foc_drv（MIT）
Core/         CubeMX：main、中断、外设初始化
others/       MT6835 / MT6816、DRV835X、Flash、SEGGER RTT
cmake/        工具链与 CubeMX 生成的 CMake
MDK-ARM/      Keil 工程
x1.ioc        STM32CubeMX 工程
```

句柄四块：

| 成员 | 内容 |
| --- | --- |
| `motor_config` | 极对数、`dir`±1、`phase` 1–6、`mode_sampling`、限幅与电机参数 |
| `motor_drv` | PWM / ADC / 角度 / 延时 / `dt` / 日志回调 |
| `motor_alg` | dq/αβ、sincos、PID |
| `motor_data` | 偏置、滤波、校准与开环过程量 |

成员名 **大类`__`标签**，如 `openloop__angle_el`。可调参数在 `foc_init` 赋值。

## 控制链路

观测与外环分两个中断（`Core/Src/stm32g4xx_it.c`）。采样、电角度、PWM 更新相位必须一致。

**ADC 注入完成（约 20 kHz）**

`update_i_offset_nonblock` → `update_iaibic` → `update_dt` → `update_angle` → `update_sincos` → `update_ialpha_ibeta` → `update_iqid` → `update_velocity_lpf`

**TIM1 更新中断**

电流 / 速度 / 位置环写 `uq`/`ud`，再 `update_svpwm`。本拍已有 `dt` 时走 `_` 接口，勿再 `update_dt`。

`main.c` 与 TIM1 回调里有开环、电流环、速度环、三环的注释例程，按需解开。使能控制前先打开 ADC / TIM1 中断：

```c
__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
```

开环位置（非阻塞）：

```c
ctrl_motor_openloop_reset(&motor);
while (ctrl_motor_openloop_angle_el_nonblock(&motor, this_dt, target, start, vel, uq, ud)) { }
```

建议上电顺序：开环确认转向相序 → 电流偏置 → 电流环 → 速度环 → 位置环。

## 构建

需要 [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain)（`arm-none-eabi-gcc` 在 PATH 中）和 CMake ≥ 3.22。

```bash
cmake --preset Debug
cmake --build --preset Debug
```

产物在 `build/Debug/`：`x1.elf`、`x1.hex`、`x1.bin`。Release 把 `Debug` 换成 `Release`。

也可用 Keil：打开 `MDK-ARM/x1.uvprojx`。CubeMX 改引脚后重新生成，再检查根 `CMakeLists.txt` 里用户源文件列表是否被清空。

## 移植

留 `foc_alg`，换 `foc_drv`。同系列只换引脚时改 `foc_drv.c`。

驱动回调：

| 回调 | 约定 |
| --- | --- |
| `set_pwm_a/b/c` | 占空比 0~1 |
| `update_i*_raw` + `cal_i*` | ADC 原始值 → 安培 |
| `update_angle_raw` + `cal_angle` | 原始码 → 机械角 0~2π |
| `delay_ms` | 毫秒延时 |
| `update_dt` | 秒；改时基时钟时重算 `DT_SCALE_FACTOR` |
| `log` | 可选；未绑定则为空操作 |

不用 RTT / CAN / `mt6835` 则不链，角度回调换成自己的。换 PWM 频率时同步改 `A/B/C_PWM_Period` 与 TIM1 ARR。换分流电阻或运放增益时改 `I_ADC_CONV`。

## 许可证

`FOC/` 为 [MIT](LICENSE)。HAL / CMSIS / RTT 及 `others/` 第三方代码见 [NOTICE](NOTICE)。
