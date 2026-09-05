# MotorCore

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

FOC 库。内核只有 `FOC/`：算法与硬件用句柄、函数指针解耦。其余目录是 STM32G4 参考工程，不是最小集。

## 功能

- Clark / Park 及逆变换，输出指针；Park 分 `_theta` / `_sincos`，电流环先 `update_sincos`
- SVPWM、SPWM
- `dt`、`angle_all` unwrap、速度一阶低通、电速度（机械转速 × 极对数）
- 自实现 `my_sin` / `my_cos` 等，不链 `arm_math`
- 位置式 PID（积分分离可选）；句柄内预置位置 / 速度 / id / iq / 混合环
- 电流极限圆、电压极限圆（隐极 Ld=Lq，限 id/iq 指令）；Ud-Uq 极限圆（`u_max/2` 径向，电流环后、SVPWM 前）
- 校准：电流零点、`dir`、`phase`、极对数、电角度零点。阻塞 / 非阻塞成对；`_` 接口由外部传入 `dt` 等
- 开环状态在实例上。单元函数是电角度位置开环（返回 1 进行中 / 0 完成）；机械量只做极对数换算

每台电机一份 `Motor_HandleTypeDef`。

## 结构

`foc_alg` 算法；`foc_drv` 实现 `Motor_DrvTypeDef`，`foc_init` 写默认参数。

句柄四块：`motor_config`（极对数、`dir`±1、`phase` 1–6、`mode_sampling`、限幅与电机参数）、`motor_drv`（PWM / ADC / 角度 / 延时 / `dt`）、`motor_alg`（dq/αβ、sincos、PID）、`motor_data`（偏置、滤波、校准与开环过程量）。

成员名 **大类`__`标签**，如 `openloop__angle_el`。可调参数在 `foc_init` 赋值；阻塞过程量用局部变量。

## 控制链路

观测与外环分两个中断（`stm32g4xx_it.c`）。采样、电角度、PWM 更新相位必须一致。

ADC 注入：`update_i_offset_nonblock` → `update_ia_ib_ic` → `update_dt` → `update_angle` → `update_sincos` → `update_i_alpha_i_beta` → `update_iq_id` → `update_velocity_lpf`

TIM1（约 20 kHz）：电流/速度环写 `uq`/`ud`，`update_svpwm`

本拍已有 `dt` 时走 `_` 接口，勿再 `update_dt`。

```c
ctrl_motor_openloop_reset(&motor);
while (ctrl_motor_openloop_angle_el_nonblock(&motor, this_dt, target, start, vel, uq, ud)) { }
```

## 移植

留 `foc_alg`，换 `foc_drv`。同系列只换引脚时改 `foc_drv.c`。

驱动回调：`set_pwm_a/b/c`（占空比 0~1）、`update_i*_raw` + `cal_i*`、`update_angle_raw` + `cal_angle`（0~2π）、`delay_ms`、`update_dt`（秒，随时基时钟重算 `DT_SCALE_FACTOR`）。

不用 RTT / CAN / `mt6835` 则不链，角度回调换成自己的。

验证：开环确认转向相序 → 电流偏置 → 电流环 → 速度环。

## 参考工程

```
FOC/        算法 + 本板 drv（MIT）
Core/       Cube：main、中断
others/     编码器、栅驱、CAN、FSM、RTT
MDK-ARM/    Keil
```

本板：TIM1 PWM、ADC 注入、SPI MT6835、TIM3 测 `dt`。

`FOC/` 为 [MIT](LICENSE)；HAL / CMSIS / RTT 见 [NOTICE](NOTICE)。
