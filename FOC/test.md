# FOC 数学库与改动函数 — 待测目录

> 范围：`motorcore/FOC/foc_alg.c` / `foc_alg.h`  
> 背景：用 `my_*` 替代 `<math.h>` 的 `sin/cos/fabs/fmod/pow/floor/round`，并调整调用这些实现的业务函数。  
> 建议：先在 **PC 主机** 用参考实现（`math.h` 或高精度库）对比 `my_*`；再在 **STM32 目标机** 做关键路径抽检。

---

## 1. 测试约定

| 项 | 说明 |
|----|------|
| 浮点比较 | 绝对误差 `ε_abs`，相对误差 `ε_rel`；默认 `ε_abs=1e-5`，`sin/cos/pow` 可放宽到 `1e-4` |
| 参考值 | 主机测试：`float` 版 libm（`sinf`、`powf`、`roundf` 等） |
| 通过 | 所有必测用例满足阈值；改动函数行为与改前 golden 一致或在文档允许偏差内 |
| 记录 | 用例 ID、输入、期望、实测、是否通过 |

---

## 2. 单元测试 — `my_*` 纯函数（按函数）

### 2.1 `my_abs`（替代 `fabsf`）

| ID | 输入 x | 期望 |
|----|--------|------|
| ABS-01 | `0.0f` | `0.0f` |
| ABS-02 | `3.5f` | `3.5f` |
| ABS-03 | `-3.5f` | `3.5f` |
| ABS-04 | `1e-30f` | `1e-30f` |
| ABS-05 | `-1e30f` | `1e30f` |

---

### 2.2 `my_sgn`

| ID | 输入 x | 期望 |
|----|--------|------|
| SGN-01 | `1.0f` | `1` |
| SGN-02 | `-1.0f` | `-1` |
| SGN-03 | `0.0f` | `0` |

---

### 2.3 `my_sat`

| ID | e | r | 期望 |
|----|---|---|------|
| SAT-01 | `0.5f` | `1.0f` | `0.5f` |
| SAT-02 | `2.0f` | `1.0f` | `1.0f` |
| SAT-03 | `-2.0f` | `1.0f` | `-1.0f` |

---

### 2.4 `my_Limit`

| ID | value | high | low | 期望 |
|----|-------|------|-----|------|
| LIM-01 | `0.5f` | `1.0f` | `0.0f` | `0.5f` |
| LIM-02 | `2.0f` | `1.0f` | `0.0f` | `1.0f` |
| LIM-03 | `-1.0f` | `1.0f` | `0.0f` | `0.0f` |

---

### 2.5 `my_map`

| ID | Data | in 范围 | out 范围 | 期望（手算） |
|----|------|---------|----------|--------------|
| MAP-01 | `0.0f` | `[-1,1]` → `[0,1]` | `0.0f` |
| MAP-02 | `1.0f` | `[-1,1]` → `[0,1]` | `1.0f` |
| MAP-03 | `0.5f` | `[0,10]` → `[0,100]` | `5.0f` |

---

### 2.6 `my_round` / `my_fast_round`（替代 `roundf`）

| ID | 函数 | 输入 x | 期望 |
|----|------|--------|------|
| RND-01 | `my_round` | `2.3f` | `2.0f` |
| RND-02 | `my_round` | `2.5f` | `3.0f` |
| RND-03 | `my_round` | `-2.3f` | `-2.0f` |
| RND-04 | `my_round` | `-2.5f` | `-3.0f` |
| RND-05 | `my_round` | `0.0f` | `0.0f` |
| RND-06 | `my_fast_round` | `2.49f` | `2` |
| RND-07 | `my_fast_round` | `-2.51f` | `-3` |
| RND-08 | `my_round` | `1.4999999e6f` | 与 `roundf` 对比（大数） |

一致性：`my_fast_round(x) == (int32_t)my_round(x)`（在 `int32` 可表示范围内）。

---

### 2.7 `my_floor`（替代 `floorf`）

| ID | 输入 x | 期望 |
|----|--------|------|
| FLR-01 | `2.9f` | `2.0f` |
| FLR-02 | `-2.1f` | `-3.0f` |
| FLR-03 | `3.0f` | `3.0f` |
| FLR-04 | `-3.0f` | `-3.0f` |
| FLR-05 | `0.0f` | `0.0f` |

---

### 2.8 `my_fmodf`（替代 `fmodf`）

| ID | x | y | 期望（与 `fmodf` 一致） |
|----|---|---|-------------------------|
| FMOD-01 | `5.5f` | `2.0f` | `1.5f` |
| FMOD-02 | `-5.5f` | `2.0f` | `-1.5f` |
| FMOD-03 | `0.8f` | `2*PI` | 落在 `[0, 2π)` |
| FMOD-04 | `3.0f` | `0.0f` | `0.0f`（除零保护） |

---

### 2.9 `my_pow`（替代 `powf`）

**整数指数（快速路径）**

| ID | base | exp | 期望 |
|----|------|-----|------|
| POW-01 | `10.0f` | `3.0f` | `1000.0f` |
| POW-02 | `2.0f` | `10.0f` | `1024.0f` |
| POW-03 | `5.0f` | `0.0f` | `1.0f` |
| POW-04 | `0.0f` | `2.0f` | `0.0f` |
| POW-05 | `2.0f` | `-2.0f` | `0.25f` |

**非整数指数（`my_ln` + `my_exp` 路径）**

| ID | base | exp | 参考 `powf` | 允许误差 |
|----|------|-----|-------------|----------|
| POW-06 | `0.5f` | `3.0f` | `0.125` | `ε_abs=1e-4` |
| POW-07 | `(1-r)`，`r=0.2` | `n=5` | AIS 公式用例 | `ε_rel=1e-3` |
| POW-08 | `10.0f` | `2.5f` | `powf(10,2.5)` | `ε_rel=1e-3` |

**边界**

| ID | 条件 | 期望 |
|----|------|------|
| POW-09 | `base < 0`，`exp` 非整数 | `0.0f`（当前实现） |
| POW-10 | `base=0`，`exp<0` | `1.0f`（当前实现） |

---

### 2.10 `my_sin` / `my_cos`（替代 `sinf` / `cosf`）

输入需先归一化到 `[0, 2π)`（与 `Limit_angle_el` 一致）。

| ID | 函数 | x (rad) | 参考 | 允许误差 |
|----|------|---------|------|----------|
| SIN-01 | `my_sin` | `0` | `0` | `1e-5` |
| SIN-02 | `my_sin` | `PI/2` | `1` | `1e-4` |
| SIN-03 | `my_sin` | `PI` | `0` | `1e-4` |
| SIN-04 | `my_sin` | `3*PI/2` | `-1` | `1e-4` |
| COS-01 | `my_cos` | `0` | `1` | `1e-4` |
| COS-02 | `my_cos` | `PI/2` | `0` | `1e-4` |
| COS-03 | `my_cos` | `PI` | `-1` | `1e-4` |

扫频（可选）：`x = i * 2π/N`，`i=0..N-1`，`N=360`，统计 max/mean 误差。

---

### 2.11 `my_round_to_decimal`（内部用 `my_pow` + `my_fast_round`）

| ID | x | n | 期望（手算 / `round(x*10^n)/10^n`） |
|----|---|----|-------------------------------------|
| DEC-01 | `3.14159f` | `2` | `3.14f` |
| DEC-02 | `3.145f` | `2` | `3.15f`（或按四舍五入规则） |
| DEC-03 | `1.23f` | `-1` | `1.23f`（n<0 原样返回） |
| DEC-04 | `0.0f` | `3` | `0.0f` |

---

## 3. 回归测试 — 被改动过的业务函数

以下函数 **实现或调用链** 已改为 `my_*`，需与改前 golden 或主机参考模型对比。

### 3.1 角度与归一化

| ID | 函数 | 改动点 | 测试要点 |
|----|------|--------|----------|
| REG-01 | `Limit_angle` | `my_abs`、`my_fmodf` | 周期边界、`period≈0`、跨 ±π |
| REG-02 | `Limit_angle_el` | 间接依赖 `my_fmodf` 调用方 | `[0,2π)` 归一化 |
| REG-03 | `update_angle` | `my_abs` | 大跳变 `>0.8*2π` 不累加 |
| REG-04 | `update_angle_NLLUT` | `my_abs` | 同上 |
| REG-05 | `update_loopcount_rotor_block` | `my_fmodf`、`my_abs` | 减速比角度、`angle_B` 收敛判据 |

---

### 3.2 三角与坐标变换

| ID | 函数 | 改动点 | 测试要点 |
|----|------|--------|----------|
| REG-10 | `Park` / 逆 Park | `my_sin`、`my_cos` | 固定 `angle_el` 下 Id/Iq 与 golden 向量一致 |
| REG-11 | `Clark` / 逆 Clark | `my_sin`、`my_cos` | 同上 |
| REG-12 | `Calculate_sector` | `my_abs` | 六扇区划分边界 Uα/Uβ |

---

### 3.3 PID 与速度

| ID | 函数 | 改动点 | 测试要点 |
|----|------|--------|----------|
| REG-20 | `Calculate_PID_IS_AIS` | `my_pow`、`my_abs` | 固定 `r,n,target,feedback,dt`，`sum` 与改前一致；积分分离阈值 |
| REG-21 | `Calculate_velocity_raw` | `my_abs` | 角速度unwrap，无异常跳变 |

---

### 3.4 标定 / LUT / 索引

| ID | 函数 | 改动点 | 测试要点 |
|----|------|--------|----------|
| REG-30 | `map_samples_to_lut` | `my_floor` | 多组 `N_SAMPLES/N_LUT`：`i` 单调、边界 `i<N_SAMPLES-1`、`i_next` 闭环 |
| REG-31 | `Calculate_angle_NLLUT` | `my_fast_round` | 扇区索引与 LUT 长度一致 |
| REG-32 | `update_pole_pairs_sensor_block` | `my_round`、`my_abs` | 极对数估计整数合理 |
| REG-33 | `update_pole_pairs_sensor_nonblock` | 同上 | 状态机各阶段输出 |
| REG-34 | `update_angle_el_zero_sensor_block` | `my_round` | 采样索引 `i` / `i_int` 一致 |
| REG-35 | `update_angle_el_zero_sensor_nonblock` | `my_round` | 非阻塞流程 |
| REG-36 | `update_NLLUT_and_angle_el_zero_sensor_block` | `my_round`、`map_samples_to_lut` | LUT 填表与零点 |
| REG-37 | `update_NLLUT_and_angle_el_zero_sensor_nonblock` | 同上 | 非阻塞 LUT 更新 |
| REG-38 | NLLUT 后处理（约 2048 行） | `map_samples_to_lut` | 完整 LUT 与改前 bin 对比 |

---

### 3.5 开环与其它

| ID | 函数 | 改动点 | 测试要点 |
|----|------|--------|----------|
| REG-40 | `ctrl_motor_openloop_angle_nonblock` | `my_abs` | 积分终止条件 |
| REG-41 | `Get_angle_el` / `Calculate_angle_el` 链 | `my_sin/cos` 下游 | 端到端电角度 |

---

## 4. 建议测试实现方式

### 4.1 主机单元测试（推荐优先）

```
motorcore/FOC/tests/
  test_my_math.c      # 第 2 节全部 my_*
  test_foc_regress.c  # 第 3 节可隔离函数（需 mock Motor_HandleTypeDef）
```

- 编译：`-DUNIT_TEST` 将 `foc_alg.c` 中硬件相关部分 `#ifndef` 隔离，或只链接数学函数到新 TU。
- 断言宏：`ASSERT_NEAR(a, b, eps)`。
- CI：对比 `powf/sinf/roundf` 生成 `.csv` golden，失败打印用例 ID。

### 4.2 目标机抽检

| 优先级 | 内容 |
|--------|------|
| P0 | 上电后 FOC 电流环稳定；`my_sin/cos` 波形无异常 |
| P0 | 编码器标定流程（极对数、电角度零点、NLLUT）与改前结果一致 |
| P1 | RTT 打印 `Calculate_PID_IS_AIS` 中间量 `sum` |
| P2 | 长时间运行无 NaN/Inf |

---

## 5. 用例统计

| 类别 | 条数（约） |
|------|------------|
| `my_*` 单元测试 | 55+ |
| 业务函数回归 | 20+ |
| **合计** | **75+** |

---

## 6. 未纳入本目录（仍依赖 libm）

| 文件 | 仍用 libm | 备注 |
|------|-----------|------|
| `others/dual_encoder.c` | `fabs`、`fmod` | 若统一风格可另开测试章节 |
| `FOC/foc_drv.c` | `#include <math.h>` | 当前无直接调用，可删头文件后复测编译 |

---

## 7. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-05-25 | 初版：覆盖 libm→my_* 替代及 `foc_alg.c` 调用点回归目录 |
