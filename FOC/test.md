# FOC 数学库与改动函数 — 测试说明

> 范围：`motorcore/FOC/foc_alg.c` / `foc_alg.h`
> 背景：用 `my_*` 替代 `<math.h>` 的 `sin/cos/fabs/fmod/pow/floor/round`，并调整调用这些实现的业务函数。
> 自动化单元测试：`motorcore/test/test.c`（入口 `test_run_all_math()`，对照 `math.h` 标准库）。

---

## 1. 测试约定

| 项         | 说明                                                                                                           |
| ---------- | -------------------------------------------------------------------------------------------------------------- |
| 参考值     | `float` 版 libm（`sinf`、`powf`、`roundf`、`fmodf`、`floorf`、`fabsf` 等）                       |
| 通用容差   | `EPS = 1e-5f`（RTT 打印常为 `0.000009`）                                                                   |
| 三角容差   | `EPS_TRIG = 5e-3f`（切比雪夫近似；RTT 打印常为 `0.004999`）                                                |
| 幂运算容差 | `max(EPS_POW, REL_POW × \|reference\|)`，`EPS_POW=1e-4`，`REL_POW=1e-3`                                   |
| 通过条件   | `absolute_error <= tolerance`                                                                                |
| 日志       | `SEGGER_RTT_printf` **仅可靠支持 `%s` / `%f`**；数值以 `(double)` + `%f` 输出（默认 6 位小数） |
| 日志间隔   | `TEST_LOG_DELAY_MS`（默认 1 ms，见 `test/test.h`），避免 RTT 缓冲未刷完                                    |

---

## 2. 已实现单元测试（`test/test.c`）

调用方式：在 `main` 或调试入口执行 `test_run_all_math()`，返回失败条数（0 表示全部通过）。

### 2.1 `my_abs`（对照 `fabsf`）

| ID     | 输入       | 参考              | 容差 | 目标机实测                                                                   |
| ------ | ---------- | ----------------- | ---- | ---------------------------------------------------------------------------- |
| ABS-01 | `0.0f`   | `0.0f`          | EPS  | PASSED                                                                       |
| ABS-02 | `3.5f`   | `fabsf(3.5f)`   | EPS  | PASSED                                                                       |
| ABS-03 | `-3.5f`  | `fabsf(-3.5f)`  | EPS  | PASSED                                                                       |
| ABS-04 | `1e-30f` | `fabsf(1e-30f)` | EPS  | PASSED（RTT 显示 `0.000000`）                                              |
| ABS-05 | `-1e30f` | `fabsf(-1e30f)` | EPS  | PASSED；`float` 饱和为 **2147483648**（RTT 显示 `2147483647.xxx`） |

### 2.2 `my_sgn`

| ID     | 输入      | 参考      | 容差 | 目标机实测 |
| ------ | --------- | --------- | ---- | ---------- |
| SGN-01 | `1.0f`  | `1.0f`  | EPS  | PASSED     |
| SGN-02 | `-1.0f` | `-1.0f` | EPS  | PASSED     |
| SGN-03 | `0.0f`  | `0.0f`  | EPS  | PASSED     |

### 2.3 `my_sat`

| ID     | e         | r        | 参考      | 容差 | 目标机实测 |
| ------ | --------- | -------- | --------- | ---- | ---------- |
| SAT-01 | `0.5f`  | `1.0f` | `0.5f`  | EPS  | PASSED     |
| SAT-02 | `2.0f`  | `1.0f` | `1.0f`  | EPS  | PASSED     |
| SAT-03 | `-2.0f` | `1.0f` | `-1.0f` | EPS  | PASSED     |

### 2.4 `my_Limit`

| ID     | value     | high     | low      | 参考     | 容差 | 目标机实测 |
| ------ | --------- | -------- | -------- | -------- | ---- | ---------- |
| LIM-01 | `0.5f`  | `1.0f` | `0.0f` | `0.5f` | EPS  | PASSED     |
| LIM-02 | `2.0f`  | `1.0f` | `0.0f` | `1.0f` | EPS  | PASSED     |
| LIM-03 | `-1.0f` | `1.0f` | `0.0f` | `0.0f` | EPS  | PASSED     |

### 2.5 `my_map`

线性映射：`((Data-in_low)/(in_high-in_low))*(out_high-out_low)+out_low`。

| ID     | Data     | 输入范围   | 输出范围    | 参考               | 容差 | 目标机实测 |
| ------ | -------- | ---------- | ----------- | ------------------ | ---- | ---------- |
| MAP-01 | `0.0f` | `[-1,1]` | `[0,1]`   | **`0.5f`** | EPS  | PASSED     |
| MAP-02 | `1.0f` | `[-1,1]` | `[0,1]`   | `1.0f`           | EPS  | PASSED     |
| MAP-03 | `0.5f` | `[0,10]` | `[0,100]` | `5.0f`           | EPS  | PASSED     |

### 2.6 `my_round` / `my_fast_round`（对照 `roundf`）

| ID     | 函数              | 输入             | 参考                           | 容差     | 目标机实测 |
| ------ | ----------------- | ---------------- | ------------------------------ | -------- | ---------- |
| RND-01 | `my_round`      | `2.3f`         | `2`                          | EPS      | PASSED     |
| RND-02 | `my_round`      | `2.5f`         | `3`                          | EPS      | PASSED     |
| RND-03 | `my_round`      | `-2.5f`        | `-3`                         | EPS      | PASSED     |
| RND-04 | `my_fast_round` | `2.49f`        | `2`                          | 整数相等 | PASSED     |
| RND-05 | `my_fast_round` | `-2.51f`       | `-3`                         | 整数相等 | PASSED     |
| RND-06 | `my_round`      | `-2.3f`        | `-2`                         | EPS      | PASSED     |
| RND-07 | `my_round`      | `0.0f`         | `0`                          | EPS      | PASSED     |
| RND-08 | `my_round`      | `1.4999999e6f` | `1500000`                    | EPS      | PASSED     |
| RND-09 | `my_fast_round` | `3.14f`        | `(int32_t)my_round(3.14f)=3` | 整数相等 | PASSED     |
| RND-10 | `my_fast_round` | `-7.6f`        | `-8`                         | 整数相等 | PASSED     |
| RND-11 | `my_fast_round` | `0.0f`         | `0`                          | 整数相等 | PASSED     |

### 2.7 `my_floor`（对照 `floorf`）

| ID     | 输入      | 参考   | 容差 | 目标机实测 |
| ------ | --------- | ------ | ---- | ---------- |
| FLR-01 | `2.9f`  | `2`  | EPS  | PASSED     |
| FLR-02 | `-2.1f` | `-3` | EPS  | PASSED     |
| FLR-03 | `3.0f`  | `3`  | EPS  | PASSED     |
| FLR-04 | `-3.0f` | `-3` | EPS  | PASSED     |
| FLR-05 | `0.0f`  | `0`  | EPS  | PASSED     |

### 2.8 `my_fmodf`（对照 `fmodf`）

| ID            | x         | y        | 参考                 | 容差     | 目标机实测            |
| ------------- | --------- | -------- | -------------------- | -------- | --------------------- |
| FMOD-01       | `5.5f`  | `2.0f` | `1.5`              | EPS      | PASSED                |
| FMOD-02       | `-5.5f` | `2.0f` | `-1.5`             | EPS      | PASSED                |
| FMOD-03       | `3.0f`  | `0.0f` | `0.0f`（除零保护） | EPS      | PASSED                |
| FMOD-04       | `0.8f`  | `2π`  | `fmodf=0.8`        | EPS      | PASSED                |
| FMOD-04-RANGE | —        | —       | 结果 ∈`[0, 2π)`  | 范围检查 | PASSED；`value=0.8` |

### 2.9 `my_pow`（对照 `powf`）

| ID     | base                              | exp       | 参考           | 容差           | 目标机实测                                     |
| ------ | --------------------------------- | --------- | -------------- | -------------- | ---------------------------------------------- |
| POW-01 | `10.0f`                         | `3.0f`  | `1000.0f`    | EPS            | PASSED                                         |
| POW-02 | `2.0f`                          | `10.0f` | `1024.0f`    | EPS            | PASSED                                         |
| POW-03 | `5.0f`                          | `0.0f`  | `1.0f`       | EPS            | PASSED                                         |
| POW-04 | `2.0f`                          | `-2.0f` | `0.25f`      | EPS            | PASSED                                         |
| POW-05 | `10.0f`                         | `2.5f`  | `316.227752` | `≈0.316`    | PASSED；实测 `316.098937`，误差 `0.128814` |
| POW-06 | `-2.0f`                         | `0.5f`  | `0.0f`       | EPS            | PASSED                                         |
| POW-07 | `0.0f`                          | `-1.0f` | `1.0f`       | EPS            | PASSED                                         |
| POW-08 | `0.5f`                          | `3.0f`  | `0.125`      | `≈0.000125` | PASSED                                         |
| POW-09 | `0.8f`（即 `1-r`，`r=0.2`） | `5.0f`  | `0.327680`   | `≈0.000327` | PASSED（AIS 路径）                             |

### 2.10 `my_sin` / `my_cos`（对照 `sinf` / `cosf`）

输入角在 `[0, 2π)`（扫频经 `Limit_angle_el` 归一化）。

| ID     | 函数       | 角度     | 参考   | 容差     | 目标机实测                                    |
| ------ | ---------- | -------- | ------ | -------- | --------------------------------------------- |
| SIN-01 | `my_sin` | `0`    | `0`  | EPS      | PASSED                                        |
| SIN-02 | `my_sin` | `π/2` | `1`  | EPS_TRIG | PASSED；实测 `1.004501`，误差 `0.004501`  |
| SIN-03 | `my_sin` | `π`   | `0`  | EPS_TRIG | PASSED                                        |
| COS-01 | `my_cos` | `0`    | `1`  | EPS_TRIG | PASSED；实测 `1.004501`，误差 `0.004501`  |
| COS-02 | `my_cos` | `π/2` | `0`  | EPS_TRIG | PASSED                                        |
| COS-03 | `my_cos` | `π`   | `-1` | EPS_TRIG | PASSED；实测 `-1.004501`，误差 `0.004501` |

**扫频（TRIG-SWEEP）**

- 3 个抽样点 + 360 点全周：`sample_count = 726`
- **maximum_error = 0.004501**，**mean_error = 0.000567**，容差 `0.004999` → PASSED

抽样点误差：

| index | angle (rad) | sine_error | cosine_error |
| ----- | ----------- | ---------- | ------------ |
| 0     | 0.1         | 0.000000   | 0.002851     |
| 1     | 1.2         | 0.000692   | 0.000000     |
| 2     | 3.5         | 0.000000   | 0.000743     |

> 峰值误差约 **0.45%**，出现在 ±π/2、0、π 附近。

### 2.11 `my_round_to_decimal`

| ID     | x            | n      | 参考      | 容差 | 目标机实测 |
| ------ | ------------ | ------ | --------- | ---- | ---------- |
| DEC-01 | `3.14159f` | `2`  | `3.14f` | EPS  | PASSED     |
| DEC-02 | `3.145f`   | `2`  | `3.15f` | EPS  | PASSED     |
| DEC-03 | `1.23f`    | `-1` | `1.23f` | EPS  | PASSED     |
| DEC-04 | `0.0f`     | `3`  | `0.0f`  | EPS  | PASSED     |

---

## 3. 目标机验收记录

### 2026-05-26（基础用例 42 项）

```
summary: passed 42, failed 0, total 42
```

### 2026-05-26（含扩展边界，共 58 项）

```
summary: passed 58, failed 0, total 58
```

全部 **58** 项 `my_*` 单元测试在 STM32 目标机（RTT 输出）通过。

### 2026-07-13（校准函数手动回归）

在目标机对 `foc_alg.h`「初始化相关」全部标定 API 进行阻塞 / 非阻塞回归，**同一次上电内重复调用非阻塞版本**，验证 static 累加量在完成态 case 中已正确清理。

| CAL ID | 函数 | block / nonblock | 结果 | 备注 |
| ------ | ---- | ---------------- | ---- | ---- |
| CAL-01 | `update_Ioffset_*` | both | **PASSED** | 1000 次采样平均后 `count` / 累加和归零 |
| CAL-02 | `update_2DIR_sensor_*` | both | **PASSED** | `MotorConfig.DIR` 与速度积分符号一致 |
| CAL-03 | `update_PHASE_*` | both | **PASSED** | `MotorConfig.PHASE` ∈ {1…6}，case 9 状态复位 |
| CAL-04 | `update_pole_pairs_sensor_*` | both | **PASSED** | `MotorConfig.Pole_pairs` 与实测一致 |
| CAL-05 | `update_angle_el_zero_sensor_*` | both | **PASSED** | `angle_el_zero` 写入合理，重复标定无累加 |
| CAL-06 | `update_angle_el_zero_no_sensor_block` | block | **PASSED** | 无传感器单点零点标定 |

```
summary: calibration passed 11, failed 0, total 11
```

日志内容（2026-05-26 数学单元测试）：

```
00> ======== FOC math unit test (versus standard math.h library) ========
00> 
00> --- absolute value: my_abs ---
00> PASSED | test case ABS-01 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case ABS-02 | actual=3.500000 | reference=3.500000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case ABS-03 | actual=3.500000 | reference=3.500000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case ABS-04 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case ABS-05 | actual=2147483647.////// | reference=2147483647.////// | absolute_error=0.000000 | tolerance=0.000009
00> 
00> --- sign function: my_sgn ---
00> PASSED | test case SGN-01 | actual=1.000000 | reference=1.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case SGN-02 | actual=-1.000000 | reference=-1.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case SGN-03 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> 
00> --- saturation: my_sat ---
00> PASSED | test case SAT-01 | actual=0.500000 | reference=0.500000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case SAT-02 | actual=1.000000 | reference=1.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case SAT-03 | actual=-1.000000 | reference=-1.000000 | absolute_error=0.000000 | tolerance=0.000009
00> 
00> --- limiter: my_Limit ---
00> PASSED | test case LIM-01 | actual=0.500000 | reference=0.500000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case LIM-02 | actual=1.000000 | reference=1.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case LIM-03 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> 
00> --- linear map: my_map ---
00> PASSED | test case MAP-01 | actual=0.500000 | reference=0.500000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case MAP-02 | actual=1.000000 | reference=1.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case MAP-03 | actual=5.000000 | reference=5.000000 | absolute_error=0.000000 | tolerance=0.000009
00> 
00> --- round: my_round / my_fast_round ---
00> PASSED | test case RND-01 | actual=2.000000 | reference=2.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case RND-02 | actual=3.000000 | reference=3.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case RND-03 | actual=-3.000000 | reference=-3.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case RND-04 | actual_integer=2.000000 | reference_integer=2.000000
00> PASSED | test case RND-05 | actual_integer=-3.000000 | reference_integer=-3.000000
00> PASSED | test case RND-06 | actual=-2.000000 | reference=-2.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case RND-07 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case RND-08 | actual=1500000.000000 | reference=1500000.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case RND-09 | actual_integer=3.000000 | reference_integer=3.000000
00> PASSED | test case RND-10 | actual_integer=-8.000000 | reference_integer=-8.000000
00> PASSED | test case RND-11 | actual_integer=0.000000 | reference_integer=0.000000
00> 
00> --- floor: my_floor ---
00> PASSED | test case FLR-01 | actual=2.000000 | reference=2.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case FLR-02 | actual=-3.000000 | reference=-3.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case FLR-03 | actual=3.000000 | reference=3.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case FLR-04 | actual=-3.000000 | reference=-3.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case FLR-05 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> 
00> --- floating remainder: my_fmodf ---
00> PASSED | test case FMOD-01 | actual=1.500000 | reference=1.500000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case FMOD-02 | actual=-1.500000 | reference=-1.500000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case FMOD-03 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case FMOD-04 | actual=0.800000 | reference=0.800000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case FMOD-04-RANGE | value=0.800000 in range [0.000000, 6.283185)
00> 
00> --- power: my_pow ---
00> PASSED | test case POW-01 | actual=1000.000000 | reference=1000.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case POW-02 | actual=1024.000000 | reference=1024.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case POW-03 | actual=1.000000 | reference=1.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case POW-04 | actual=0.250000 | reference=0.250000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case POW-05 | actual=316.098937 | reference=316.227752 | absolute_error=0.128814 | tolerance=0.316227
00> PASSED | test case POW-06 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case POW-07 | actual=1.000000 | reference=1.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case POW-08 | actual=0.125000 | reference=0.125000 | absolute_error=0.000000 | tolerance=0.000125
00> PASSED | test case POW-09 | actual=0.327680 | reference=0.327680 | absolute_error=0.000000 | tolerance=0.000327
00> 
00> --- sine and cosine: my_sin / my_cos ---
00> PASSED | test case SIN-01 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case SIN-02 | actual=1.004501 | reference=1.000000 | absolute_error=0.004501 | tolerance=0.004999
00> PASSED | test case SIN-03 | actual=0.000000 | reference=-0.000000 | absolute_error=0.000000 | tolerance=0.004999
00> PASSED | test case COS-01 | actual=1.004501 | reference=1.000000 | absolute_error=0.004501 | tolerance=0.004999
00> PASSED | test case COS-02 | actual=0.000000 | reference=-0.000000 | absolute_error=0.000000 | tolerance=0.004999
00> PASSED | test case COS-03 | actual=-1.004501 | reference=-1.000000 | absolute_error=0.004501 | tolerance=0.004999
00>   sample_point index=0.000000 angle=0.100000 sine_error=0.000000 cosine_error=0.002851
00>   sample_point index=1.000000 angle=1.200000 sine_error=0.000692 cosine_error=0.000000
00>   sample_point index=2.000000 angle=3.500000 sine_error=0.000000 cosine_error=0.000743
00>   sweep_statistics sample_count=726.000000 maximum_error=0.004501 mean_error=0.000567 tolerance=0.004999
00> PASSED | test case TRIG-SWEEP | actual=0.004501 | reference=0.000000 | absolute_error=0.004501 | tolerance=0.004999
00> 
00> --- decimal round: my_round_to_decimal ---
00> PASSED | test case DEC-01 | actual=3.140000 | reference=3.140000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case DEC-02 | actual=3.150000 | reference=3.150000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case DEC-03 | actual=1.230000 | reference=1.230000 | absolute_error=0.000000 | tolerance=0.000009
00> PASSED | test case DEC-04 | actual=0.000000 | reference=0.000000 | absolute_error=0.000000 | tolerance=0.000009
00> 
00> ======== summary: passed 58.000000, failed 0.000000, total 58.000000 ======== 
```

---

## 4. 待补充 / 未自动化用例

| 类别              | 说明                                                                                         |
| ----------------- | -------------------------------------------------------------------------------------------- |
| 业务回归（REG-*） | `Limit_angle`、`Park/Clark`、PID、开环等，需 mock `Motor_HandleTypeDef` 或 golden 对比 |
| 校准回归（CAL-*） | 初始化标定函数已在目标机手动回归（见 §3.3、§5.5），尚无自动化用例                          |

---

## 5. 回归测试 — 被改动过的业务函数

以下条目来自 **`foc_alg.c` 全文检索 `my_*` 调用**（2026-05-26）。**§5.5 校准类**已在目标机手动回归通过（2026-07-13）；其余分组仍待自动化 / 集成验证。

**范围说明**

- **纳入**：FOC 内核中直接或间接调用 `my_*` 的业务函数。
- **排除（不再维护 / 不列入 REG）**：LUT / NLLUT、双编码减速箱相关，以及仅在这些流程中出现的函数。

| 排除函数（含 `my_*` 但不在 REG 范围）                                |
| ---------------------------------------------------------------------- |
| `update_angle_NLLUT`、`Calculate_angle_NLLUT`                      |
| `map_samples_to_lut`、`update_NLLUT_encoder_sensor_block/nonblock` |
| `update_NLLUT_and_angle_el_zero_sensor_block/nonblock`               |
| `update_loopcount_rotor_block`                                       |

**`my_*` 已实现但 `foc_alg.c` 业务层未调用**：`my_sgn`、`my_sat`、`my_round_to_decimal`（仅单元测试覆盖）。

**未使用 `my_*` 的角度函数**（仍属 FOC API，但不在本节 my_* 回归范围）：`Limit_angle_el`（while 归一化）、`Calculate_angle_el`、`Get_angle_el`、`update_angle_el` 等。

---

### 5.1 角度与限幅

| ID     | 函数                   | 使用的 `my_*`                               | 测试要点                                      |
| ------ | ---------------------- | --------------------------------------------- | --------------------------------------------- |
| REG-01 | `Limit_angle`        | `my_abs`、`my_fmodf`                      | `period≈0`、跨周期边界、±π 限幅          |
| REG-02 | `Limit_angle_flange` | 经 `Limit_angle` → `my_abs`/`my_fmodf` | 法兰角 `±π` 限幅                          |
| REG-03 | `update_angle`       | `my_abs`                                    | 角增量 `>0.8×2π` 时不误累加 `angle_all` |

---

### 5.2 坐标变换与扇区

| ID     | 函数                 | 使用的 `my_*`        | 测试要点                                           |
| ------ | -------------------- | ---------------------- | -------------------------------------------------- |
| REG-10 | `Calculate_Park`   | `my_sin`、`my_cos` | 固定 `angle_el` 下 Iα/Iβ→Id/Iq 与 golden 一致 |
| REG-11 | `update_Park`      | 同上（调用链）         | 写入 `motor->MotorAlg.Id/Iq` 正确                |
| REG-12 | `Calculate_Park_N` | `my_sin`、`my_cos` | Uq/Ud→Uα/Uβ 与 golden 一致                      |
| REG-13 | `update_Park_N`    | 同上（调用链）         | 写入 `motor->MotorAlg.Ualpha/Ubeta` 正确         |
| REG-14 | `Calculate_Sector` | `my_abs`             | 六扇区边界（含 `Ualpha→0`）                     |
| REG-15 | `update_Sector`    | 同上（调用链）         | 扇区号与 `Calculate_Sector` 一致                 |

> `Calculate_Clark` / `Calculate_Clark_N` / `update_Clark*` **未调用** `my_*`，不列入本节。

---

### 5.3 PWM 与调制

| ID     | 函数                              | 使用的 `my_*`                                            | 测试要点                                  |
| ------ | --------------------------------- | ---------------------------------------------------------- | ----------------------------------------- |
| REG-20 | `update_pwm`                    | `my_Limit`                                               | 三相占空比 ∈ [0,1]                       |
| REG-21 | `set_pwm` / `set_pwm_nodir`   | `my_Limit`                                               | 输入 Ta/Tb/Tc 限幅后输出正确              |
| REG-22 | `update_svpwm`                  | `my_map`；链：`update_Park_N`、`update_Sector`       | 给定 Uq/Ud 下 Ta/Tb/Tc 映射与 golden 一致 |
| REG-23 | `set_svpwm` / `set_svpwm_dir` | `my_map`；链：`Calculate_Park_N`、`Calculate_Sector` | 单次 SVPWM 输出占空比正确                 |
| REG-24 | `update_spwm` / `set_spwm`    | `my_map`                                                 | 三相电压→占空比线性映射                  |

---

### 5.4 PID 与速度

| ID     | 函数                       | 使用的 `my_*`                      | 测试要点                           |
| ------ | -------------------------- | ------------------------------------ | ---------------------------------- |
| REG-30 | `Calculate_PID`          | `my_Limit`                         | 积分/输出限幅                      |
| REG-31 | `Calculate_PID_IS`       | `my_Limit`                         | 积分分离区间外不累加 I             |
| REG-32 | `Calculate_PID_IS_AIS`   | `my_pow`、`my_abs`、`my_Limit` | AIS 阈值 `sum`、积分分离逻辑     |
| REG-33 | `Calculate_velocity_raw` | `my_abs`                           | 跨 2π unwrap 速度无异常跳变       |
| REG-34 | `update_velocity_raw`    | 同上（调用链）                       | 与 `Calculate_velocity_raw` 一致 |

---

### 5.5 标定（目标机已回归，2026-07-13）

| ID     | 函数                                     | 使用的 `my_*`          | 测试要点                              | 状态 |
| ------ | ---------------------------------------- | ------------------------ | ------------------------------------- | ---- |
| REG-40 | `update_pole_pairs_sensor_block`       | `my_round`、`my_abs` | 极对数估计合理、与开环速度积分一致    | **PASSED** |
| REG-41 | `update_pole_pairs_sensor_nonblock`    | `my_round`、`my_abs` | 完成态清理 `total_time` / `velocity_integral`；重复标定结果一致 | **PASSED** |
| REG-42 | `update_angle_el_zero_sensor_block`    | `my_round`             | 采样索引 `i` / `i_int` 与角度一致 | **PASSED** |
| REG-43 | `update_angle_el_zero_sensor_nonblock` | `my_round`             | case 4 清理 `angle_el_zero_all` 等；重复标定无累加 | **PASSED** |
| REG-44 | `update_2DIR_sensor_block`           | —（链：`my_abs` 经速度计算） | `MotorConfig.DIR` 与转动方向一致 | **PASSED** |
| REG-45 | `update_2DIR_sensor_nonblock`        | —                        | 完成态清理 `total_time` / `velocity_integral` | **PASSED** |
| REG-46 | `update_PHASE_block`                 | —                        | 阻塞封装与 `update_PHASE_nonblock` 结果一致 | **PASSED** |
| REG-47 | `update_PHASE_nonblock`              | —                        | case 9 复位 `time` / `state` / 电流积分 | **PASSED** |
| REG-48 | `update_angle_el_zero_no_sensor_block` | —（链：`update_angle`） | 无传感器单点 `angle_el_zero` 合理 | **PASSED** |
| REG-49 | `update_Ioffset_block`               | —                        | 三相偏置均值写入 `IA/IB/IC_offset_raw` | **PASSED** |
| REG-50 | `update_Ioffset_nonblock`            | —                        | 1000 次累加后清零 `count` 与累加和 | **PASSED** |

> 上表 **11** 项与 §3.3 CAL-01…CAL-06 对应；非阻塞重复调用场景已纳入验收。

---

### 5.6 开环与 Id/Iq 观测

| ID     | 函数                                      | 使用的 `my_*`        | 测试要点                                              |
| ------ | ----------------------------------------- | ---------------------- | ----------------------------------------------------- |
| REG-55 | `ctrl_motor_openloop_angle_el_nonblock` | `my_abs`             | 电角积分终止条件                                      |
| REG-56 | `ctrl_motor_openloop_angle_nonblock`    | `my_abs`             | 机械角积分终止条件                                    |
| REG-57 | `Calculate_IdIq`                        | `my_sin`、`my_cos` | 给定 IA/IB/IC 与 `angle_el` 的 Id/Iq 与 golden 一致 |

> `ctrl_motor_openloop_*_block` 封装对应 `nonblock`，回归可复用 REG-55/56。

---

### 5.7 REG 用例统计

| 分组               | 条数         | 状态 |
| ------------------ | ------------ | ---- |
| 5.1 角度与限幅     | 3            | 待测 |
| 5.2 坐标变换与扇区 | 6            | 待测 |
| 5.3 PWM 与调制     | 5            | 待测 |
| 5.4 PID 与速度     | 5            | 待测 |
| 5.5 标定           | 11           | **已通过（目标机 2026-07-13）** |
| 5.6 开环与 Id/Iq   | 3            | 待测 |
| **合计**     | **33** | **11 / 33 已通过** |

---

## 6. 测试工程与运行

| 文件            | 说明                                                |
| --------------- | --------------------------------------------------- |
| `test/test.h` | `test_run_all_math()` 声明，`TEST_LOG_DELAY_MS` |
| `test/test.c` | 全部**58** 项 `my_*` 单元测试               |

**目标机**：工程链接 `test/test.c`，上电或调试时调用 `test_run_all_math()`。

**主机（可选）**：

```bash
gcc -DUNIT_TEST -I FOC -I test -I others \
    test/test.c FOC/foc_alg.c others/SEGGER_RTT.c others/SEGGER_RTT_printf.c \
    -lm -o test_math && ./test_math
```

---

## 7. 用例统计

| 类别                                 | 条数         | 状态                                    |
| ------------------------------------ | ------------ | --------------------------------------- |
| `my_*` 单元测试（`test/test.c`） | **58** | 目标机已全部通过                        |
| 业务回归（REG-*）                    | **33** | **11 已通过**（§5.5 标定）；其余待自动化 / 集成验证 |
| 校准手动回归（CAL-*）                | **6**  | 目标机已全部通过（2026-07-13）          |

用例构成：基础 **42** + 扩展边界 **16**（ABS×2、RND×6、FLR×3、FMOD×2、POW×2、DEC×1）。

---

## 8. 未纳入本目录（仍依赖 libm）

| 文件                      | 仍用 libm             | 备注                            |
| ------------------------- | --------------------- | ------------------------------- |
| `others/dual_encoder.c` | `fabs`、`fmod`    | 应用层扩展，非 FOC 内核维护范围 |
| `FOC/foc_drv.c`         | `#include <math.h>` | 当前无直接调用                  |

---

## 9. 修订记录

| 日期       | 说明                                                                                                              |
| ---------- | ----------------------------------------------------------------------------------------------------------------- |
| 2026-05-25 | 初版：覆盖 libm→my_* 替代及业务回归目录                                                                          |
| 2026-05-26 | 同步 `test/test.c`；目标机 42/42 通过；修正 MAP-01；补充 RTT `%f` 约束                                        |
| 2026-05-26 | 扩展边界与非整数 pow 用例入 `test/test.c`；目标机 **58/58** 通过；记录 ABS-05 float 饱和与 POW-08/09 实测 |
| 2026-05-26 | 回归测试范围收缩：移除 LUT/NLLUT 与双编码相关 REG 用例（不再作为 FOC 内核维护项）                                 |
| 2026-05-26 | 检索 `foc_alg.c` 补全 my_* 业务调用 REG 清单（26 项，见第 5 节）                                                |
| 2026-07-13 | 全部校准函数（11 API / 6 CAL 组）目标机手动回归通过；§5.5 扩至 11 项并标记 PASSED；REG 合计 33 项（11 已通过） |
