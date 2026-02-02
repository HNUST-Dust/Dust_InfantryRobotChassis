# Algorithm 模块

> 目标：提供“纯算法/纯数学”的可复用实现，为上层控制（云台/底盘/执行器）提供 PID、滤波、估计等基础能力。
>
> 原则：尽量平台无关、可预测、可移植；不承担设备驱动、通信、任务调度、Topic 发布等职责。

## 1. 模块职责与边界

**负责：**
- 基础数学与数值工具（角度归一化、限幅、快速数学等）。
- 通用控制算法（PID、滑模控制等）。
- 通用滤波与估计（低通滤波、卡尔曼滤波、四元数 EKF 等）。
- 小型工具算法（斜坡函数、OLS 导数/平滑等）。

**不负责：**
- 与具体硬件或通信协议绑定的逻辑（CAN/UART、传感器驱动、任务创建、Topic 系统）。
- “系统策略”类逻辑（模式切换、守护喂狗判据、故障处理策略）。

## 2. 目录结构（按职责拆分）

- `common/`
  - `alg_common.h`：算法模块的轻量级公共常量与单位换算（尽量不引入平台依赖）。
- `math/`
  - `alg_math.h/.cpp`：数学/角度处理/大小端与求和等工具函数（历史遗留 C 风格 API）。
  - `interpolation.hpp`：插值相关工具。
- `control/`
  - `alg_pid.h/.cpp`：`alg::Pid`（带积分限幅/积分分离/微分先行/微分低通等能力）以及兼容旧接口的包装。
  - `slidingmodec.h/.cpp`：滑模控制（历史实现，当前仍包含 HAL 依赖）。
- `filter/`
  - `low_pass_filter.hpp`：一阶低通滤波（新增 C++ API + 旧接口 wrapper）。
  - `kalman_filter.h/.cpp`：卡尔曼滤波（历史实现，依赖 CMSIS-DSP，内部使用动态内存）。
- `estimation/`
  - `QuaternionEKF.h/.cpp`：四元数姿态 EKF（基于 `kalman_filter` 的扩展与定制）。
- `utils/`
  - `alg_constrain.*`：限幅/死区/环形限幅等。
  - `alg_fast_math.*`：快速数学函数（如快速 sqrt）。
  - `alg_ramp.*`：斜坡函数。
  - `alg_ols.*`：OLS（固定数组实现，避免动态内存）。
  - `alg_debug.*`：算法侧调试辅助（如有）。

## 3. 设计约束（嵌入式友好）

- **平台无关优先：**除少数历史文件外，算法层不应包含 HAL/设备头文件。
- **异常/RTTI：**工程通常禁用异常与 RTTI；算法代码应避免依赖它们。
- **动态内存：**
  - 新增/改造的算法尽量避免动态内存。
  - 但历史模块 `filter/kalman_filter.*` 会在 `Kalman_Filter_Init()` 内部通过 `user_malloc` 分配矩阵空间（在 FreeRTOS 下默认映射到 `pvPortMalloc`，否则为 `malloc`）。该模块目前没有 deinit/释放接口，因此应视为“初始化一次、长期复用”的组件。

## 4. 线程模型与可重入性

- **纯函数/无状态工具：**通常线程安全、可重入（例如多数 `math/*` 与 `utils/*` 的函数）。
- **带内部状态的类：**
  - `alg::Pid`、`alg::LowPassFilter`、`alg::OrdinaryLeastSquares`、`alg::Ramp` 都包含内部状态。
  - 建议每个控制回路/任务持有自己的实例；不要在多个任务之间共享同一个实例（除非自行加锁）。
  - 模式切换或控制对象切换时，应显式调用 `reset()`/`init()` 以避免旧状态污染。
- **全局状态：**
  - `estimation/QuaternionEKF.*` 暴露了全局实例 `QEKF_INS`，默认假设在单一任务上下文中更新与读取。

## 5. 典型使用方式（示例）

### 5.1 PID（C++ 推荐接口）

```cpp
#include "control/alg_pid.h"

alg::Pid pid;
alg::PidConfig cfg;
cfg.kp = 10.0f;
cfg.ki = 0.2f;
cfg.kd = 0.0f;
cfg.dt = 0.001f;
cfg.out_max = 100.0f;

pid.configure(cfg);
pid.reset();

const float u = pid.update(target, feedback);
```

### 5.2 一阶低通滤波

```cpp
#include "filter/low_pass_filter.hpp"

alg::LowPassFilter lpf;
lpf.configure(30.0f /*Hz*/, 0.001f /*s*/);

const float y = lpf.update(x);
```

## 6. 常见误用与注意事项

- **单位混用：**
  - `QuaternionEKF` 输出的 `Yaw/Pitch/Roll` 为角度制（deg），而多数控制回路使用弧度（rad）。使用前需确认单位。
- **滤波参数：**
  - `low_pass_filter.hpp` 中若 `dt<=0` 或 `cutoff_hz<=0`，会退化为“直接输出输入”（`alpha=1`）。这通常意味着配置错误。
- **PID 的角度误差：**
  - `Pid::update_angle()` 会进行角度误差处理；普通 `update()` 不会。
  - 角度控制必须明确“误差归一化范围”（例如 $(-\pi,\pi]$）以避免跨零震荡。
- **卡尔曼滤波初始化：**
  - `Kalman_Filter_Init()` 会分配矩阵与缓冲区；应避免在高频循环中反复调用。
  - 由于缺少释放接口，重复 init 会导致内存持续消耗。
