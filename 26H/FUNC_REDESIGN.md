# 26H 主功能切换逻辑重构 - 实施方案

## 改动进度
- [x] ✅ ball_balance.h: 新增参数+API声明
- [x] ✅ ball_balance.c: SetTarget/前馈/IsPaused实现
- [x] ✅ track.h: 双控参数+时限宏
- [ ] 🔄 main.c: 状态机+OLED (正在执行)
- [ ] ⏳ 编译验证

## 状态设计
- `FuncMode`: MODE_TRACK_PURE(0) / BALL_DEMO(1) / DUAL_8S(2) / DUAL_30S(3) / DUAL_CUSTOM(4)
- `RunState`: STATE_IDLE / RUNNING / ESTOP / CALIBRATION / ZERO_SET / CUSTOM_SAMP

## 按键映射
- KEY_L/R 短按: IDLE下切换功能模式
- KEY 短按: 启动当前功能
- KEY 长按: 任意运行态→急停
- KEY_L 长按: IDLE→标定
- KEY_R 长按: IDLE→零点设置

## OLED统一格式
- 第0行: [8路循迹] [模式名3字符] [状态 RUN/STP/IDL]
- 第1-2行: 运行时间 SS.Ds
- 第3-4行: 电机A/B 目标/实际/PWM
- 第5行: 误差 + 基础速度
- 第6-7行: 平衡状态 E/A/X/T/F (仅F2-F5显示)

## F2 演示流程
1. 切换到F2 → 电机回零, 显示"BALL DEMO"
2. 短按KEY → 目标=+5cm, PID控制球移动
3. 到达±0.8cm容差 → 切换目标=-5cm, 球折返
4. 平衡在-5cm处, 长按KEY复位

| 模式 | 名称 | 循迹 | 平衡球 | 前馈 | 缓启动 | 时限 | 说明 |
|------|------|:----:|:------:|:----:|:------:|------|------|
| **F1** | 纯循迹 | ✅ | ❌ | ❌ | ❌ | 无 | 开机默认，使用现有循迹参数 |
| **F2** | 平衡球演示 | ❌ | ✅ | ❌ | ❌ | 无 | 自主往返 ±5cm，不容差 0.8cm 平衡 |
| **F3** | 双控 8 秒 | ✅ | ✅ | ✅ | ✅ | 8s | 循迹+平衡同时，新参数 |
| **F4** | 双控 30 秒 | ✅ | ✅ | ✅ | ✅ | 30s | 循迹+平衡同时，新参数 |
| **F5** | 自定义平衡 | ✅ | ✅ | ✅ | ✅ | 30s | 手动摆球→采样→定目标→启动 |

---

## 二、按键逻辑

| 按键 | 场景 | 动作 |
|------|------|------|
| KEY (中键) 短按 | IDLE | 激活当前功能模式 |
| KEY (中键) 长按 | 任意运行态 | 紧急停车 → IDLE |
| KEY_L (左键) 短按 | IDLE | 切换到上一个功能模式 |
| KEY_R (右键) 短按 | IDLE | 切换到下一个功能模式 |
| KEY_L 长按 | IDLE | 进入标定模式 (保留) |
| KEY_R 长按 | IDLE | 进入零点设置 (保留) |

---

## 三、F2 平衡球自主演示详细流程

```
初始化: BallBalance_Init(0cm_pixel), 步进电机到原点
阶段1: 等待短按 KEY 启动
阶段2: 目标=+5cm, PID控制球移动到+5cm, 最远不超过+6cm
阶段3: 到达±0.8cm容差后, 立即切换目标=-5cm
阶段4: PID控制球移动到-5cm, 平衡在-5cm处
阶段5: 长按 KEY → 复位到原点, 回到阶段1
```

关键：**不使用 K230 反馈坐标**，只用内部目标值驱动 PID。

---

## 四、加速度前馈 (Feedforward)

小车加减速时对球产生惯性力，需补偿：

```
feedforward_angle = K_FF * chassis_accel_x
```

- `K_FF`: 前馈系数 (需标定, 初值 0.15)
- `chassis_accel_x`: 由底盘电机目标速度变化率计算 (每 10ms 更新)

**底盘加速度计算**：
```
chassis_accel = (current_target_speed - prev_target_speed) / dt
```

---

## 五、缓启动 (Soft Start)

底盘电机启动时，目标速度从 0 线性爬升到目标值：

```
ramp_time = 500ms (可配置)
ramp_step = base_speed / (ramp_time / 10ms)
```

在每个 PID 周期，`effective_base_speed += ramp_step` 直到达到 `base_speed`。

---

## 六、循迹参数 (F2~F5 新参数)

```c
/* F2~F5 共用循迹参数 (区别于 F1 的默认参数) */
#define TRACK_BASE_SPEED_DUAL   800   /* 双控模式基础速度 */
#define TRACK_TURN_LIMIT_DUAL  1200   /* 转弯差速上限 */
#define TRACK_LINE_KP_DUAL     0.06f  /* 循迹 PID_Kp */
#define TRACK_LINE_KI_DUAL     0.0005f
#define TRACK_LINE_KD_DUAL     0.8f
```

---

## 七、F5 自定义目标流程

1. 切换到 F5 → 步进电机回零，提示"Place Ball"
2. 短按 KEY → 连续采样 10 次 K230 X 坐标 (间隔 50ms)
3. 取平均值 → `g_custom_target_x`
4. 调用 `BallBalance_SetTarget(g_custom_target_x)`
5. 启动循迹 + 平衡 (同 F4，时限 30s)

---

## 八、状态机设计

```c
typedef enum {
    FUNC_IDLE = 0,       // 空闲, 显示当前功能名
    FUNC_TRACK_PURE,     // F1: 纯循迹运行中
    FUNC_BALL_DEMO,      // F2: 平衡球演示运行中
    FUNC_DUAL_8S,        // F3: 双控 8s 运行中
    FUNC_DUAL_30S,       // F4: 双控 30s 运行中
    FUNC_DUAL_CUSTOM,    // F5: 自定义平衡运行中
    FUNC_ESTOP,          // 急停
    FUNC_CALIBRATION,    // 标定 (保留)
    FUNC_ZERO_SET,       // 零点设置 (保留)
} FuncState;

typedef enum {
    MODE_TRACK_PURE = 0, // F1
    MODE_BALL_DEMO,      // F2
    MODE_DUAL_8S,        // F3
    MODE_DUAL_30S,       // F4
    MODE_DUAL_CUSTOM,    // F5
    MODE_COUNT           // 模式总数 (5)
} FuncMode;
```

---

## 九、OLED 显示 (与 F1 风格一致)

每 200ms 刷新，格式：

```
F1: "TRACK"     + 8路循迹 + 状态 RUN/STP + 运行时间 + 电机A/B + 误差
F2: "BALL DEMO" + 目标位置 + 当前角度 + 滤波X + 误差 + "KEY:Start"
F3: "DUAL 8s"   + F1 全部信息 + 平衡角度 + 剩余时间 + X/T/F
F4: "DUAL 30s"  + 同 F3
F5: "CUSTOM"    + "Place Ball" + 采样进度 + 目标X + (运行后同 F3)
```

---

## 十、文件变更清单

| 文件 | 变更 |
|------|------|
| `main.c` | 新增 FuncState/FuncMode 枚举, 状态机重构, OLED 显示 |
| `ball_balance.h` | 新增 `BallBalance_SetTarget()`, `BallBalance_SetFeedforward()`, feedforward 参数 |
| `ball_balance.c` | 实现 SetTarget/SetFeedforward, 支持无 K230 自主模式 |
| `track.h` | 新增双控模式参数宏 |
| `track.c` | 支持缓启动 ramp |
| `motor.h` | 新增 `Motor_GetTargetSpeed()` 供前馈使用 |

---

## 十一、实施顺序

- [ ] 1. `ball_balance.h/c`: 新增 SetTarget, 自主模式, 前馈接口
- [ ] 2. `track.h`: 新增双控参数宏
- [ ] 3. `main.c`: 状态机重构 + 模式切换 + OLED 显示
- [ ] 4. 编译验证
