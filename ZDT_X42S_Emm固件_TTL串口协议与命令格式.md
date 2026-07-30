# 📡 ZDT_X42S Emm 固件 TTL 串口协议

## 🔧 接口参数

| 项目 | 定义 |
|---|---|
| 接线 | 主机 `TX → R/A/H`，主机 `RX ← T/B/L`，`GND ↔ Gnd` |
| 波特率 | 默认 `115200`；可选 `9600/19200/25000/38400/57600/115200/256000/512000/921600` |
| 数据格式 | 二进制字节流，HEX 发送/接收；数据位、奇偶校验位、停止位手册未说明 |
| 地址 | `01-FF`；默认 `01`；`00` 为广播地址 |
| 多机接线 | 所有 `R/A/H` 并联至主机 TX，所有 `T/B/L` 并联至主机 RX，所有设备共地，ID 不重复 |

## 📡 帧格式

```text
主机发送：[Addr] [Code] [Data...] [Chk]
电机返回：[Addr] [Code] [Data/Status...] [Chk]
```

| 字段 | 定义 |
|---|---|
| `Addr` | 1 字节地址 |
| `Code` | 1 字节功能码 |
| `Data` | 0-N 字节数据 |
| `Chk` | 固定 `6B`、XOR 或 CRC-8 时为 1 字节；Modbus CRC-16 为 2 字节 |

| 状态 | 定义 |
|---|---|
| `02` | 命令正确 |
| `12/22` | 回零时已在零点或限位已触发 |
| `E2` | 参数错误、条件不满足或保护状态 |
| `EE` | 帧格式错误 |
| `9F` | 动作完成 |

| 应答模式 | 值 | 返回规则 |
|---|---:|---|
| `None` | `00` | 不返回控制命令确认或完成帧 |
| `Receive` | `01` | 仅返回接收确认，默认 |
| `Reached` | `02` | 仅返回动作完成帧 |
| `Both` | `03` | 返回接收确认和动作完成帧 |
| `Other` | `04` | 位置命令返回完成帧，其他控制命令返回确认帧 |

```text
位置到达：[Addr] FD 9F [Chk]
回零完成：[Addr] 9A 9F [Chk]
```

## 📐 字段编码

| 字段 | 编码 |
|---|---|
| `u8/u16/u32/i32` | 1/2/4/4 字节；多字节高位在前 |
| `Store` | `00` 不保存；`01` 掉电保存 |
| `Dir` | `00` CW；`01` CCW |
| `Sync` | `00` 立即执行；`01` 缓存并等待同步触发 |
| `Sign` | `00` 正；`01` 负；温度符号相反：`00` 负、`01` 正 |
| `Speed` | `u16`，默认单位 RPM，范围 `0000-0BB8` |
| `Acc` | `u8`，`00-FF`；`00` 直接启停；非零时 `Δt=(256-Acc)×50 us/RPM` |
| Emm 位置 | `角度(°)=位置值×360/65536` |
| Emm 脉冲 | 默认 1.8° 电机、16 细分时 `3200 pulse=360°` |

## 🎮 动作与运动命令

| 功能 | 发送 | 返回 |
|---|---|---|
| 编码器校准 | `[Addr] 06 45 [Chk]` | `[Addr] 06 02/E2/EE [Chk]` |
| 重启 | `[Addr] 08 97 [Chk]` | `[Addr] 08 02/E2/EE [Chk]` |
| 当前位置清零 | `[Addr] 0A 6D [Chk]` | `[Addr] 0A 02/E2/EE [Chk]` |
| 解除堵转/过热/过流保护 | `[Addr] 0E 52 [Chk]` | `[Addr] 0E 02/E2/EE [Chk]` |
| 恢复出厂设置 | `[Addr] 0F 5F [Chk]` | `[Addr] 0F 02/E2/EE [Chk]` |
| 电机使能 | `[Addr] F3 AB [Enable] [Sync] [Chk]` | `[Addr] F3 02/E2/EE [Chk]` |
| Emm 速度模式 | `[Addr] F6 [Dir] [Speed:u16] [Acc] [Sync] [Chk]` | `[Addr] F6 02/12/22/E2/EE [Chk]` |
| Emm 位置模式 | `[Addr] FD [Dir] [Speed:u16] [Acc] [Pulse:u32] [PosMode] [Sync] [Chk]` | `[Addr] FD 02/12/22/9F/E2/EE [Chk]` |
| 快速位置参数 | `[Addr] F1 [Speed:u16] [Acc] [PosMode] [Sync] [Chk]` | `[Addr] F1 02/12/22/9F/E2/EE [Chk]` |
| 快速位置脉冲 | `[Addr] FC [Pulse:i32] [Chk]` | `[Addr] FC 02/12/22/9F/E2/EE [Chk]` |
| 立即停止 | `[Addr] FE 98 [Sync] [Chk]` | `[Addr] FE 02/E2/EE [Chk]` |
| 触发同步运动 | `[Addr] FF 66 [Chk]` | `[Addr] FF 02/E2/EE [Chk]` |

| 参数 | 值 |
|---|---|
| `Enable` | `00` 失能；`01` 使能 |
| `PosMode` | `00` 相对上一目标；`01` 绝对坐标；`02` 相对实时位置 |
| 快速位置固件要求 | V2.0.0 以上 |

### 多电机复合命令

```text
00 AA [TotalLen:u16] [完整子命令1] [完整子命令2] ... [Chk]
```

`TotalLen` 为外层帧总字节数；子命令保留各自地址和校验；外层帧再附加校验；Modbus-RTU 不支持。

## 📌 回零命令

| 功能 | 发送 | 返回 |
|---|---|---|
| 设置单圈零点 | `[Addr] 93 88 [Store] [Chk]` | `[Addr] 93 02/E2/EE [Chk]` |
| 触发回零 | `[Addr] 9A [HomeMode] [Sync] [Chk]` | `[Addr] 9A 02/12/9F/E2/EE [Chk]` |
| 中断回零 | `[Addr] 9C 48 [Chk]` | `[Addr] 9C 02/E2/EE [Chk]` |
| 读取回零状态 | `[Addr] 3B [Chk]` | `[Addr] 3B [HomeStatus] [Chk]` |
| 读取回零参数 | `[Addr] 22 [Chk]` | 见参数帧 |
| 修改回零参数 | 见参数帧 | `[Addr] 4C 02/E2/EE [Chk]` |

| `HomeMode` | 定义 |
|---:|---|
| `00` | 单圈就近回零 |
| `01` | 单圈方向回零 |
| `02` | 无限位碰撞回零 |
| `03` | 限位回零 |
| `04` | 回到绝对坐标零点 |
| `05` | 回到上次掉电位置 |

```text
读取返回：[Addr] 22 [HomeMode] [Dir] [Speed:u16] [TimeoutMs:u32] [HitSpeed:u16] [HitCurrent:u16] [HitTimeMs:u16] [PowerOnHome] [Chk]
修改发送：[Addr] 4C AE [Store] [HomeMode] [Dir] [Speed:u16] [TimeoutMs:u32] [HitSpeed:u16] [HitCurrent:u16] [HitTimeMs:u16] [PowerOnHome] [Chk]
```

| 参数 | 范围/单位 |
|---|---|
| `Speed/HitSpeed` | `0-3000 RPM` |
| `TimeoutMs` | `u32 ms` |
| `HitCurrent` | `u16 mA` |
| `HitTimeMs` | `u16 ms` |
| `PowerOnHome` | `00` 关闭；`01` 开启 |

| `HomeStatus` 位 | 掩码 | 定义 |
|---:|---:|---|
| 0 | `01` | `Enc_Rdy` 编码器正常 |
| 1 | `02` | `Cal_Rdy` 校准完成 |
| 2 | `04` | `Org_SF` 正在回零 |
| 3 | `08` | `Org_CF` 回零失败 |
| 4 | `10` | `Otp_TF` 过热保护 |
| 5 | `20` | `Ocp_TF` 过流保护 |

## 📊 系统参数读取

返回格式均为 `[Addr] [Code] [返回数据] [Chk]`。

| 功能 | 发送 | 返回数据 |
|---|---|---|
| 固件/硬件版本 | `[Addr] 1F [Chk]` | `[FW:u16] [HWSeries/Type:u8] [HWVer:u8]` |
| 相电阻/相电感 | `[Addr] 20 [Chk]` | `[PhaR:u16 mΩ] [PhaL:u16 µH]` |
| 总线电压 | `[Addr] 24 [Chk]` | `[VBus:u16 mV]` |
| 总线电流 | `[Addr] 26 [Chk]` | `[CBus:u16 mA]` |
| 相电流 | `[Addr] 27 [Chk]` | `[CPha:u16 mA]` |
| 线性化编码器 | `[Addr] 31 [Chk]` | `[Encoder:u16]`，`0-65535=0-360°` |
| 输入脉冲数 | `[Addr] 32 [Chk]` | `[Sign] [Pulse:u32]` |
| 目标位置 | `[Addr] 33 [Chk]` | `[Sign] [Position:u32]` |
| 实时设定目标位置 | `[Addr] 34 [Chk]` | `[Sign] [Position:u32]` |
| 实时转速 | `[Addr] 35 [Chk]` | `[Sign] [Speed:u16 RPM]` |
| 实时位置 | `[Addr] 36 [Chk]` | `[Sign] [Position:u32]` |
| 位置误差 | `[Addr] 37 [Chk]` | `[Sign] [Error:u32]` |
| 驱动温度 | `[Addr] 39 [Chk]` | `[TempSign] [Temp:u8 °C]` |
| 电机状态 | `[Addr] 3A [Chk]` | `[MotorStatus]` |
| 回零状态 | `[Addr] 3B [Chk]` | `[HomeStatus]` |
| 回零状态+电机状态 | `[Addr] 3C [Chk]` | `[HomeStatus] [MotorStatus]` |
| 引脚状态 | `[Addr] 3D [Chk]` | `[PinStatus]` |

```text
定时返回：[Addr] 11 18 [InfoCode] [PeriodMs:u16] [Chk]
PeriodMs=0000：停止返回；返回格式为 InfoCode 对应读取帧。
```

| `MotorStatus` 位 | 掩码 | 定义 |
|---:|---:|---|
| 0 | `01` | `Ens_TF` 已使能 |
| 1 | `02` | `Prf_TF` 已到位 |
| 2 | `04` | `Cgi_TF` 堵转条件成立 |
| 3 | `08` | `Cgp_TF` 堵转保护 |
| 4 | `10` | `Esi_LF` 左限位高电平 |
| 5 | `20` | `Esi_RF` 右限位高电平 |
| 7 | `80` | `Oac_TF` 掉电标志 |

| `PinStatus` 位 | 掩码 | 定义 |
|---:|---:|---|
| 0 | `01` | `En_Pin` |
| 2 | `04` | `Stp_Pin` |
| 4 | `10` | `Dir_Pin` |
| 5 | `20` | `Dir_OM`：`0` 输入，`1` 输出 |

## 🎛️ 驱动参数读写

写命令返回 `[Addr] [Code] 02/E2/EE [Chk]`。

| 功能 | 发送/读取 | 参数 |
|---|---|---|
| 修改 ID | `[Addr] AE 4B [Store] [ID] [Chk]` | `ID=01-FF` |
| 修改细分 | `[Addr] 84 8A [Store] [Microstep] [Chk]` | `01-FF=1-255`，`00=256` |
| 修改掉电标志 | `[Addr] 50 [Flag] [Chk]` | `00/01` |
| 读取选项状态 | `[Addr] 1A [Chk]` | 返回 `[OptionStatus]` |
| 修改电机类型 | `[Addr] D7 35 [Store] [MotorType] [Chk]` | `19=1.8°`，`32=0.9°` |
| 修改固件类型 | `[Addr] D5 69 [Store] [FWType] [Chk]` | `00=X`，`01=Emm`，`02=Emm Turbo` |
| 修改开/闭环 | `[Addr] 46 69 [Store] [Mode] [Chk]` | `00/01=开环/闭环` |
| 修改正方向 | `[Addr] D4 60 [Store] [Dir] [Chk]` | `00/01=CW/CCW` |
| 锁定按键 | `[Addr] D0 B3 [Store] [Lock] [Chk]` | `00/01=解锁/锁定` |
| 速度值缩小 10 倍 | `[Addr] 4F 71 [Store] [Scale] [Chk]` | `00/01=关闭/开启` |
| 开环工作电流 | `[Addr] 44 33 [Store] [Current:u16] [Chk]` | `0-5000 mA` |
| 闭环堵转最大电流 | `[Addr] 45 66 [Store] [Current:u16] [Chk]` | `0-5000 mA` |
| 读取 Emm PID | `[Addr] 21 [Chk]` | 返回 `[Kp:u32] [Ki:u32] [Kd:u32]` |
| 修改 Emm PID | `[Addr] 4A C3 [Store] [Kp:u32] [Ki:u32] [Kd:u32] [Chk]` | - |
| 读取 DMX512 参数 | `[Addr] 49 78 [Chk]` | 见 DMX 帧 |
| 修改 DMX512 参数 | 见 DMX 帧 | `Code=D9` |
| 读取位置窗口 | `[Addr] 41 [Chk]` | 返回 `[Window:u16]`，单位 `0.1°` |
| 修改位置窗口 | `[Addr] D1 07 [Store] [Window:u16] [Chk]` | - |
| 读取过热/过流阈值 | `[Addr] 13 [Chk]` | 返回 `[Temp:u16] [Current:u16] [Time:u16]` |
| 修改过热/过流阈值 | `[Addr] D3 56 [Store] [Temp:u16] [Current:u16] [Time:u16] [Chk]` | `°C/mA/ms` |
| 读取心跳时间 | `[Addr] 16 [Chk]` | 返回 `[TimeMs:u32]` |
| 修改心跳时间 | `[Addr] 68 38 [Store] [TimeMs:u32] [Chk]` | `0` 关闭 |
| 读取积分限幅 | `[Addr] 23 [Chk]` | 返回 `[Limit:u32]` |
| 修改积分限幅 | `[Addr] 4B 57 [Store] [Limit:u32] [Chk]` | - |
| 读取碰撞回零返回角度 | `[Addr] 3F [Chk]` | 返回 `[Angle:u16]`，单位 `0.1°` |
| 修改碰撞回零返回角度 | `[Addr] 5C AC [Store] [Angle:u16] [Chk]` | `0` 为电流检测返回 |
| 广播读取 ID | `00 15 [Chk]` | 返回 `[Addr] 15 [Addr] [Chk]` |
| 锁定参数修改 | `[Addr] D6 4B [Store] [Level] [Chk]` | `00` 解锁；`01/02/03` 锁定等级 |
| 上电自动运行 | `[Addr] F7 1C [ClearOrStore] [Dir] [Speed:u16] [Acc] [EnControl] [Chk]` | `ClearOrStore=00/01`；`EnControl=00/01` |

| `OptionStatus` 位 | 定义 |
|---:|---|
| 0 | `MotType`：`0/1=1.8°/0.9°` |
| 1 | `FwType`：`0/1=X/Emm` |
| 2 | `CtrMode`：`0/1=开环/闭环` |
| 4 | `MotDir`：`0/1=CW/CCW` |
| 5 | `BtLock`：`0/1=解锁/锁定` |
| 7 | `Scale`：`0/1=关闭/开启` |

```text
DMX 读取返回：[Addr] 49 [TotalChannels:u16] [ChannelsPerMotor] [MoveMode] [SingleSpeed:u16] [Acc:u16] [DualSpeedStep:u16] [DualMoveStep:u32] [Chk]
DMX 修改发送：[Addr] D9 90 [Store] [TotalChannels:u16] [ChannelsPerMotor] [MoveMode] [SingleSpeed:u16] [Acc:u16] [DualSpeedStep:u16] [DualMoveStep:u32] [Chk]
```

`ChannelsPerMotor=01/02`；`MoveMode=00/01` 相对/绝对；`DualMoveStep` 单位 `0.1°`。DMX512 固定 `250000` 波特率、2 个停止位。

## 📦 批量参数帧

```text
读取系统状态发送：[Addr] 43 7A [Chk]
返回：[Addr] 43 1F 09 [VBus:u16] [PhaseCurrent:u16] [Encoder:u16] [TargetSign] [TargetPosition:u32] [SpeedSign] [RealSpeed:u16] [RealSign] [RealPosition:u32] [ErrorSign] [PositionError:u32] [HomeStatus] [MotorStatus] [Chk]

读取驱动配置发送：[Addr] 42 6C [Chk]
返回：[Addr] 42 21 15 [MotorType] [PulseMux] [CommMux] [EnLevel] [DirLevel] [Microstep] [Interpolation] 00 [OpenCurrent:u16] [ClosedMaxCurrent:u16] [MaxOutput:u16] [UartBaud] [CanBaud] [ID] [ChecksumMode] [ResponseMode] [StallProtect] [StallSpeed:u16] [StallCurrent:u16] [StallTime:u16] [PositionWindow:u16] [Chk]

修改驱动配置发送：[Addr] 48 D1 [Store] [MotorType] [PulseMux] [CommMux] [EnLevel] [DirLevel] [Microstep] [Interpolation] 00 [OpenCurrent:u16] [ClosedMaxCurrent:u16] [MaxOutput:u16] [UartBaud] [CanBaud] 00 [ChecksumMode] [ResponseMode] [StallProtect] [StallSpeed:u16] [StallCurrent:u16] [StallTime:u16] [PositionWindow:u16] [Chk]
返回：[Addr] 48 02/E2/EE [Chk]
```

| 字段 | 值 |
|---|---|
| `PulseMux` | `00/01/02/03/04=OFF/OPEN/FOC/ESI_RCO/pLR_ESI` |
| `CommMux` | `00/01/02/03/04=OFF/ESI_ALO/UART/CAN/uLR_ESI` |
| `EnLevel` | `00/01/02=L/H/Hold` |
| `DirLevel` | `00/01=CW/CCW` |
| `Interpolation` | `00/01=关闭/开启` |
| `UartBaud` | `00-08=9600/19200/25000/38400/57600/115200/256000/512000/921600` |
| `ChecksumMode` | `00/01/02/03/04=6B/XOR/CRC-8/Modbus/6B+DMX512` |
| `StallProtect` | `00/01/02=关闭/保护后松轴/堵转后清零且不松轴` |

## 🧮 校验

```text
固定校验：Chk=6B
XOR：Chk=校验字节前所有字节逐字节异或
CRC-8：校验字节前所有字节，CRC-8/MAXIM，反射多项式 0x8C，初值 0x00
Modbus-RTU：CRC-16
```

```c
uint8_t zdt_crc8(const uint8_t*p,size_t n){uint8_t c=0;while(n--){c^=*p++;for(int i=0;i<8;i++)c=c&1?(c>>1)^0x8C:c>>1;}return c;}
```
