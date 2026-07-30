# UART2 串口通讯协议说明

## 1. 硬件接口

使用开发板的 **串口2 (UART2)**，通过 **GH1.25-4P 带锁座** 输出数据。

| 座子引脚 | 丝印  | 功能      | GPIO   | 说明                           |
| :------: | :---: | --------- | ------ | ------------------------------ |
|    1     |   V   | 5V 输出   | —      | 不接外部设备                   |
|    2     |   R   | UART2_RXD | GPIO12 | 接收端（本程序仅发送，可不接） |
|    3     |   T   | UART2_TXD | GPIO11 | 发送端 → 接外部设备的 RX       |
|    4     |   G   | GND       | —      | 必须与外部设备共地             |

> **注意**：
> - TX 接对方 RX，RX 接对方 TX，交叉连接。
> - 两设备必须共地（GND 互连），否则无法正常通讯。
> - 两板（K230 / Lite K230D）的串口2引脚完全一致，代码通用。

---

## 2. 串口参数

| 参数               | 值              | 说明                    |
| ------------------ | --------------- | ----------------------- |
| 波特率 (Baudrate)  | **115200**      | 每秒传输 115200 个 bit  |
| 数据位 (Data Bits) | **8**           | 每个数据帧包含 8 位数据 |
| 校验位 (Parity)    | **无 (None)**   | 不启用奇偶校验          |
| 停止位 (Stop Bits) | **1**           | 1 位停止位              |
| 简称               | **115200, 8N1** | 最通用的串口配置        |

> 电脑端串口助手（MobaXterm / SSCOM / PuTTY）请按上表配置。

---

## 3. 数据帧格式（二进制 HEX 协议）

每帧 **6 字节**，纯二进制格式（非 ASCII 文本），小端字节序。

### 3.1 帧结构

| 字节偏移 | 长度 | 字段 | 说明 |
| :------: | :--: | :--- | :--- |
| 0 | 1 | `0xAA` | 帧头同步字 1 |
| 1 | 1 | `0x55` | 帧头同步字 2 |
| 2 | 2 | `X_Pos` | 当前钢球 X 坐标，`uint16`，小端序 (LSB 在前) |
| 4 | 2 | `TGT_X` | 目标 X 坐标，`uint16`，小端序，预设 `400` (即 `0x90 0x01`) |

> **总帧长 = 6 字节**

### 3.2 示例

检测到钢球 X=120，目标 X=400 时发送：

```
AA 55  78 00  90 01
│  │   └─X=120 └─TGT=400(小端)
│  └─帧头2
└─帧头1
```

### 3.3 无检测结果时

不发送任何数据（静默）。

### 3.4 帧时序示意

```
时间 →

[AA 55 78 00 90 01]   检测到钢球 X=120
[AA 55 36 01 90 01]   检测到钢球 X=310

--- 下一帧图像 ---

[AA 55 76 00 90 01]   检测到钢球 X=118
```

> 每帧图像仅发送第一个检测到的钢球坐标。
> 发送频率 ≈ 检测帧率（约 60 FPS）。

---

## 4. 接收端解析示例

### 4.1 Python 解析

```python
import serial
import struct

ser = serial.Serial('COM3', 115200, timeout=1)
buf = bytearray()

while True:
    buf.extend(ser.read(ser.in_waiting or 1))
    while len(buf) >= 6:
        if buf[0] == 0xAA and buf[1] == 0x55:
            x_pos, tgt_x = struct.unpack('<HH', buf[2:6])
            print(f"X={x_pos}, Target={tgt_x}")
            buf = buf[6:]
        else:
            buf.pop(0)  # 丢字节同步
```

### 4.2 STM32 (C) 解析

```c
#define FRAME_LEN 6
uint8_t rx_buf[FRAME_LEN];
uint8_t rx_idx = 0;

void USARTx_IRQHandler(void) {
    if (USART_GetITStatus(USARTx, USART_IT_RXNE)) {
        uint8_t byte = USART_ReceiveData(USARTx);
        if (rx_idx == 0 && byte != 0xAA) return;  // 等待帧头1
        if (rx_idx == 1 && byte != 0x55) { rx_idx = 0; return; }  // 帧头2不对则重置
        rx_buf[rx_idx++] = byte;
        if (rx_idx >= FRAME_LEN) {
            rx_idx = 0;
            uint16_t x_pos = rx_buf[2] | (rx_buf[3] << 8);
            uint16_t tgt_x  = rx_buf[4] | (rx_buf[5] << 8);
            // 使用 x_pos, tgt_x ...
        }
    }
}
```

---

## 5. 错误处理

| 情况 | 串口输出 |
| :--- | :--- |
| 检测到钢球 | `AA 55 [X_L] [X_H] [TGT_L] [TGT_H]` |
| 未检测到钢球 | 不输出 |
| 坐标异常 (负值/超大) | 限幅至 0~65535 后正常发送 |
| 程序异常 | 停止输出，`finally` 中释放 UART 资源 |

---

## 6. 参考

- [串口通讯【UART】| 立创开发板技术文档中心](https://wiki.lckfb.com/zh-hans/lushan-pi-k230/basic/uart.html)
- UART2 引脚：GPIO11 (TX) / GPIO12 (RX)
- MicroPython API：`machine.UART` + `machine.FPIOA`
