# k230-steel-ball-detection
YOLOv8 steel-ball detection demo for CanMV K230

基于 YOLOv8 和 CanMV K230 的钢球目标检测示例。

仅初步测试，模型训练还不完善。后续会优化。

## 文件说明

- `main.py`：K230 端推理和显示程序
- `best.pt`：原始pt模型，可用于maixcam
- `best.kmodel`：转换后的 K230 模型

## 当前配置

- LCD 显示：800 × 480
- AI 摄像头通道：320 × 180
- 模型输入：320 × 320
- 摄像头目标帧率：60 FPS
- 类别：`ball`
- 置信度阈值：0.3
- 显示方式：LCD
- 摄像头编号：sensor_id=2

## 帧率优化

`main.py` 保持 800 × 480 LCD、模型输入和逐帧检测，通过减少采集、预处理及主循环开销提高帧率：

- 缓存高频调用的方法和 FPS 文本；
- AI 通道由 640 × 360 调整为 320 × 180，保持 16:9 视野并直接匹配模型有效图像区域；
- 垃圾回收调整为每 60 帧执行一次；
- 绕过 `PipeLine.py` 对 `k230_canmv_lckfb` 强制设置的 30 FPS 上限；
- 关闭 CanMV IDE 画面镜像，只保留板载 LCD 输出。

当前优化面向庐山派 K230 和 CanMV v1.6-56，不改变检测阈值、绘制内容或板载 LCD 操作方式。

> 运行后请以板载 LCD 为准；`to_ide=False` 后 CanMV IDE 不再同步预览画面。

## 使用方法

1. 将 `main.py` 复制到 K230 存储卡根目录。
2. 将 `best.kmodel` 复制为：

   `/sdcard/best.kmodel`

3. 确保运行环境能够导入：

   - `libs.PipeLine`
   - `libs.YOLO`

4. 在 CanMV IDE 中运行 `main.py`。

## 注意

当前仓库没有包含 `libs` 模块。如果所使用的 CanMV 固件或示例环境没有提供这些模块，
需要另外复制对应的 `PipeLine.py`、`YOLO.py` 及其依赖文件。

## License

MIT License
