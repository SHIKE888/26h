from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from media.sensor import Sensor
from machine import UART, FPIOA
import os
import gc
import time

GC_INTERVAL = 60
SENSOR_ID = 2
SENSOR_FPS = 60
DISPLAY_SIZE = [800, 480]

# ---- 目标坐标预设 (后续可改为动态变量) ----
TARGET_X = 400

pl = None
yolo = None
uart = None

try:
    model_path = "/sdcard/best.kmodel"

    rgb888p_size = [320, 180]
    model_input_size = [320, 320]
    labels = ["ball"]

    # ---- 初始化 UART2 (GPIO11 TX, GPIO12 RX) ----
    # 配置 FPIOA 将 GPIO 复用为 UART2 功能
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)

    # 初始化 UART2: 115200波特率, 8位数据, 无校验, 1停止位 (8N1)
    uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    print("[UART2] 初始化完成, 115200@8N1, GPIO11(TX) GPIO12(RX)")

    sensor = Sensor(id=SENSOR_ID, fps=SENSOR_FPS)
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode="lcd", display_size=DISPLAY_SIZE)
    pl.create(sensor=sensor, to_ide=False)

    display_size = pl.get_display_size()

    yolo = YOLOv8(
        task_type="detect", mode="video",
        kmodel_path=model_path, labels=labels,
        rgb888p_size=rgb888p_size, model_input_size=model_input_size,
        display_size=display_size,
        conf_thresh=0.3, nms_thresh=0.45,
        max_boxes_num=20, debug_mode=0
    )
    yolo.config_preprocess()

    # FPS
    fps_cnt = 0
    fps_tm = time.ticks_ms()
    fps_text = "FPS:0"
    gc_cnt = 0

    exitpoint = os.exitpoint
    ticks_ms = time.ticks_ms
    ticks_diff = time.ticks_diff
    get_frame = pl.get_frame
    show_image = pl.show_image
    run_yolo = yolo.run
    draw_result = yolo.draw_result
    osd_img = pl.osd_img
    draw_string = osd_img.draw_string

    while True:
        exitpoint()

        img = get_frame()
        result = run_yolo(img)

        # ---- 坐标提取 & UART 发送 ----
        # 屏幕显示和串口输出使用同一份 coord_str
        coord_str = "ND"
        try:
            d0 = result[0]
            coord_str = str(d0)[:16]
        except Exception:
            coord_str = "E"

        draw_result(result, osd_img)

        # ---- 串口输出: 直接发送屏幕坐标字符串 + 目标坐标 ----
        try:
            uart.write(coord_str + "," + str(TARGET_X) + "\n")
        except Exception as e:
            print("[UART] 发送异常:", e)

        # FPS
        fps_cnt += 1
        now = ticks_ms()
        if ticks_diff(now, fps_tm) >= 1000:
            fps_text = "FPS:{}".format(fps_cnt)
            fps_cnt = 0
            fps_tm = now

        draw_string(0, 0, fps_text, (0, 255, 0), 2)
        draw_string(0, 24, "XY:" + coord_str, (255, 255, 0), 2)

        show_image()

        gc_cnt += 1
        if gc_cnt >= GC_INTERVAL:
            gc.collect()
            gc_cnt = 0

except KeyboardInterrupt as e:
    print("user stop:", e)

except BaseException as e:
    print("error:", e)

finally:
    if yolo is not None:
        yolo.deinit()

    if uart is not None:
        uart.deinit()
        print("[UART2] 资源已释放")

    if pl is not None:
        pl.destroy()

    gc.collect()
    time.sleep_ms(100)
