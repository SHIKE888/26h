# ============================================================
# 立创·庐山派 Lite-K230D
# GC2093 CSI2 摄像头 -> YOLOv8钢球检测 -> LCD显示 + RTSP推流
#
# 合并 main.py(检测+LCD) + p.py(RTSP推流) 功能
#
# 架构 (单摄像头, 不冲突):
#   GC2093(Sensor id=2)
#     ├── CHN_0 (RGB888) -> YOLO检测 -> LCD OSD叠加显示
#     └── CHN_1 (YUV420SP) -> H.264编码 -> RTSP推流
#
# 播放地址:
#   rtsp://开发板IP:8554/live
#
# 使用前请修改 WIFI_SSID / WIFI_PASSWORD
# ============================================================

import os
import gc
import time
import uctypes
import network
import multimedia as mm

from media.sensor import *
from media.vencoder import *
from media.media import *
from media.display import *

from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8


# ============================================================
# 用户配置
# ============================================================

# --------- Wi-Fi ----------
WIFI_SSID = "J5-306B"
WIFI_PASSWORD = "306306306"

# --------- 摄像头 ----------
SENSOR_ID = 2
SENSOR_WIDTH = 1920
SENSOR_HEIGHT = 1080
SENSOR_FPS = 60

# --------- YOLO 模型 ----------
MODEL_PATH = "/sdcard/best.kmodel"
LABELS = ["ball"]
CONF_THRESH = 0.3
NMS_THRESH = 0.45
MAX_BOXES = 20

# --------- 分辨率 ----------
RGB888P_SIZE = [640, 360]
MODEL_INPUT_SIZE = [320, 320]
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360

# --------- RTSP ----------
RTSP_PORT = 8554
RTSP_SESSION = "live"
RTSP_TIMESTAMP = 1000

# --------- H.264 编码 ----------
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
VIDEO_BITRATE_KBPS = 4000
VIDEO_GOP = 30
VIDEO_OUTPUT_FPS = 30
VENC_CHANNEL = VENC_CHN_ID_0
VENC_BUFFER_NUM = 8

# --------- 统计间隔 ----------
STATS_PERIOD_MS = 2000


# ============================================================
# Wi-Fi 连接
# ============================================================

def connect_wifi(ssid, password, timeout_s=30):
    wlan = network.WLAN(network.STA_IF)
    if not wlan.active():
        wlan.active(True)

    if wlan.isconnected():
        ip_info = wlan.ifconfig()
        print("[WiFi] 已连接, IP:", ip_info[0])
        return wlan

    print("[WiFi] 正在连接:", ssid)
    wlan.connect(ssid, password)

    start_ms = time.ticks_ms()
    last_print_ms = start_ms

    while not wlan.isconnected():
        os.exitpoint()
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_print_ms) >= 1000:
            waited_s = time.ticks_diff(now_ms, start_ms) // 1000
            print("[WiFi] 等待连接, 已等待", waited_s, "秒")
            last_print_ms = now_ms
        if time.ticks_diff(now_ms, start_ms) >= timeout_s * 1000:
            try:
                wlan.disconnect()
            except BaseException:
                pass
            raise OSError("Wi-Fi连接超时")
        time.sleep_ms(100)

    # 等待 DHCP
    dhcp_start = time.ticks_ms()
    while wlan.ifconfig()[0] == "0.0.0.0":
        os.exitpoint()
        if time.ticks_diff(time.ticks_ms(), dhcp_start) >= 10000:
            raise OSError("DHCP获取IP超时")
        time.sleep_ms(100)

    ip_info = wlan.ifconfig()
    print("[WiFi] 连接成功, IP:", ip_info[0])
    return wlan


# ============================================================
# 主程序
# ============================================================

def main():
    sensor = None
    yolo = None
    encoder = None
    rtsp_server = None
    wifi = None
    pl = None

    media_initialized = False
    encoder_created = False
    encoder_started = False
    rtsp_initialized = False
    rtsp_started = False
    rtsp_session_created = False
    sensor_started = False
    display_initialized = False

    os.exitpoint(os.EXITPOINT_ENABLE)

    try:
        # ---- 1. 连接 Wi-Fi ----
        wifi = connect_wifi(WIFI_SSID, WIFI_PASSWORD, timeout_s=30)

        # ---- 2. 初始化摄像头 ----
        print("\n[Camera] 初始化 GC2093 摄像头 ...")
        sensor = Sensor(
            id=SENSOR_ID,
            width=SENSOR_WIDTH,
            height=SENSOR_HEIGHT,
            fps=SENSOR_FPS
        )
        sensor.reset()

        # CHN_0: RGB888 用于 YOLO 检测 + LCD 显示
        sensor.set_framesize(
            width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT,
            chn=CAM_CHN_ID_0, alignment=12
        )
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)

        # CHN_1: YUV420SP 用于 H.264 编码 -> RTSP
        enc_width = ALIGN_UP(VIDEO_WIDTH, 16)
        sensor.set_framesize(
            width=enc_width, height=VIDEO_HEIGHT,
            chn=CAM_CHN_ID_1, alignment=12
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_1)

        print("[Camera] CHN_0: {}x{} RGB888  (检测+LCD)".format(
            DISPLAY_WIDTH, DISPLAY_HEIGHT))
        print("[Camera] CHN_1: {}x{} YUV420SP (编码+RTSP)".format(
            enc_width, VIDEO_HEIGHT))

        # ---- 3. 加载 YOLO 模型 ----
        print("\n[YOLO] 加载模型:", MODEL_PATH)
        yolo = YOLOv8(
            task_type="detect", mode="video",
            kmodel_path=MODEL_PATH, labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=[DISPLAY_WIDTH, DISPLAY_HEIGHT],
            conf_thresh=CONF_THRESH,
            nms_thresh=NMS_THRESH,
            max_boxes_num=MAX_BOXES,
            debug_mode=0
        )
        yolo.config_preprocess()
        print("[YOLO] 模型加载完成")

        # ---- 4. 初始化 LCD 显示 ----
        print("\n[Display] 初始化 LCD ...")
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
        display_initialized = True
        print("[Display] LCD 初始化完成")

        # ---- 5. 初始化 H.264 编码器 (CHN_1) ----
        print("\n[Encoder] 初始化 H.264 编码器 ...")
        encoder = Encoder()
        encoder.SetOutBufs(VENC_CHANNEL, VENC_BUFFER_NUM, enc_width, VIDEO_HEIGHT)

        try:
            channel_attr = ChnAttrStr(
                encoder.PAYLOAD_TYPE_H264,
                encoder.H264_PROFILE_MAIN,
                enc_width, VIDEO_HEIGHT,
                VIDEO_BITRATE_KBPS, VIDEO_GOP,
                VIDEO_OUTPUT_FPS, VIDEO_OUTPUT_FPS
            )
        except TypeError:
            channel_attr = ChnAttrStr(
                encoder.PAYLOAD_TYPE_H264,
                encoder.H264_PROFILE_MAIN,
                enc_width, VIDEO_HEIGHT
            )

        encoder.Create(VENC_CHANNEL, channel_attr)
        encoder_created = True
        encoder.Start(VENC_CHANNEL)
        encoder_started = True
        print("[Encoder] {}kbps GOP={} 启动完成".format(VIDEO_BITRATE_KBPS, VIDEO_GOP))

        # ---- 6. 绑定 CHN_1 -> Encoder ----
        sensor_src = sensor.bind_info(chn=CAM_CHN_ID_1)["src"]
        enc_dst = (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, VENC_CHANNEL)
        media_link = MediaManager.link(sensor_src, enc_dst)
        print("[Encoder] CHN_1 -> Encoder 绑定完成")

        # ---- 7. 初始化 RTSP 服务器 ----
        print("\n[RTSP] 初始化 RTSP 服务器 ...")
        MediaManager.init()
        media_initialized = True

        rtsp_server = mm.rtsp_server()
        rtsp_server.rtspserver_init(RTSP_PORT)
        rtsp_initialized = True
        rtsp_server.rtspserver_createsession(
            RTSP_SESSION, mm.multi_media_type.media_h264, False
        )
        rtsp_session_created = True
        rtsp_server.rtspserver_start()
        rtsp_started = True

        rtsp_url = rtsp_server.rtspserver_getrtspurl(RTSP_SESSION)
        print("[RTSP] 推流地址:", rtsp_url)

        # ---- 8. 启动传感器 ----
        sensor.run()
        sensor_started = True
        print("[Camera] 传感器已启动")

        # ---- 9. 主循环 ----
        print("\n" + "=" * 50)
        print("  钢球检测 + RTSP推流 启动成功!")
        print("  LCD  : YOLO检测叠加 (CHN_0)")
        print("  RTSP : 原始画面 (CHN_1)")
        print("  播放 :", rtsp_url)
        print("=" * 50 + "\n")

        stream_data = StreamData()

        # 检测 FPS
        fps_val = 0
        fps_cnt = 0
        fps_tm = time.ticks_ms()

        # 推流统计
        total_frames = 0
        period_frames = 0
        period_bytes = 0
        stats_start_ms = time.ticks_ms()

        while True:
            os.exitpoint()

            # ---- 9a. 获取 CHN_0 RGB 图像用于检测 ----
            rgb_img = sensor.snapshot(chn=CAM_CHN_ID_0)

            # ---- 9b. YOLO 检测 ----
            result = yolo.run(rgb_img)

            # ---- 9c. 提取坐标 ----
            coord_str = "ND"
            try:
                d0 = result[0]
                s = str(d0)
                coord_str = s[:16]
            except Exception:
                coord_str = "E"

            # ---- 9d. 绘制检测结果 ----
            yolo.draw_result(result, rgb_img)

            # ---- 9e. FPS 计算 ----
            fps_cnt += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, fps_tm) >= 1000:
                fps_val = fps_cnt
                fps_cnt = 0
                fps_tm = now

            # ---- 9f. 叠加 OSD 文字 ----
            rgb_img.draw_string(0, 0, "FPS:{}".format(fps_val),
                                color=(0, 255, 0), scale=2)
            rgb_img.draw_string(0, 24, "XY:" + coord_str,
                                color=(255, 255, 0), scale=2)

            # ---- 9g. LCD 显示 ----
            Display.show_image(rgb_img)

            # ---- 9h. RTSP 推流 (从 CHN_1 编码器取数据) ----
            enc_result = encoder.GetStream(VENC_CHANNEL, stream_data)
            if enc_result in (0, None):
                try:
                    for pack_index in range(stream_data.pack_cnt):
                        packet_size = stream_data.data_size[pack_index]
                        if packet_size <= 0:
                            continue
                        packet_data = bytes(
                            uctypes.bytearray_at(
                                stream_data.data[pack_index],
                                packet_size
                            )
                        )
                        rtsp_server.rtspserver_sendvideodata(
                            RTSP_SESSION, packet_data,
                            packet_size, RTSP_TIMESTAMP
                        )
                        period_bytes += packet_size

                    total_frames += 1
                    period_frames += 1

                finally:
                    encoder.ReleaseStream(VENC_CHANNEL, stream_data)

            # ---- 9i. 定期统计 ----
            now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, stats_start_ms)
            if elapsed_ms >= STATS_PERIOD_MS:
                elapsed_s = elapsed_ms / 1000.0
                actual_fps = period_frames / elapsed_s if elapsed_s > 0 else 0
                bitrate = (period_bytes * 8.0 / elapsed_s / 1000000.0) \
                    if elapsed_s > 0 else 0
                print("[Stats] 检测FPS:{} 推流FPS:{:.1f} "
                      "码率:{:.2f}Mbps 总帧:{} 检测:{}".format(
                          fps_val, actual_fps, bitrate,
                          total_frames, coord_str))
                period_frames = 0
                period_bytes = 0
                stats_start_ms = now_ms

            gc.collect()

    except KeyboardInterrupt as e:
        print("\n[Main] 用户停止:", e)

    except BaseException as e:
        error_text = repr(e)
        if "IDE interrupt" in error_text:
            print("\n[Main] IDE 停止程序")
        else:
            print("\n[Main] 异常:", type(e).__name__, error_text)

    finally:
        print("\n" + "=" * 50)
        print("[Cleanup] 释放资源...")
        print("=" * 50)

        # 1. 停止传感器
        if sensor_started and sensor is not None:
            try:
                sensor.stop()
                print("[Cleanup] 传感器已停止")
            except BaseException as e:
                print("[Cleanup] sensor.stop:", e)

        # 2. 解除媒体绑定
        try:
            if media_link is not None:
                media_link.destroy()
                print("[Cleanup] 媒体绑定已解除")
        except BaseException as e:
            print("[Cleanup] media_link:", e)

        # 3. 停止 & 销毁编码器
        if encoder_started and encoder is not None:
            try:
                encoder.Stop(VENC_CHANNEL)
                print("[Cleanup] 编码器已停止")
            except BaseException as e:
                print("[Cleanup] encoder.Stop:", e)
        if encoder_created and encoder is not None:
            try:
                encoder.Destroy(VENC_CHANNEL)
                print("[Cleanup] 编码通道已销毁")
            except BaseException as e:
                print("[Cleanup] encoder.Destroy:", e)

        # 4. 停止 RTSP
        if rtsp_started and rtsp_server is not None:
            try:
                rtsp_server.rtspserver_stop()
                print("[Cleanup] RTSP服务器已停止")
            except BaseException as e:
                print("[Cleanup] rtspserver_stop:", e)
        if rtsp_session_created and rtsp_server is not None:
            try:
                rtsp_server.rtspserver_destroysession(RTSP_SESSION)
            except BaseException:
                pass
        if rtsp_initialized and rtsp_server is not None:
            try:
                rtsp_server.rtspserver_deinit()
                print("[Cleanup] RTSP已反初始化")
            except BaseException as e:
                print("[Cleanup] rtspserver_deinit:", e)

        # 5. 释放 MediaManager
        if media_initialized:
            try:
                time.sleep_ms(100)
                MediaManager.deinit()
                print("[Cleanup] MediaManager已释放")
            except BaseException as e:
                print("[Cleanup] MediaManager:", e)

        # 6. 释放 LCD
        if display_initialized:
            try:
                Display.deinit()
            except BaseException:
                pass

        # 7. 释放 YOLO
        if yolo is not None:
            try:
                yolo.deinit()
                print("[Cleanup] YOLO已释放")
            except BaseException as e:
                print("[Cleanup] yolo.deinit:", e)

        # 8. 断开 Wi-Fi
        if wifi is not None:
            try:
                if wifi.isconnected():
                    wifi.disconnect()
                    print("[Cleanup] Wi-Fi已断开")
            except BaseException:
                pass

        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        gc.collect()
        print("[Main] 程序结束")


if __name__ == "__main__":
    main()

