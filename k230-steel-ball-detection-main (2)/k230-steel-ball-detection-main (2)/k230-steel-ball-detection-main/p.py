# ============================================================
# 立创·庐山派 Lite-K230D
# GC2093 CSI2 摄像头 -> H.264 -> Wi-Fi RTSP
#
# 目标：
#   1280×720、约30 FPS、高画质、流畅优先
#   可以接受2～3秒播放延迟
#
# 适用固件：
#   CanMV v1.8-0-gc2d1f5c
#   k230d_canmv_lushanpi_lite with K230D
#
# 本版本修复：
#   1. 明确设置H.264码率为4000 kbps；
#   2. 明确设置GOP为30；
#   3. 输入60 FPS、编码输出30 FPS；
#   4. RTSP时间戳恢复为官方示例使用的1000；
#   5. 使用H.264 Main Profile提高兼容性；
#   6. IDE interrupt按正常停止处理。
#
# 使用前：
#   1. 修改 WIFI_PASSWORD；
#   2. 在CanMV IDE中按Ctrl+A删除旧代码；
#   3. 只粘贴和运行本程序；
#   4. VLC网络缓存建议设置为2000～3000 ms。
#
# 播放地址：
#   rtsp://开发板IP:8554/live
# ============================================================

import os
import time
import network
import uctypes
import multimedia as mm

from media.sensor import *
from media.vencoder import *
from media.media import *


# ============================================================
# 一、用户配置
# ============================================================

# 手机热点
WIFI_SSID = "J5-306B"
WIFI_PASSWORD = "306306306"

# 默认CSI2摄像头
SENSOR_ID = 2

# RTSP参数
RTSP_PORT = 8554
RTSP_SESSION = "live"

# 推流输出
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720

# GC2093原生采集模式
SENSOR_INPUT_WIDTH = 1920
SENSOR_INPUT_HEIGHT = 1080
SENSOR_INPUT_FPS = 60

# H.264编码参数
VIDEO_BITRATE_KBPS = 4000
VIDEO_GOP = 30
VIDEO_OUTPUT_FPS = 30

# 编码通道与缓冲区
VENC_CHANNEL = VENC_CHN_ID_0
VENC_BUFFER_NUM = 8

# RTSP官方示例使用固定1000
RTSP_TIMESTAMP = 1000

# 每2秒打印一次统计信息
STATS_PERIOD_MS = 2000


# ============================================================
# 二、Wi-Fi连接
# ============================================================

def connect_wifi(ssid, password, timeout_s=30):
    """
    以STA模式连接2.4 GHz Wi-Fi。
    """

    wlan = network.WLAN(network.STA_IF)

    if not wlan.active():
        wlan.active(True)

    print("[WiFi] 模块激活状态:", wlan.active())

    if wlan.isconnected():
        ip_info = wlan.ifconfig()

        print("[WiFi] 已经连接")
        print("[WiFi] IP地址:", ip_info[0])
        print("[WiFi] 子网掩码:", ip_info[1])
        print("[WiFi] 网关:", ip_info[2])
        print("[WiFi] DNS:", ip_info[3])

        return wlan

    print("[WiFi] 正在连接:", ssid)
    wlan.connect(ssid, password)

    start_ms = time.ticks_ms()
    last_print_ms = start_ms

    while not wlan.isconnected():
        os.exitpoint()

        now_ms = time.ticks_ms()

        if time.ticks_diff(now_ms, last_print_ms) >= 1000:
            waited_s = (
                time.ticks_diff(now_ms, start_ms) // 1000
            )

            print(
                "[WiFi] 等待连接，已等待",
                waited_s,
                "秒"
            )

            last_print_ms = now_ms

        if time.ticks_diff(
            now_ms,
            start_ms
        ) >= timeout_s * 1000:

            try:
                wlan.disconnect()
            except BaseException:
                pass

            raise OSError(
                "Wi-Fi连接超时，请检查热点名称、密码，"
                "并确认热点使用2.4 GHz"
            )

        time.sleep_ms(100)

    # 等待DHCP获取IP
    dhcp_start_ms = time.ticks_ms()

    while wlan.ifconfig()[0] == "0.0.0.0":
        os.exitpoint()

        if time.ticks_diff(
            time.ticks_ms(),
            dhcp_start_ms
        ) >= 10000:

            raise OSError(
                "Wi-Fi已经连接，但DHCP获取IP地址超时"
            )

        time.sleep_ms(100)

    ip_info = wlan.ifconfig()

    print("[WiFi] 连接成功")
    print("[WiFi] IP地址:", ip_info[0])
    print("[WiFi] 子网掩码:", ip_info[1])
    print("[WiFi] 网关:", ip_info[2])
    print("[WiFi] DNS:", ip_info[3])

    return wlan


# ============================================================
# 三、打印摄像头支持模式
# ============================================================

def print_sensor_modes():
    """
    打印GC2093支持的模式。
    某些固件不支持list_mode时不影响主程序。
    """

    try:
        sensor_name, modes = Sensor.list_mode(
            id=SENSOR_ID
        )

        print("")
        print("========================================")
        print("[Sensor] 摄像头模式列表")
        print("========================================")
        print("[Sensor] 名称:", sensor_name)

        if modes:
            for index in range(len(modes)):
                mode = modes[index]

                print(
                    "[Sensor] 模式",
                    index,
                    ":",
                    mode["width"],
                    "x",
                    mode["height"],
                    "@",
                    mode["fps"],
                    "fps"
                )

        print("========================================")

    except BaseException as exc:
        print(
            "[Sensor] 无法读取摄像头模式列表:",
            repr(exc)
        )


# ============================================================
# 四、创建编码属性
# ============================================================

def create_h264_channel_attr(encoder, width, height):
    """
    创建适合当前CanMV v1.8固件的H.264编码属性。

    当前固件的ChnAttrStr很可能采用以下参数顺序：
        payloadType
        profile
        picWidth
        picHeight
        bit_rate
        gopLen
        src_frame_rate
        dst_frame_rate
        mjpeg_quality_factor

    因此必须明确把4000放在第5个位置，把GOP放在第6个位置。
    上一版本把60放在第5个位置，可能被解释成60 kbps码率。
    """

    try:
        attr = ChnAttrStr(
            encoder.PAYLOAD_TYPE_H264,
            encoder.H264_PROFILE_MAIN,
            width,
            height,
            VIDEO_BITRATE_KBPS,
            VIDEO_GOP,
            SENSOR_INPUT_FPS,
            VIDEO_OUTPUT_FPS
        )

        print("[Video] 使用扩展ChnAttrStr参数")
        print(
            "[Video] 目标码率:",
            VIDEO_BITRATE_KBPS,
            "kbps"
        )
        print("[Video] GOP:", VIDEO_GOP)
        print(
            "[Video] 编码帧率:",
            SENSOR_INPUT_FPS,
            "->",
            VIDEO_OUTPUT_FPS
        )

        return attr

    except TypeError as exc:
        # 仅用于兼容只支持5参数的旧构造函数。
        # 如果进入这里，该固件不能直接通过ChnAttrStr设置码率。
        print(
            "[Video] 扩展ChnAttrStr不受支持:",
            repr(exc)
        )

        print(
            "[Video] 回退到基础参数，使用固件默认码率"
        )

        return ChnAttrStr(
            encoder.PAYLOAD_TYPE_H264,
            encoder.H264_PROFILE_MAIN,
            width,
            height
        )


# ============================================================
# 五、主程序
# ============================================================

def main():
    wifi = None
    sensor = None
    encoder = None
    media_link = None
    rtsp_server = None

    media_initialized = False
    encoder_created = False
    encoder_started = False
    sensor_started = False
    rtsp_initialized = False
    rtsp_started = False
    rtsp_session_created = False

    width = ALIGN_UP(VIDEO_WIDTH, 16)
    height = VIDEO_HEIGHT

    os.exitpoint(os.EXITPOINT_ENABLE)

    try:
        # ----------------------------------------------------
        # 1. 摄像头模式
        # ----------------------------------------------------

        print_sensor_modes()

        # ----------------------------------------------------
        # 2. 连接Wi-Fi
        # ----------------------------------------------------

        wifi = connect_wifi(
            WIFI_SSID,
            WIFI_PASSWORD,
            timeout_s=30
        )

        # ----------------------------------------------------
        # 3. 初始化摄像头
        # ----------------------------------------------------

        print("")
        print("========================================")
        print("[Video] 初始化摄像头")
        print("========================================")

        sensor = Sensor(
            id=SENSOR_ID,
            width=SENSOR_INPUT_WIDTH,
            height=SENSOR_INPUT_HEIGHT,
            fps=SENSOR_INPUT_FPS
        )

        sensor.reset()

        sensor.set_framesize(
            width=width,
            height=height,
            chn=CAM_CHN_ID_0,
            alignment=12
        )

        sensor.set_pixformat(
            Sensor.YUV420SP,
            chn=CAM_CHN_ID_0
        )

        print("[Video] 摄像头编号:", SENSOR_ID)

        print(
            "[Video] 原生输入请求:",
            SENSOR_INPUT_WIDTH,
            "x",
            SENSOR_INPUT_HEIGHT,
            "@",
            SENSOR_INPUT_FPS,
            "fps"
        )

        print(
            "[Video] RTSP输出:",
            width,
            "x",
            height
        )

        print("[Video] 像素格式: YUV420SP")

        # ----------------------------------------------------
        # 4. 初始化H.264编码器
        # ----------------------------------------------------

        print("")
        print("========================================")
        print("[Video] 初始化H.264编码器")
        print("========================================")

        encoder = Encoder()

        print("[Video] Encoder对象创建成功")

        encoder.SetOutBufs(
            VENC_CHANNEL,
            VENC_BUFFER_NUM,
            width,
            height
        )

        print(
            "[Video] VENC缓冲区数量:",
            VENC_BUFFER_NUM
        )

        # ----------------------------------------------------
        # 5. 绑定摄像头与VENC
        # ----------------------------------------------------

        sensor_source = sensor.bind_info(
            chn=CAM_CHN_ID_0
        )["src"]

        encoder_destination = (
            VIDEO_ENCODE_MOD_ID,
            VENC_DEV_ID,
            VENC_CHANNEL
        )

        media_link = MediaManager.link(
            sensor_source,
            encoder_destination
        )

        print("[Video] 摄像头与VENC绑定成功")

        MediaManager.init()
        media_initialized = True

        print("[Media] MediaManager初始化成功")

        # ----------------------------------------------------
        # 6. 创建H.264编码通道
        # ----------------------------------------------------

        channel_attr = create_h264_channel_attr(
            encoder,
            width,
            height
        )

        encoder.Create(
            VENC_CHANNEL,
            channel_attr
        )

        encoder_created = True

        print("[Video] H.264编码通道创建成功")
        print("[Video] H.264 Profile: Main")

        # ----------------------------------------------------
        # 7. 初始化RTSP服务器
        # ----------------------------------------------------

        print("")
        print("========================================")
        print("[RTSP] 初始化RTSP服务器")
        print("========================================")

        rtsp_server = mm.rtsp_server()

        rtsp_server.rtspserver_init(
            RTSP_PORT
        )

        rtsp_initialized = True

        rtsp_server.rtspserver_createsession(
            RTSP_SESSION,
            mm.multi_media_type.media_h264,
            False
        )

        rtsp_session_created = True

        rtsp_server.rtspserver_start()
        rtsp_started = True

        print("[RTSP] RTSP服务器启动成功")
        print("[RTSP] 端口:", RTSP_PORT)
        print("[RTSP] 会话:", RTSP_SESSION)

        # ----------------------------------------------------
        # 8. 启动编码器和摄像头
        # ----------------------------------------------------

        encoder.Start(
            VENC_CHANNEL
        )

        encoder_started = True

        print("[Video] H.264编码器启动成功")

        sensor.run()
        sensor_started = True

        print("[Video] 摄像头启动成功")

        rtsp_url = (
            rtsp_server.rtspserver_getrtspurl(
                RTSP_SESSION
            )
        )

        print("")
        print("================================================")
        print("高画质流畅版RTSP推流启动成功")
        print("播放地址:", rtsp_url)
        print("")
        print("目标参数：1280×720、30FPS、4000kbps")
        print("VLC网络缓存建议设置为2000～3000ms")
        print("RTSP时间戳使用官方示例值1000")
        print("================================================")
        print("")

        # ----------------------------------------------------
        # 9. 获取H.264码流并发送
        # ----------------------------------------------------

        stream_data = StreamData()

        total_frames = 0
        period_frames = 0
        period_bytes = 0

        stats_start_ms = time.ticks_ms()

        while True:
            os.exitpoint()

            result = encoder.GetStream(
                VENC_CHANNEL,
                stream_data
            )

            if result not in (0, None):
                print(
                    "[Video] GetStream异常返回:",
                    result
                )

                time.sleep_ms(2)
                continue

            try:
                frame_bytes = 0

                for pack_index in range(
                    stream_data.pack_cnt
                ):
                    packet_size = (
                        stream_data.data_size[
                            pack_index
                        ]
                    )

                    if packet_size <= 0:
                        continue

                    packet_data = bytes(
                        uctypes.bytearray_at(
                            stream_data.data[
                                pack_index
                            ],
                            packet_size
                        )
                    )

                    # 官方CanMV RTSP示例使用固定1000。
                    # 不再使用time.ticks_ms()，避免播放器时间轴异常。
                    rtsp_server.rtspserver_sendvideodata(
                        RTSP_SESSION,
                        packet_data,
                        packet_size,
                        RTSP_TIMESTAMP
                    )

                    frame_bytes += packet_size

                total_frames += 1
                period_frames += 1
                period_bytes += frame_bytes

            finally:
                encoder.ReleaseStream(
                    VENC_CHANNEL,
                    stream_data
                )

            # ------------------------------------------------
            # 统计实际FPS与平均码率
            # ------------------------------------------------

            now_ms = time.ticks_ms()

            elapsed_ms = time.ticks_diff(
                now_ms,
                stats_start_ms
            )

            if elapsed_ms >= STATS_PERIOD_MS:
                elapsed_s = elapsed_ms / 1000.0

                actual_fps = (
                    period_frames / elapsed_s
                )

                bitrate_mbps = (
                    period_bytes
                    * 8.0
                    / elapsed_s
                    / 1000000.0
                )

                print(
                    "[Stats] FPS:",
                    round(actual_fps, 1),
                    "码率:",
                    round(bitrate_mbps, 2),
                    "Mbps",
                    "总帧数:",
                    total_frames
                )

                period_frames = 0
                period_bytes = 0
                stats_start_ms = now_ms

    except KeyboardInterrupt as exc:
        print("")
        print(
            "[Main] 用户停止程序:",
            repr(exc)
        )

    except BaseException as exc:
        # CanMV IDE停止按钮有时抛出Exception('IDE interrupt')
        error_text = repr(exc)

        if "IDE interrupt" in error_text:
            print("")
            print("[Main] IDE停止程序")

        else:
            print("")
            print("========================================")
            print("[Main] 捕获到真实异常")
            print("[Main] 异常类型:", type(exc))
            print("[Main] 异常内容:", error_text)
            print("========================================")

    finally:
        print("")
        print("========================================")
        print("[Stop] 开始释放资源")
        print("========================================")

        # 1. 停止摄像头
        if sensor_started and sensor is not None:
            try:
                sensor.stop()
                print("[Stop] 摄像头已停止")

            except BaseException as exc:
                print(
                    "[Stop] sensor.stop异常:",
                    repr(exc)
                )

        # 2. 解除媒体绑定
        if media_link is not None:
            try:
                media_link.destroy()
                print("[Stop] 媒体绑定已解除")

            except BaseException as exc:
                print(
                    "[Stop] media_link.destroy提示:",
                    repr(exc)
                )

            media_link = None

        # 3. 停止编码器
        if encoder_started and encoder is not None:
            try:
                encoder.Stop(
                    VENC_CHANNEL
                )

                print("[Stop] H.264编码器已停止")

            except BaseException as exc:
                print(
                    "[Stop] encoder.Stop异常:",
                    repr(exc)
                )

        # 4. 销毁编码通道
        if encoder_created and encoder is not None:
            try:
                encoder.Destroy(
                    VENC_CHANNEL
                )

                print("[Stop] H.264编码通道已销毁")

            except BaseException as exc:
                print(
                    "[Stop] encoder.Destroy异常:",
                    repr(exc)
                )

        # 5. 停止RTSP服务器
        if rtsp_started and rtsp_server is not None:
            try:
                rtsp_server.rtspserver_stop()
                print("[Stop] RTSP服务器已停止")

            except BaseException as exc:
                print(
                    "[Stop] rtspserver_stop异常:",
                    repr(exc)
                )

        # 6. 销毁RTSP会话
        if (
            rtsp_session_created
            and rtsp_server is not None
        ):
            try:
                rtsp_server.rtspserver_destroysession(
                    RTSP_SESSION
                )

                print("[Stop] RTSP会话已销毁")

            except BaseException:
                pass

        # 7. RTSP反初始化
        if (
            rtsp_initialized
            and rtsp_server is not None
        ):
            try:
                rtsp_server.rtspserver_deinit()
                print("[Stop] RTSP服务器已反初始化")

            except BaseException as exc:
                print(
                    "[Stop] rtspserver_deinit异常:",
                    repr(exc)
                )

        # 8. 释放MediaManager
        if media_initialized:
            try:
                time.sleep_ms(100)

                MediaManager.deinit()
                print("[Stop] MediaManager已释放")

            except BaseException as exc:
                print(
                    "[Stop] MediaManager.deinit异常:",
                    repr(exc)
                )

        # 9. 断开Wi-Fi
        if wifi is not None:
            try:
                if wifi.isconnected():
                    wifi.disconnect()
                    print("[WiFi] 已断开")

            except BaseException as exc:
                print(
                    "[WiFi] 断开异常:",
                    repr(exc)
                )

        os.exitpoint(
            os.EXITPOINT_ENABLE_SLEEP
        )

        time.sleep_ms(100)

        print("[Main] 程序结束")


if __name__ == "__main__":
    main()
