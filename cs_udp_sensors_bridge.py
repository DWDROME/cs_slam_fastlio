#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket, select, time, sys, re, math, binascii
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointField

DEFAULT_LIDAR_IP   = "0.0.0.0"
DEFAULT_LIDAR_PORT = 9999
DEFAULT_LIDAR_FRAME_ID = "cs_lidar"
DEFAULT_PC_TOPIC   = "velodyne_points"

DEFAULT_IMU_IP     = "0.0.0.0"
DEFAULT_IMU_PORT   = 10000
DEFAULT_IMU_FRAME_ID = "cs_imu"
DEFAULT_IMU_TOPIC  = "cs/imu"

MAX_RANGE_M = 60.0
PRINT_STATS_EVERY = 1.0
DEBUG_HEX_FIRST_BAD_PER_SEC = 3

LINE5_RE = re.compile(
    r"^\s*([+-]?\d+(?:\.\d+)?)\s+"
    r"([+-]?\d+(?:\.\d+)?)\s+"
    r"([+-]?\d+(?:\.\d+)?)\s+"
    r"(\d+)\s+"
    r"([+-]?\d+(?:\.\d+)?)\s*$"
)
LINE5I_RE = re.compile(
    r"^\s*([+-]?\d+)\s+([+-]?\d+)\s+([+-]?\d+)\s+(\d+)\s+([+-]?\d+)\s*$"
)

def _finite(*vals): return all(math.isfinite(v) for v in vals)

def quat_mul(a, b):
    aw,ax,ay,az = a; bw,bx,by,bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )

# 32B  XYZIRT
FIELDS_XYZIRT_32 = [
    PointField('x',          0,  PointField.FLOAT32, 1),
    PointField('y',          4,  PointField.FLOAT32, 1),
    PointField('z',          8,  PointField.FLOAT32, 1),
    PointField('pad1',      12,  PointField.FLOAT32, 1),   # 对齐到16
    PointField('intensity', 16,  PointField.FLOAT32, 1),
    PointField('ring',      20,  PointField.UINT16,  1),
    PointField('pad2',      22,  PointField.UINT16,  1),   # 对齐到24
    PointField('time',      24,  PointField.FLOAT32, 1),
    PointField('pad3',      28,  PointField.UINT32,  1),   # 共32B
]
CONST_INTENSITY = 1.0

def main():
    rospy.init_node("cs_udp_sensors_bridge", anonymous=True)

    # =============== 参数 ===============
    # LiDAR
    use_accel = bool(rospy.get_param("~use_accel", True))  #

    lidar_ip   = rospy.get_param("~lidar_ip", DEFAULT_LIDAR_IP)
    lidar_port = int(rospy.get_param("~lidar_port", DEFAULT_LIDAR_PORT))
    lidar_frame= rospy.get_param("~lidar_frame_id", DEFAULT_LIDAR_FRAME_ID)
    lidar_topic= rospy.get_param("~lidar_topic", DEFAULT_PC_TOPIC)
    accept_tokens   = bool(rospy.get_param("~accept_frame_tokens", True))
    scan_period     = float(rospy.get_param("~scan_period", 0.07))
    xyz_only        = bool(rospy.get_param("~xyz_only", False))
    drop_far        = float(rospy.get_param("~drop_far_m", MAX_RANGE_M))
    frame_timeout   = float(rospy.get_param("~frame_timeout", 0.0))
    begin_starts_new_publish_old = bool(rospy.get_param("~begin_publish_previous_if_buffered", True))

    # IMU
    imu_ip   = rospy.get_param("~imu_ip", DEFAULT_IMU_IP)
    imu_port = int(rospy.get_param("~imu_port", DEFAULT_IMU_PORT))
    imu_frame= rospy.get_param("~imu_frame_id", DEFAULT_IMU_FRAME_ID)
    imu_topic= rospy.get_param("~imu_topic", DEFAULT_IMU_TOPIC)
    apply_rz_pi = bool(rospy.get_param("~apply_rz_pi", False)) 

    # 发布器
    pub_pc  = rospy.Publisher(lidar_topic, PointCloud2, queue_size=1, latch=True)
    pub_imu = rospy.Publisher(imu_topic, Imu, queue_size=200)

    # Socket
    sock_pc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_pc.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1<<20)
    sock_pc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_pc.bind((lidar_ip, lidar_port))
    sock_pc.setblocking(False)

    sock_imu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_imu.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1<<20)
    sock_imu.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_imu.bind((imu_ip, imu_port))
    sock_imu.setblocking(False)

    rospy.loginfo(f"[LiDAR] listening on {lidar_ip}:{lidar_port}, frame='{lidar_frame}', topic='{lidar_topic}'")
    rospy.loginfo(f"[IMU]   listening on {imu_ip}:{imu_port}, frame='{imu_frame}', topic='{imu_topic}', Rz(pi)={apply_rz_pi}")

    # =============== LiDAR 帧状态 ===============
    buf_pts = []   # list of (x,y,z,0.0,intensity,ring,0,time,0)
    frame_stamp = rospy.Time(0)
    in_frame = False
    t_last_frame_event = time.time()   # 最近一次收到 BEGIN/END/点 的时间

    # 统计
    last_log = 0.0
    shown_hex = 0
    dropped_bad = dropped_far_cnt = 0
    empty_frames = 0
    frames_ok = 0

    # IMU 修正四元数（左乘 Rz(pi)）
    q_fix = (0.0, 0.0, 0.0, 1.0) if apply_rz_pi else (1.0, 0.0, 0.0, 0.0)

    rate = rospy.Rate(500)

    FIELDS_XYZ = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    def publish_cloud(reason="END"):
        nonlocal buf_pts, frames_ok, empty_frames, frame_stamp, in_frame
        if not buf_pts:
            empty_frames += 1
            return
        hdr = Header(stamp=frame_stamp if frame_stamp != rospy.Time(0) else rospy.Time.now(),
                     frame_id=lidar_frame)
        if xyz_only:
            pts_xyz = [(p[0], p[1], p[2]) for p in buf_pts]
            cloud = pc2.create_cloud(hdr, FIELDS_XYZ, pts_xyz)
        else:
            cloud = pc2.create_cloud(hdr, FIELDS_XYZIRT_32, buf_pts)

        # 一秒一次 bbox 与样例点
        now_dbg = time.time()
        if not hasattr(publish_cloud, "_last_dbg") or now_dbg - publish_cloud._last_dbg > 1.0:
            xs = [p[0] for p in buf_pts]; ys = [p[1] for p in buf_pts]; zs = [p[2] for p in buf_pts]
            rospy.loginfo(f"[PC] bbox x[{min(xs):.2f},{max(xs):.2f}] y[{min(ys):.2f},{max(ys):.2f}] z[{min(zs):.2f},{max(zs):.2f}]  ({reason})")
            mid = len(buf_pts)//2
            rospy.loginfo(f"[PC] samples: {buf_pts[0][0]:.2f},{buf_pts[0][1]:.2f},{buf_pts[0][2]:.2f} | "
                          f"{buf_pts[mid][0]:.2f},{buf_pts[mid][1]:.2f},{buf_pts[mid][2]:.2f} | "
                          f"{buf_pts[-1][0]:.2f},{buf_pts[-1][1]:.2f},{buf_pts[-1][2]:.2f}")
            publish_cloud._last_dbg = now_dbg

        pub_pc.publish(cloud)
        frames_ok += 1
        rospy.loginfo_throttle(1.0, f"[PC] published frame #{frames_ok} with {len(buf_pts)} pts")

        buf_pts = []
        frame_stamp = rospy.Time(0)
        in_frame = False

    def safe_decode(data: bytes) -> str:
        return data.replace(b'\x00', b'').decode("utf-8", "ignore")

    def drain_pc_socket():
        nonlocal buf_pts, shown_hex, dropped_bad, dropped_far_cnt, frame_stamp, last_log, in_frame, t_last_frame_event
        lines_total = 0
        parsed_i = parsed_f = unparsable = 0

        while True:
            r,_,_ = select.select([sock_pc], [], [], 0.0)
            if not r:
                break
            data,_ = sock_pc.recvfrom(16384)
            text = safe_decode(data)

            for raw in text.splitlines():
                sline = raw.strip()
                if not sline:
                    continue
                lines_total += 1
                t_last_frame_event = time.time()

                # 帧标记
                if accept_tokens and (sline == "FRAME_BEGIN" or sline == "FRAME_END"):
                    if sline == "FRAME_BEGIN":
                        if buf_pts:
                            if begin_starts_new_publish_old:
                                rospy.logwarn("[PC] got BEGIN but buffer not empty -> publish previous frame (no END).")
                                publish_cloud(reason="BEGIN(no END prev)")
                            else:
                                rospy.logwarn("[PC] got BEGIN but buffer not empty -> drop previous frame.")
                                buf_pts.clear()
                        frame_stamp = rospy.Time.now()
                        in_frame = True
                    else: 
                        publish_cloud(reason="END")
                    continue

                if accept_tokens and not in_frame:
                    continue

                m = LINE5I_RE.match(sline)
                if m:
                    try:
                        xm = int(m.group(1)); ym = int(m.group(2)); zm = int(m.group(3))
                        ring = int(m.group(4)); t_us = int(m.group(5))
                        x = xm * 1e-3; y = ym * 1e-3; z = zm * 1e-3
                        t_rel = t_us * 1e-6
                        if drop_far > 0.0 and (x*x + y*y + z*z) > drop_far*drop_far:
                            dropped_far_cnt += 1
                            continue
                        parsed_i += 1
                        buf_pts.append((x, y, z, 0.0, float(CONST_INTENSITY), ring, 0, t_rel, 0))
                        continue
                    except Exception:
                        pass

                m = LINE5_RE.match(sline)
                if m:
                    try:
                        x = float(m.group(1)); y = float(m.group(2)); z = float(m.group(3))
                        ring = int(m.group(4)); t_rel = float(m.group(5))
                        if not _finite(x,y,z,t_rel):
                            raise ValueError("non-finite")
                        if drop_far > 0.0 and (x*x + y*y + z*z) > drop_far*drop_far:
                            dropped_far_cnt += 1
                            continue
                        parsed_f += 1
                        buf_pts.append((x, y, z, 0.0, float(CONST_INTENSITY), ring, 0, t_rel, 0))
                        continue
                    except Exception:
                        pass

                unparsable += 1
                dropped_bad += 1
                if DEBUG_HEX_FIRST_BAD_PER_SEC > 0 and shown_hex < DEBUG_HEX_FIRST_BAD_PER_SEC:
                    rospy.logwarn_throttle(
                        1.0, f"[PC] bad line: {repr(sline)} hex={binascii.hexlify(raw.encode('utf-8','ignore'))}"
                    )
                    shown_hex += 1

        # 每秒打印一次
        now = time.time()
        if (now - last_log) > PRINT_STATS_EVERY:
            rospy.loginfo(
                f"[PC] lines={lines_total} | parsed_int={parsed_i} parsed_float={parsed_f} "
                f"| unparsable={unparsable} | buf_pts={len(buf_pts)} in_frame={in_frame}"
            )
            shown_hex = 0
            last_log = now

        if frame_timeout > 0.0 and in_frame and (time.time() - t_last_frame_event) >= frame_timeout and buf_pts:
            rospy.logwarn(f"[PC] frame timeout ({frame_timeout:.3f}s) -> fallback publish.")
            publish_cloud(reason="TIMEOUT")

    def drain_imu_socket():
        nonlocal q_fix
        while True:
            r,_,_ = select.select([sock_imu], [], [], 0.0)
            if not r: break
            data,_ = sock_imu.recvfrom(4096)
            text = safe_decode(data)

            for sline in text.splitlines():
                sline = sline.strip()
                if not sline or not sline.startswith("IMU"):
                    continue
                parts = sline.split()
                if len(parts) not in (8,11):
                    continue
                try:
                    qw,qx,qy,qz = map(float, parts[1:5])
                    gx,gy,gz    = map(float, parts[5:8])
                    if len(parts)==11:
                        ax,ay,az = map(float, parts[8:11])
                    else:
                        ax=ay=az=None
                except Exception:
                    continue
                if not _finite(qw,qx,qy,qz,gx,gy,gz): continue
                if ax is not None and not _finite(ax,ay,az): continue

                if not use_accel:
                    ax = ay = az = None


                if apply_rz_pi:
                    gx, gy, gz = -gx, -gy, gz
                    qw,qx,qy,qz = quat_mul(q_fix, (qw,qx,qy,qz))

                n = math.sqrt(qw*qw+qx*qx+qy*qy+qz*qz)
                if n > 1e-6:
                    qw,qx,qy,qz = qw/n, qx/n, qy/n, qz/n

                msg = Imu()
                msg.header = Header(stamp=rospy.Time.now(), frame_id=imu_frame)
                msg.orientation.w = qw; msg.orientation.x = qx
                msg.orientation.y = qy; msg.orientation.z = qz
                msg.orientation_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.02]

                msg.angular_velocity.x = gx
                msg.angular_velocity.y = gy
                msg.angular_velocity.z = gz
                msg.angular_velocity_covariance = [0.02,0,0, 0,0.02,0, 0,0,0.02]

                if ax is None:
                    msg.linear_acceleration_covariance = [-1.0,0,0, 0,0,0, 0,0,0]
                else:
                    if apply_rz_pi:
                        ax,ay,az = -ax,-ay,az
                    msg.linear_acceleration.x = float(ax)
                    msg.linear_acceleration.y = float(ay)
                    msg.linear_acceleration.z = float(az)
                    msg.linear_acceleration_covariance = [0.2,0,0, 0,0.2,0, 0,0,0.2]

                pub_imu.publish(msg)

    # 主循环
    while not rospy.is_shutdown():
        drain_pc_socket()
        drain_imu_socket()
        rate.sleep()

if __name__ == "__main__":
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(line_buffering=True)
        main()
    except rospy.ROSInterruptException:
        pass
