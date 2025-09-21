#include <amxmodx>
#include <engine>
#include <sockets>
#include <fakemeta>
#include <fakemeta_util>   // 外挂的引擎接口，提供 create_tr2 / free_tr2 / get_tr2


#define PLUGIN   "CS16 Virtual LiDAR 32L (XYZ+RING+TIME)"
#define VERSION  "1.0.0"
#define AUTHOR   "先生-POC"

// ====== 目标地址 ======
#define SERVER_IP   "10.63.249.91"
// #define SERVER_IP   "10.61.69.189"
#define SERVER_PORT 9999

// ====== 单位与常量 ======
#define UNIT2M      0.0254            // GoldSrc -> m
#define PACK_CHUNK  1200
#define BIGBUF_SIZE 8192

// ====== CVAR ======
new cvar:g_cv_verbose;        // 0静默 1帧统计 2+打印样例点
new cvar:g_cv_interval;       // scan_period (s)
new cvar:g_cv_degstep;        // 水平分辨率（度）
new cvar:g_cv_aliveonly;      // 仅活着的玩家
new cvar:g_cv_limit_on;       // 限幅开关（当前未用，可留作开关位）
new cvar:g_cv_subs_az;        // 方位细分数(>=1)，默认1
new cvar:g_cv_interleav;      // 半步交错(0/1)
new cvar:g_cv_nscan;          // 线数，默认32
new cvar:g_cv_elev_bottom;    // 最下俯仰角(度)，默认-30
new cvar:g_cv_elev_res;       // 垂直分辨率(度)，默认1.333333(≈32线)

// UDP 合包策略
new cvar:g_cv_pack_mode;      // 0=不合 1=合包
new cvar:g_cv_pkt_size;       // 单包最大字节数

// ---- IMU cvars ----
new cvar:g_cv_imu_rate;          // 10..200 Hz
new cvar:g_cv_imu_fix_rxpi;      // 0/1  左乘 Rx(pi)
new cvar:g_cv_imu_use_grav;      // 0/1  输出 specific force
new cvar:g_cv_imu_g_mps2;        // 9.81
new cvar:g_cv_imu_lpf_a;         // 0..1  一阶低通


new g_sock = -1;

// 统计
new g_cnt_limitdrop;
new g_cnt_sendok;

// 合包缓冲
static bigbuf[BIGBUF_SIZE];
static used;

// -----------------------------------------------------------
public plugin_init()
{
    register_plugin(PLUGIN, VERSION, AUTHOR);

    g_cv_verbose      = register_cvar("lidar_verbose",        "1");
    g_cv_interval     = register_cvar("lidar_interval_s",     "0.07");
    g_cv_degstep      = register_cvar("lidar_deg_step",       "2");
    g_cv_aliveonly    = register_cvar("lidar_alive_only",     "1");
    g_cv_limit_on     = register_cvar("lidar_limit_enable",   "1");
    g_cv_subs_az      = register_cvar("lidar_subs_az",        "1");
    g_cv_interleav    = register_cvar("lidar_interleave",     "1");

    g_cv_nscan        = register_cvar("lidar_n_scan",         "32");
    g_cv_elev_bottom  = register_cvar("lidar_elev_bottom",    "-30.0");
    g_cv_elev_res     = register_cvar("lidar_elev_res",       "1.333333");

    // UDP 合包策略
    g_cv_pack_mode    = register_cvar("lidar_pack_mode",      "1");
    g_cv_pkt_size     = register_cvar("lidar_pkt_size",       "1200");

    // IMU
    imu_register_cvars();
    set_task(0.6, "imu_init_socket");
    set_task(0.9, "imu_start_loop");

    set_task(1.0, "init_socket");
    set_task(1.5, "start_scan_task");
}

public init_socket()
{
    if (g_sock != -1) return;
    new err = 0;
    g_sock = socket_open(SERVER_IP, SERVER_PORT, SOCKET_UDP, err);
    if (g_sock <= 0 || err != 0)
        server_print("[LiDAR] UDP connect failed %s:%d (err=%d)", SERVER_IP, SERVER_PORT, err);
    else
        server_print("[LiDAR] UDP connected %s:%d", SERVER_IP, SERVER_PORT);
}

public start_scan_task()
{
    new Float:itv = get_pcvar_float(g_cv_interval);
    if (itv < 0.02) itv = 0.02;
    //set_task(Float:time, const function[], id=0, const parameter[]="", len=0, const flags[]="“b”循环/“a和同id叠加”/“f忘了”")
    set_task(itv, "do_scan", 0, "", 0, "b");
}


// ------- UDP 工具、-------
stock get_pkt_budget()
{
    new mtu = get_pcvar_num(g_cv_pkt_size);
    if (mtu < 600)  mtu = 600;
    if (mtu > 1400) mtu = 1400;
    return mtu;
}

stock flush_points_chunk()
{
    if (used <= 0) return;
    if (g_sock > 0) socket_send(g_sock, bigbuf, used);
    used = 0;
}

// -----------------------------------------------------------
// —— 发送层（默认合包合包 ）
// -----------------------------------------------------------
stock send_line_token(const msg[])
{
    if (g_sock <= 0) return;
    static tmp[32];
    new l = formatex(tmp, charsmax(tmp), "%s", msg);
    tmp[l]   = 13;  // \r
    tmp[l+1] = 10;  // \n
    socket_send(g_sock, tmp, l + 2);
}

stock frame_begin()
{
    used = 0;
    flush_points_chunk();     // 检查一下，重置缓冲区。
    send_line_token("FRAME_BEGIN");
}

stock frame_add_point5(Float:x_m, Float:y_m, Float:z_m, ring, Float:t)
{
    static line[128];
    new n = formatex(line, charsmax(line), "%.3f %.3f %.3f %d %.6f", x_m, y_m, z_m, ring, t);
    line[n]   = 13; line[n+1] = 10;
    new line_len = n + 2;

    new mode = get_pcvar_num(g_cv_pack_mode);

    if (mode <= 0) {
        if (g_sock > 0) socket_send(g_sock, line, line_len);  // 逐行一包
        return;
    }

    new budget = get_pkt_budget();

    //判断一下极端情况
    if (line_len >= budget) {
        flush_points_chunk();
        if (g_sock > 0) socket_send(g_sock, line, line_len);
        return;
    }

    // 超包了
    if (used + line_len > budget) {
        flush_points_chunk();
    }

    // 追加到缓冲
    for (new i = 0; i < line_len; i++)
        bigbuf[used + i] = line[i];
    used += line_len;
}

stock frame_end_and_send()
{
    flush_points_chunk();           // 发完
    send_line_token("FRAME_END");   // 结束标记单独一包
}
// -----------------------------------------------------------
// 玩家检测
// -----------------------------------------------------------
stock bool:is_valid_player(id)
{
    if (!is_user_connected(id)) return false;
    if (get_pcvar_num(g_cv_aliveonly) && !is_user_alive(id)) return false;
    return true;
}

stock find_carrier_player()  
{
    if (is_valid_player(1)) return 1;
    return 0;
}

// ===== 坐标系工具 =====

// 由玩家视角角度生成正交基
// 机体系 B 定义为：X=forward, Y=left(= -right), Z=up
// 找的代码。
stock basis_from_angles(id, Float:ex[3], Float:ey[3], Float:ez[3])
{
    static Float:ang[3], Float:f[3], Float:r[3], Float:u[3];
    entity_get_vector(id, EV_VEC_v_angle, ang);  // [pitch,yaw,roll], deg
    engfunc(EngFunc_MakeVectors, ang);
    /*
    engfunc在Fakemeta中，调用其中的EngFunc_MakeVectors，通过输入pitch、yaw、roll来生成下面三个。
    */
    global_get(glb_v_forward, f);
    global_get(glb_v_right,   r);
    global_get(glb_v_up,      u);

    ex[0]=f[0];   ex[1]=f[1];   ex[2]=f[2];     // X_b
    ey[0]=-r[0];  ey[1]=-r[1];  ey[2]=-r[2];    // Y_b = -right  因为我建图的时候发现视角和rviz转的不同。
    ez[0]=u[0];   ez[1]=u[1];   ez[2]=u[2];     // Z_b
}


// 在机体系（B）按方位/俯仰构造单位向量 b = (bx,by,bz)
// 约定：az 绕 Z_b，从 X_b 朝 Y_b（左）为正；el 绕 Y_b，从 X_b 抬头为正
stock body_unit_from_azel(Float:az_deg, Float:el_deg, &Float:bx, &Float:by, &Float:bz)
{
    new Float:cb = floatcos(az_deg, degrees);
    new Float:sb = floatsin(az_deg, degrees);
    new Float:ce = floatcos(el_deg, degrees);
    new Float:se = floatsin(el_deg, degrees);

    bx = ce * cb;      // 沿 X_b 的分量
    by = ce * sb;      // 沿 Y_b 的分量（左为正）
    bz = se;           // 沿 Z_b 的分量（上为正）
}

// 用基向量把机体系方向 b 映射到世界系 w：w = ex*bx + ey*by + ez*bz
// 因为我们的雷达构建需要在世界坐标系上构建，即使用trace_line和碰撞检测等。
stock body_dir_to_world(const Float:ex[3], const Float:ey[3], const Float:ez[3],
                        Float:bx, Float:by, Float:bz,
                        &Float:wx, &Float:wy, &Float:wz)
{
    wx = ex[0]*bx + ey[0]*by + ez[0]*bz;
    wy = ex[1]*bx + ey[1]*by + ez[1]*bz;
    wz = ex[2]*bx + ey[2]*by + ez[2]*bz;
}

// 把世界位移向量 d_w（米）投回机体系：d_b = (dot(d,ex), dot(d,ey), dot(d,ez))
// 计算完了需要逆运算。
stock project_world_to_body(const Float:ex[3], const Float:ey[3], const Float:ez[3],
                            Float:dx_m, Float:dy_m, Float:dz_m,
                            &Float:x_b, &Float:y_b, &Float:z_b)
{
    x_b = ex[0]*dx_m + ex[1]*dy_m + ex[2]*dz_m;
    y_b = ey[0]*dx_m + ey[1]*dy_m + ey[2]*dz_m;
    z_b = ez[0]*dx_m + ez[1]*dy_m + ez[2]*dz_m;
}

// 归一化方位角到 [0,360)
stock Float:norm_az(Float:az_deg)
{
    while (az_deg >= 360.0) az_deg -= 360.0;
    while (az_deg <   0.0)  az_deg += 360.0;
    return az_deg;
}


// -----------------------------------------------------------
// 主循环：一帧扫描（32线 XYZ+ring+time）
// -----------------------------------------------------------
public do_scan()
{
    if (g_sock <= 0) return;

    new id = find_carrier_player();
    if (!is_valid_player(id)) return;

    // 眼睛位置（世界系，GoldSrc）
    new eye_i[3];
    get_user_origin(id, eye_i, 1);
    static Float:eye[3];
    eye[0]=float(eye_i[0]); eye[1]=float(eye_i[1]); eye[2]=float(eye_i[2]);

    // —— 由引擎角度获得正交基：B 系 {X=forward, Y=left, Z=up} —— //
    static Float:ex[3], Float:ey[3], Float:ez[3];
    basis_from_angles(id, ex, ey, ez);

    // 参数
    new n_scan      = get_pcvar_num(g_cv_nscan);       if (n_scan < 1) n_scan = 32;
    new Float:ang0  = get_pcvar_float(g_cv_elev_bottom);
    new Float:vres  = get_pcvar_float(g_cv_elev_res);  if (vres <= 0.0) vres = 1.333333;
    new step        = get_pcvar_num(g_cv_degstep);     if (step < 1) step = 1; if (step > 90) step = 90;
    new subs_az     = clamp(get_pcvar_num(g_cv_subs_az), 1, 4);
    new interleav   = get_pcvar_num(g_cv_interleav);
    new Float:Tscan = get_pcvar_float(g_cv_interval);

    static frame_parity = 0; frame_parity = 1 - frame_parity;
    new Float:halfshift = (interleav && frame_parity) ? float(step) * 0.5 : 0.0;

    // 统计
    new miss_cnt = 0, total_cnt = 0;
    g_cnt_sendok = 0;

    frame_begin();

    // 最大量程（GoldSrc units / meters）
    const Float:MAX_M     = 50.0;
    new   Float:max_units = MAX_M / UNIT2M;

    // === 扫描 ===
    for (new ring = 0; ring < n_scan; ring++)
    {
        new Float:elev = ang0 + float(ring)*vres;

        for (new deg = 0; deg < 360; deg += step)
        {
            for (new s = 0; s < subs_az; s++)
            {
                // —— 方位角（含半步交错 + 均匀细分）——
                new Float:frac   = (subs_az==1) ? 0.0 : (float(s)+0.5)/float(subs_az);
                new Float:az_deg = float(deg) + halfshift + float(step)*frac;

                // 1) 机体系单位射线方向 b
                new Float:bx, Float:by, Float:bz;
                body_unit_from_azel(az_deg, elev, bx, by, bz);

                // 2) 映射到世界系 w，做 Trace
                new Float:wx, Float:wy, Float:wz;
                body_dir_to_world(ex, ey, ez, bx, by, bz, wx, wy, wz);

                static Float:endp[3], Float:hit_w[3];
                endp[0]=eye[0] + wx*max_units;
                endp[1]=eye[1] + wy*max_units;
                endp[2]=eye[2] + wz*max_units;

                trace_line(id, eye, endp, hit_w);

                // 3) 命中位移（世界系）→ 米
                new Float:dx_m = (hit_w[0]-eye[0]) * UNIT2M;
                new Float:dy_m = (hit_w[1]-eye[1]) * UNIT2M;
                new Float:dz_m = (hit_w[2]-eye[2]) * UNIT2M;

                // 距离 / 漏扫统计
                new Float:r_m = floatsqroot(dx_m*dx_m + dy_m*dy_m + dz_m*dz_m);
                if (r_m >= (MAX_M - 0.05)) { miss_cnt++; total_cnt++; continue; }
                total_cnt++;

                // 4) 投回机体系（一次性点积；与 IMU 统一基准）
                new Float:x_m, Float:y_m, Float:z_m;
                project_world_to_body(ex, ey, ez, dx_m, dy_m, dz_m, x_m, y_m, z_m);

                // 5) 点内时间戳（按方位映射）
                new Float:az_n = norm_az(az_deg);
                new Float:t    = (az_n / 360.0) * Tscan;

                // 6) 发送（x,y,z, ring, t）
                frame_add_point5(x_m, y_m, z_m, ring, t);
                g_cnt_sendok++;
            }
        }
    }

    frame_end_and_send();

    if (get_pcvar_num(g_cv_verbose) >= 1)
    {
        new pct = (total_cnt>0) ? (miss_cnt*100/total_cnt) : 0;
        server_print("[LiDAR32] frame sent: %d pts | miss=%d/%d(%d%%) | n_scan=%d, elev=[%.1f..%.1f]/%.3f | step=%d subs=%d inter=%d",
                     g_cnt_sendok, miss_cnt, total_cnt, pct,
                     n_scan, get_pcvar_float(g_cv_elev_bottom),
                     get_pcvar_float(g_cv_elev_bottom)+(float(n_scan-1)*get_pcvar_float(g_cv_elev_res)),
                     get_pcvar_float(g_cv_elev_res), step, subs_az, interleav);
    }
}


// ========================= IMU SECTION (embed-ready) =========================
#define IMU_IP    "10.63.249.91"
#define IMU_PORT  10000
#define DEG2RAD   (3.1415926535 / 180.0)
#define UNIT2M    0.0254         // GoldSrc units -> meters 引擎单位和标准米的换算。

// ---- IMU socket ----
new g_sock_imu = -1;

// 点积
stock Float:dot3(const Float:a[3], const Float:b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// 由基向量（列：ex,ey,ez）构造四元数（B->W），返回 (qw,qx,qy,qz)
stock quat_from_basis(const Float:ex[3], const Float:ey[3], const Float:ez[3],
                      &Float:qw, &Float:qx, &Float:qy, &Float:qz)
{
    new Float:r00 = ex[0], r01 = ey[0], r02 = ez[0];
    new Float:r10 = ex[1], r11 = ey[1], r12 = ez[1];
    new Float:r20 = ex[2], r21 = ey[2], r22 = ez[2];


    new Float:trace = r00 + r11 + r22;
    new Float:eps = 0.000000001;

    if (trace > 0.0) {
        new Float:s = floatsqroot(trace + 1.0) * 2.0; if (s < eps) s = eps;
        qw = 0.25 * s;
        qx = (r21 - r12) / s;
        qy = (r02 - r20) / s;
        qz = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        new Float:s = floatsqroot(1.0 + r00 - r11 - r22) * 2.0; if (s < eps) s = eps;
        qw = (r21 - r12) / s;
        qx = 0.25 * s;
        qy = (r01 + r10) / s;
        qz = (r02 + r20) / s;
    } else if (r11 > r22) {
        new Float:s = floatsqroot(1.0 + r11 - r00 - r22) * 2.0; if (s < eps) s = eps;
        qw = (r02 - r20) / s;
        qx = (r01 + r10) / s;
        qy = 0.25 * s;
        qz = (r12 + r21) / s;
    } else {
        new Float:s = floatsqroot(1.0 + r22 - r00 - r11) * 2.0; if (s < eps) s = eps;
        qw = (r10 - r01) / s;
        qx = (r02 + r20) / s;
        qy = (r12 + r21) / s;
        qz = 0.25 * s;
    }

    new Float:n = floatsqroot(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n > 0.0) { qw /= n; qx /= n; qy /= n; qz /= n; }
}




// ---------------- 注册与启动（在 plugin_init() 里调用这三个函数） ----------------
public imu_register_cvars()
{
    g_cv_imu_rate     = register_cvar("imu_rate_hz",        "100");
    g_cv_imu_fix_rxpi = register_cvar("imu_fix_rxpi",       "1");
    g_cv_imu_use_grav = register_cvar("imu_use_gravity",    "1");
    g_cv_imu_g_mps2   = register_cvar("imu_gravity_mps2",   "9.81");
    g_cv_imu_lpf_a    = register_cvar("imu_acc_lpf_alpha",  "0.15");
}

public imu_init_socket()
{
    if (g_sock_imu != -1) return;
    new err = 0;
    g_sock_imu = socket_open(IMU_IP, IMU_PORT, SOCKET_UDP, err);
    if (g_sock_imu <= 0 || err != 0)
        server_print("[IMU] UDP connect failed %s:%d (err=%d)", IMU_IP, IMU_PORT, err);
    else
        server_print("[IMU] UDP connected %s:%d", IMU_IP, IMU_PORT);
}

public imu_start_loop()
{
    new Float:hz = get_pcvar_float(g_cv_imu_rate);
    if (hz < 10.0) hz = 10.0;
    if (hz > 200.0) hz = 200.0;
    new Float:dt = 1.0 / hz;
    set_task(dt, "imu_tick", 0, "", 0, "b");
}

public plugin_end()
{
    if (g_sock > 0) {
        socket_close(g_sock);
        g_sock = -1;
    }
    if (g_sock_imu > 0) {
        socket_close(g_sock_imu);
        g_sock_imu = -1;
    }
}


// ---------------- 发送一行 ----------------
stock imu_send_line(Float:qw, Float:qx, Float:qy, Float:qz,
                    Float:gx, Float:gy, Float:gz,
                    Float:ax, Float:ay, Float:az)
{
    if (g_sock_imu <= 0) return;

    static line[192];
    new n = formatex(line, charsmax(line),
        "IMU %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f",
        qw,qx,qy,qz, gx,gy,gz, ax,ay,az);
    line[n]   = 13;  // \r
    line[n+1] = 10;  // \n
    line[n+2] = 0;   // \0
    socket_send(g_sock_imu, line, n + 3);
}

public imu_tick()
{
    if (g_sock_imu <= 0) return;

    new id = find_carrier_player();
    if (!is_valid_player(id)) return;

    // === 基向量（与 LiDAR 完全一致的 B 系） ===
    static Float:ex[3], ey[3], ez[3];
    basis_from_angles(id, ex, ey, ez);   // X=forward, Y=left, Z=up

    // === 时间步长（带硬保护） ===
    new Float:t  = get_gametime();
    static bool:has_prev = false;
    static Float:t_prev = 0.0;

    new Float:dt;
    if (!has_prev) { dt = 0.01; t_prev = t; }
    else           { dt = t - t_prev; t_prev = t; }

    if (dt < 0.005 || dt > 0.2) {
        dt = 0.01;
        has_prev = false;
    }

    // === 四元数（B->W） ===
    new Float:qw, qx, qy, qz;
    quat_from_basis(ex, ey, ez, qw, qx, qy, qz);

    // 可选：左乘 Rx(pi) 修正（一般不用）
    if (get_pcvar_num(g_cv_imu_fix_rxpi)) {
        // q' = [0,1,0,0] * q = [-qx, qw, qz, -qy]
        new Float:nqw = -qx;
        new Float:nqx =  qw;
        new Float:nqy =  qz;
        new Float:nqz = -qy;
        qw=nqw; qx=nqx; qy=nqy; qz=nqz;
    }

    // === 角速度 ω_b（体坐标） ===
    static Float:ex_prev[3], ey_prev[3], ez_prev[3];
    new Float:gx=0.0, gy=0.0, gz=0.0;
    if (has_prev) {
        angvel_body_from_bases(ex_prev, ey_prev, ez_prev, ex, ey, ez, dt, gx, gy, gz);
    }

    // === 世界速度（GoldSrc→m/s） + basevelocity 融合 ===
    static Float:vel_gs[3], base_gs[3];
    entity_get_vector(id, EV_VEC_velocity,     vel_gs);
    entity_get_vector(id, EV_VEC_basevelocity, base_gs);

    new Float:vel_w[3], base_w[3], vtot_w[3];
    vel_w[0]  = float(vel_gs[0])  * UNIT2M;
    vel_w[1]  = float(vel_gs[1])  * UNIT2M;
    vel_w[2]  = float(vel_gs[2])  * UNIT2M;
    base_w[0] = float(base_gs[0]) * UNIT2M;
    base_w[1] = float(base_gs[1]) * UNIT2M;
    base_w[2] = float(base_gs[2]) * UNIT2M;

    vtot_w[0] = vel_w[0] + base_w[0];
    vtot_w[1] = vel_w[1] + base_w[1];
    vtot_w[2] = vel_w[2] + base_w[2];

    // === 世界线加速度（差分 + 语义修正） ===
    static bool:vtot_init = false;
    static Float:vtot_prev_w[3] = {0.0,0.0,0.0};
    new Float:acc_w[3] = {0.0,0.0,0.0};

    if (!vtot_init || !has_prev) {
        vtot_prev_w[0] = vtot_w[0];
        vtot_prev_w[1] = vtot_w[1];
        vtot_prev_w[2] = vtot_w[2];
        vtot_init = true;
    } else {
        acc_w[0] = (vtot_w[0] - vtot_prev_w[0]) / dt;
        acc_w[1] = (vtot_w[1] - vtot_prev_w[1]) / dt;
        acc_w[2] = (vtot_w[2] - vtot_prev_w[2]) / dt;
        vtot_prev_w[0] = vtot_w[0];
        vtot_prev_w[1] = vtot_w[1];
        vtot_prev_w[2] = vtot_w[2];
    }

    // 语义：地面/水/低速抑制
    new flags = entity_get_int(id, EV_INT_flags);
    new on_ground  = (flags & FL_ONGROUND) ? 1 : 0;
    new waterlevel = entity_get_int(id, EV_INT_waterlevel);

    // 地面：Z 轴加速度置稳态
    if (on_ground) {
        acc_w[2] = 0.0;
    } else {
        const Float:AZ_MAX = 30.0; // m/s^2
        if (acc_w[2] < -AZ_MAX) acc_w[2] = -AZ_MAX; else if (acc_w[2] > AZ_MAX) acc_w[2] = AZ_MAX;
    }

    // 水中：水平加速度降权
    new Float:a_xy_scale = (waterlevel > 1) ? 0.3 : 1.0;
    acc_w[0] *= a_xy_scale;
    acc_w[1] *= a_xy_scale;

    // 低速消抖（人物几乎静止时不产生水平线加速度）
    new Float:speed_xy = floatsqroot(vtot_w[0]*vtot_w[0] + vtot_w[1]*vtot_w[1]);
    if (speed_xy < 0.2) { acc_w[0] = 0.0; acc_w[1] = 0.0; }

    // 水平加速度极限（人物极限推进）
    const Float:AXY_MAX = 8.0; // 推荐5~12 
    if (acc_w[0] < -AXY_MAX) acc_w[0] = -AXY_MAX; else if (acc_w[0] > AXY_MAX) acc_w[0] = AXY_MAX;
    if (acc_w[1] < -AXY_MAX) acc_w[1] = -AXY_MAX; else if (acc_w[1] > AXY_MAX) acc_w[1] = AXY_MAX;

    // === 重力加速度计算：a_meas_w = a_w - g_w = a_w + (0,0,g) ===
    if (get_pcvar_num(g_cv_imu_use_grav)) {
        new Float:g = get_pcvar_float(g_cv_imu_g_mps2);
        acc_w[2] += g;
    }

    // === 世界 -> 机体，加速度投影到 B 系 ===
    new Float:ax_b, ay_b, az_b;
    project_world_to_body(ex, ey, ez, acc_w[0], acc_w[1], acc_w[2], ax_b, ay_b, az_b);

    // 数值极限（10g）
    const Float:A_CLAMP = 98.1;
    if (ax_b < -A_CLAMP) ax_b = -A_CLAMP; else if (ax_b > A_CLAMP) ax_b = A_CLAMP;
    if (ay_b < -A_CLAMP) ay_b = -A_CLAMP; else if (ay_b > A_CLAMP) ay_b = A_CLAMP;
    if (az_b < -A_CLAMP) az_b = -A_CLAMP; else if (az_b > A_CLAMP) az_b = A_CLAMP;

    // 一阶低通
    static bool:lpf_init = false;
    static Float:acc_b_lpf[3] = {0.0,0.0,0.0};
    new Float:alpha = get_pcvar_float(g_cv_imu_lpf_a);
    if (alpha < 0.0) alpha = 0.0; else if (alpha > 1.0) alpha = 1.0;

    if (!lpf_init || !has_prev) {
        acc_b_lpf[0] = ax_b; acc_b_lpf[1] = ay_b; acc_b_lpf[2] = az_b;
        lpf_init = true;
    } else {
        acc_b_lpf[0] = alpha*acc_b_lpf[0] + (1.0 - alpha)*ax_b;
        acc_b_lpf[1] = alpha*acc_b_lpf[1] + (1.0 - alpha)*ay_b;
        acc_b_lpf[2] = alpha*acc_b_lpf[2] + (1.0 - alpha)*az_b;
    }

    // === 发送（qw qx qy qz  gx gy gz  ax ay az） ===
    imu_send_line(qw,qx,qy,qz, gx,gy,gz, acc_b_lpf[0], acc_b_lpf[1], acc_b_lpf[2]);

    // === 更新上一帧基 ===
    ex_prev[0]=ex[0]; ex_prev[1]=ex[1]; ex_prev[2]=ex[2];
    ey_prev[0]=ey[0]; ey_prev[1]=ey[1]; ey_prev[2]=ey[2];
    ez_prev[0]=ez[0]; ez_prev[1]=ez[1]; ez_prev[2]=ez[2];
    has_prev = true;
}

// 由上一帧与当前帧基向量求体坐标角速度 ω_b（rad/s）
// R_delta = R_prev^T * R_curr；小角速度近似：
// ωx = (R_delta[2,1]-R_delta[1,2])/(2*dt), 同理求 ωy, ωz
stock angvel_body_from_bases(const Float:ex_prev[3], const Float:ey_prev[3], const Float:ez_prev[3],
                             const Float:ex[3],      const Float:ey[3],      const Float:ez[3],
                             Float:dt, &Float:gx, &Float:gy, &Float:gz)
{
    if (dt <= 0.0) { gx=0.0; gy=0.0; gz=0.0; return; }
    if (dt < 0.001) dt = 0.001; // 1 ms 下限，避免极端分母

    new Float:m01 = ex_prev[0]*ey[0] + ex_prev[1]*ey[1] + ex_prev[2]*ey[2];
    new Float:m02 = ex_prev[0]*ez[0] + ex_prev[1]*ez[1] + ex_prev[2]*ez[2];

    new Float:m10 = ey_prev[0]*ex[0] + ey_prev[1]*ex[1] + ey_prev[2]*ex[2];
    new Float:m12 = ey_prev[0]*ez[0] + ey_prev[1]*ez[1] + ey_prev[2]*ez[2];

    new Float:m20 = ez_prev[0]*ex[0] + ez_prev[1]*ex[1] + ez_prev[2]*ex[2];
    new Float:m21 = ez_prev[0]*ey[0] + ez_prev[1]*ey[1] + ez_prev[2]*ey[2];

    gx = (m21 - m12) / (2.0 * dt);
    gy = (m02 - m20) / (2.0 * dt);
    gz = (m10 - m01) / (2.0 * dt);
}
