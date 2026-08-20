#!/usr/bin/env python3
# =============================================================
# slope_cloud_filter
#
# 격자(폴라 그리드) 기반 지역 지면 추정 방식.
#
# 기존 방식과의 차이:
#   - pitch 게이팅 없음. 로봇이 평지에 있든 경사로 위에 있든
#     항상 동일하게 동작함.
#   - 고정 각도밴드/고정 높이밴드 없음. 방위각-거리 셀마다
#     지면 높이를 따로 추정하고, "그 지역 지면으로부터의 높이"가
#     임계값을 넘는 점만 장애물로 남김.
#   - 경사면은 셀마다 지면 높이가 같이 올라가므로 자동으로 제거됨.
#     경사로 위의 사람/벽은 지역 지면보다 높으므로 그대로 남음.
#
# 출력 좌표계는 입력과 동일한 BODY frame 유지.
# =============================================================
import math
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rcl_interfaces.msg import SetParametersResult


class SlopeCloudFilter(Node):
    def __init__(self):
        super().__init__('slope_cloud_filter')

        # =====================================================
        # Topics
        # =====================================================
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('input_cloud_topic', '/cloud_registered_body')
        self.declare_parameter('output_cloud_topic', '/cloud_global_filtered')

        # =====================================================
        # 센서 기하
        # =====================================================
        # BODY 원점에서 지면까지의 거리(= LiDAR 설치 높이).
        # 첫 번째 링의 지면 높이 초기 추정값으로만 쓰임.
        self.declare_parameter('lidar_height_m', 0.35)

        # true: odom의 roll/pitch로 포인트를 중력정렬 프레임으로 회전.
        #       (yaw는 제거해서 격자는 로봇 전방 기준 유지)
        # false: BODY frame 그대로 사용. odom이 불안정하면 false 권장.
        self.declare_parameter('use_level_frame', True)

        # =====================================================
        # 폴라 그리드 설정
        # =====================================================
        # 방위각 분할 수. 72이면 5도 간격.
        self.declare_parameter('azimuth_bins', 36)
        # 거리 방향 셀 크기(m)
        self.declare_parameter('radial_bin_size_m', 0.5)
        # 처리할 거리 범위(m)
        self.declare_parameter('min_range_m', 0.3)
        self.declare_parameter('max_range_m', 15.0)

        # =====================================================
        # 지면 추정 / 장애물 판정
        # =====================================================
        # 주행 가능하다고 볼 최대 지면 경사(도).
        # 이 경사 이내로 높아지는 표면은 "지면(경사로)"으로 인정.
        # 이보다 급하게 튀어오르면 지면이 아니라 장애물/벽으로 봄.
        self.declare_parameter('max_ground_slope_deg', 20.0)

        # 셀 간 지면 높이 변화 허용 여유(m). 노이즈 흡수용.
        self.declare_parameter('ground_jump_tolerance_m', 0.10)

        # 지역 지면보다 이만큼 이상 높아야 장애물로 인정(m).
        # 값이 작으면 노면 요철까지 장애물이 되고,
        # 크면 낮은 연석/턱을 놓침.
        self.declare_parameter('obstacle_height_thresh_m', 0.25)

        # 지역 지면 기준 이 높이 이상은 무시(m). 나뭇가지/천장 등.
        self.declare_parameter('max_obstacle_height_m', 2.0)

        # =====================================================
        # 스캔 누적
        #
        # FAST-LIO의 /cloud_registered_body는 스캔 매칭용으로
        # 데시메이션된 클라우드라 1프레임 ~1700점으로 매우 성기다.
        # 그 상태로는 먼 거리에 광선을 쏠 점이 없어서 costmap의
        # clearing이 로봇 근처 몇 미터에서 끝나고, 그 바깥은
        # 관측되지 않은 채 unknown(회색)으로 남는다.
        #
        # 최근 N개 스캔을 odom으로 보정해 현재 body frame에 모아
        # 밀도를 N배로 올린다.
        #
        # 1 = 누적 안 함
        # 5 = 0.5초분 (10Hz 기준). 움직이는 물체는 약간 번진다.
        # =====================================================
        self.declare_parameter('accumulate_scans', 5)

        self.declare_parameter('log_period_sec', 1.0)

        # 디버그: 제거된 점을 별도 토픽으로 발행 (RViz 확인용)
        self.declare_parameter('debug_removed_topic', '/cloud_slope_removed')
        self.declare_parameter('publish_removed', True)

        # =====================================================
        # clearing 전용 출력
        #
        # costmap의 raytracing은 2D라서, 원본 클라우드를 그대로
        # clearing에 쓰면 나무 캐노피(지상 15m)로 가는 광선이
        # 그 아래 실제 장애물 셀까지 관통해서 지워버린다.
        #
        # 그래서 "지역 지면 기준 max_obstacle_height_m 이하"인 점만
        # 모아서 별도 토픽으로 낸다.
        #   - 지면 점이 포함되므로 지나온 길을 쓸어준다
        #   - 캐노피가 빠지므로 과도한 clearing이 없다
        # =====================================================
        self.declare_parameter('output_clearing_topic', '/cloud_ground_clearing')
        self.declare_parameter('publish_clearing', True)
        # 지면보다 이만큼 아래는 노이즈로 보고 clearing에서도 제외(m)
        self.declare_parameter('clearing_min_height_m', -1.0)

        # =====================================================
        # Parameter 읽기
        # =====================================================
        gp = self.get_parameter
        self.odom_topic = gp('odom_topic').value
        self.input_cloud_topic = gp('input_cloud_topic').value
        self.output_cloud_topic = gp('output_cloud_topic').value

        self.lidar_height_m = float(gp('lidar_height_m').value)
        self.use_level_frame = bool(gp('use_level_frame').value)

        self.azimuth_bins = int(gp('azimuth_bins').value)
        self.radial_bin_size_m = float(gp('radial_bin_size_m').value)
        self.min_range_m = float(gp('min_range_m').value)
        self.max_range_m = float(gp('max_range_m').value)

        self.max_ground_slope_deg = float(gp('max_ground_slope_deg').value)
        self.ground_jump_tol = float(gp('ground_jump_tolerance_m').value)
        self.obstacle_height_thresh_m = float(gp('obstacle_height_thresh_m').value)
        self.max_obstacle_height_m = float(gp('max_obstacle_height_m').value)
        self.accumulate_scans = max(1, int(gp('accumulate_scans').value))
        self.log_period_sec = float(gp('log_period_sec').value)
        self.debug_removed_topic = gp('debug_removed_topic').value
        self.publish_removed = bool(gp('publish_removed').value)
        self.output_clearing_topic = gp('output_clearing_topic').value
        self.publish_clearing = bool(gp('publish_clearing').value)
        self.clearing_min_height_m = float(gp('clearing_min_height_m').value)

        self.tan_max_slope = self._safe_tan(self.max_ground_slope_deg)
        self.n_r = max(
            1,
            int(math.ceil(
                (self.max_range_m - self.min_range_m) / self.radial_bin_size_m
            ))
        )

        # =====================================================
        # 상태
        # =====================================================
        self.quat = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z
        self.odom_pos = np.zeros(3, dtype=np.float64)
        self.pitch_deg = 0.0
        self.last_log_time = 0.0

        # 스캔 누적 버퍼: (xyz_body, R_body2world, t_world)
        self.scan_buf = deque(maxlen=self.accumulate_scans)

        # =====================================================
        # Sub / Pub
        # =====================================================
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 20
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_cloud_topic, self.cloud_callback, 10
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.output_cloud_topic, 10
        )
        self.removed_pub = self.create_publisher(
            PointCloud2, self.debug_removed_topic, 10
        )
        self.clearing_pub = self.create_publisher(
            PointCloud2, self.output_clearing_topic, 10
        )

        self.get_logger().info('===================================================')
        self.get_logger().info('Slope Cloud Filter READY (local ground estimation)')
        self.get_logger().info(f'Input  : {self.input_cloud_topic}')
        self.get_logger().info(f'Output : {self.output_cloud_topic}')
        self.get_logger().info(
            f'Grid   : {self.azimuth_bins} az x {self.n_r} r '
            f'({self.radial_bin_size_m:.2f} m/bin, '
            f'{self.min_range_m:.1f}~{self.max_range_m:.1f} m)'
        )
        self.get_logger().info(
            f'Ground : max slope {self.max_ground_slope_deg:.1f} deg, '
            f'jump tol {self.ground_jump_tol:.2f} m'
        )
        self.get_logger().info(
            f'Obst   : height > {self.obstacle_height_thresh_m:.2f} m '
            f'above local ground (max {self.max_obstacle_height_m:.2f} m)'
        )
        self.get_logger().info(
            f'Level frame : {self.use_level_frame}  '
            f'(lidar height {self.lidar_height_m:.2f} m)'
        )
        self.get_logger().info('No pitch gating: filter is ALWAYS active.')
        self.get_logger().info('===================================================')

        # =====================================================
        # 런타임 파라미터 변경 지원
        # rqt_reconfigure / ros2 param set 으로 즉시 반영됨
        # (이게 없으면 __init__에서 읽은 값이 계속 쓰여서
        #  런타임 변경이 무시됨)
        # =====================================================
        self.add_on_set_parameters_callback(self.on_param_change)

    # =========================================================
    # tan() 안전 계산
    # 90도 이상이면 부호가 뒤집혀 지면 추정이 완전히 망가지므로
    # 85도에서 클램프
    # =========================================================
    @staticmethod
    def _safe_tan(deg):
        d = max(0.0, min(85.0, float(deg)))
        return math.tan(math.radians(d))

    # =========================================================
    # 파라미터 런타임 변경 콜백
    # =========================================================
    def on_param_change(self, params):
        regrid = False
        for p in params:
            n = p.name
            try:
                if n == 'obstacle_height_thresh_m':
                    self.obstacle_height_thresh_m = float(p.value)
                elif n == 'max_ground_slope_deg':
                    self.max_ground_slope_deg = float(p.value)
                    self.tan_max_slope = self._safe_tan(self.max_ground_slope_deg)
                elif n == 'ground_jump_tolerance_m':
                    self.ground_jump_tol = float(p.value)
                elif n == 'max_obstacle_height_m':
                    self.max_obstacle_height_m = float(p.value)
                elif n == 'lidar_height_m':
                    self.lidar_height_m = float(p.value)
                elif n == 'use_level_frame':
                    self.use_level_frame = bool(p.value)
                elif n == 'log_period_sec':
                    self.log_period_sec = float(p.value)
                elif n == 'publish_removed':
                    self.publish_removed = bool(p.value)
                elif n == 'accumulate_scans':
                    self.accumulate_scans = max(1, int(p.value))
                    self.scan_buf = deque(
                        self.scan_buf, maxlen=self.accumulate_scans
                    )
                elif n == 'publish_clearing':
                    self.publish_clearing = bool(p.value)
                elif n == 'clearing_min_height_m':
                    self.clearing_min_height_m = float(p.value)
                elif n == 'azimuth_bins':
                    self.azimuth_bins = int(p.value)
                elif n == 'radial_bin_size_m':
                    self.radial_bin_size_m = float(p.value)
                    regrid = True
                elif n == 'min_range_m':
                    self.min_range_m = float(p.value)
                    regrid = True
                elif n == 'max_range_m':
                    self.max_range_m = float(p.value)
                    regrid = True
            except (TypeError, ValueError) as e:
                self.get_logger().error(f'Bad value for {n}: {e}')
                return SetParametersResult(successful=False)

        if regrid:
            self.n_r = max(
                1,
                int(math.ceil(
                    (self.max_range_m - self.min_range_m) / self.radial_bin_size_m
                ))
            )

        self.get_logger().info(
            f'[PARAM UPDATED] slope={self.max_ground_slope_deg:.1f} deg | '
            f'obst_thresh={self.obstacle_height_thresh_m:.2f} m | '
            f'jump_tol={self.ground_jump_tol:.2f} m'
        )
        return SetParametersResult(successful=True)

    # =========================================================
    # Odometry: 자세만 저장 (게이팅에는 쓰지 않음)
    # =========================================================
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.quat = (q.w, q.x, q.y, q.z)
        p = msg.pose.pose.position
        self.odom_pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        sin_pitch = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch_deg = math.degrees(math.asin(sin_pitch))

    # =========================================================
    # quaternion -> body->world 회전행렬
    # =========================================================
    @staticmethod
    def quat_to_rotmat(w, x, y, z):
        n = math.sqrt(w * w + x * x + y * y + z * z)
        if n < 1e-9:
            return np.eye(3, dtype=np.float32)
        w, x, y, z = w / n, x / n, y / n, z / n
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float32,
        )

    # =========================================================
    # body -> 중력정렬(yaw 제거) 프레임 회전행렬
    # =========================================================
    def level_rotation(self):
        w, x, y, z = self.quat
        R = self.quat_to_rotmat(w, x, y, z)
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
        cy, sy = math.cos(yaw), math.sin(yaw)
        Rz_inv = np.array(
            [[cy, sy, 0.0], [-sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32
        )
        return Rz_inv @ R

    # =========================================================
    # PointCloud Callback
    # =========================================================
    def cloud_callback(self, msg):
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=['x', 'y', 'z'], skip_nans=True
            )
        except Exception as e:
            self.get_logger().error(f'PointCloud read error: {e}')
            return

        if pts is None or len(pts) == 0:
            return

        xyz = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        raw_count = int(xyz.shape[0])

        # -----------------------------------------------------
        # 스캔 누적
        #
        # 각 스캔을 그 시점의 odom 자세와 함께 저장해두고,
        # 현재 body frame으로 되돌려 합친다.
        #
        #   p_world = R_i @ p_i + t_i
        #   p_now   = R_now^T @ (p_world - t_now)
        #
        # 행벡터 형태에서는
        #   p_world_row = p_i_row @ R_i.T + t_i
        #   p_now_row   = (p_world_row - t_now) @ R_now
        # -----------------------------------------------------
        R_now = self.quat_to_rotmat(*self.quat).astype(np.float64)
        t_now = self.odom_pos.copy()
        self.scan_buf.append((xyz.astype(np.float64), R_now, t_now))

        if self.accumulate_scans > 1 and len(self.scan_buf) > 1:
            parts = []
            for p_i, R_i, t_i in self.scan_buf:
                p_world = p_i @ R_i.T + t_i
                parts.append((p_world - t_now) @ R_now)
            xyz = np.vstack(parts).astype(np.float32)

        accum_count = int(xyz.shape[0])

        # -----------------------------------------------------
        # 중력정렬 프레임으로 회전 (판정용 좌표)
        # 출력은 원본 BODY 좌표를 그대로 사용
        # -----------------------------------------------------
        if self.use_level_frame:
            R = self.level_rotation()
            pl = xyz @ R.T
        else:
            pl = xyz

        x = pl[:, 0]
        y = pl[:, 1]
        z = pl[:, 2]
        r = np.sqrt(x * x + y * y)

        in_range = (r >= self.min_range_m) & (r <= self.max_range_m)
        if not np.any(in_range):
            return

        xyz_in = xyz[in_range]
        x = x[in_range]
        y = y[in_range]
        z = z[in_range].astype(np.float64)
        r = r[in_range]

        # -----------------------------------------------------
        # 폴라 그리드 인덱스
        # -----------------------------------------------------
        az = np.arctan2(y, x)
        az_idx = ((az + math.pi) / (2.0 * math.pi) * self.azimuth_bins).astype(np.int32)
        np.clip(az_idx, 0, self.azimuth_bins - 1, out=az_idx)

        r_idx = ((r - self.min_range_m) / self.radial_bin_size_m).astype(np.int32)
        np.clip(r_idx, 0, self.n_r - 1, out=r_idx)

        flat = az_idx * self.n_r + r_idx

        # -----------------------------------------------------
        # 셀별 최저 z
        # -----------------------------------------------------
        minz = np.full(self.azimuth_bins * self.n_r, np.inf, dtype=np.float64)
        np.minimum.at(minz, flat, z)
        minz = minz.reshape(self.azimuth_bins, self.n_r)

        # -----------------------------------------------------
        # 거리 방향으로 지면 높이 전파
        #
        # 각 방위각 섹터마다 가까운 링부터 바깥으로 진행하며,
        # 이전 링 지면 대비 허용 경사 이내로 올라온 최저점만
        # 지면으로 인정. 그보다 급하게 튀면 지면이 아니라고 보고
        # 이전 지면 높이를 유지(보수적 추정).
        #
        # -> 완만한 오르막: 지면이 같이 따라 올라감 => 경사면 제거됨
        # -> 벽/급턱     : 지면이 따라가지 않음   => 장애물로 남음
        # -----------------------------------------------------
        # [수정] 빈 셀을 건너뛴 만큼 허용 상승량을 누적한다.
        #
        # 예전 코드는 링 하나당 고정 허용량만 줬기 때문에,
        # 포인트가 없는 링이 몇 개 연속되면 그 사이 실제 지면은
        # 계속 올라갔는데 허용량은 그대로여서 다음 관측이
        # 무조건 거부되고 지면 추정이 영구히 얼어붙었다.
        # (GroundZ가 시드값 근처에서 멈추던 원인)
        ground = np.empty_like(minz)
        base_step = self.tan_max_slope * self.radial_bin_size_m
        prev_g = np.full(self.azimuth_bins, -self.lidar_height_m, dtype=np.float64)
        gap = np.zeros(self.azimuth_bins, dtype=np.int32)  # 마지막 채택 이후 빈 링 수

        for i in range(self.n_r):
            m = minz[:, i]
            allow = base_step * (gap + 1) + self.ground_jump_tol
            accept = np.isfinite(m) & (m <= prev_g + allow)
            g = np.where(accept, m, prev_g)
            gap = np.where(accept, 0, gap + 1)
            ground[:, i] = g
            prev_g = g

        # -----------------------------------------------------
        # 지역 지면 대비 높이로 장애물 판정
        # -----------------------------------------------------
        g_pt = ground[az_idx, r_idx]
        height = z - g_pt

        keep = (
            (height > self.obstacle_height_thresh_m)
            & (height < self.max_obstacle_height_m)
        )

        filtered_xyz = xyz_in[keep]

        # -----------------------------------------------------
        # 로그
        # -----------------------------------------------------
        now = time.time()
        if now - self.last_log_time >= self.log_period_sec:
            total = int(len(xyz_in))
            kept = int(filtered_xyz.shape[0])
            finite_ground = ground[np.isfinite(ground)]
            g_min = float(np.min(finite_ground)) if finite_ground.size else float('nan')
            g_max = float(np.max(finite_ground)) if finite_ground.size else float('nan')
            n_cells = self.azimuth_bins * self.n_r
            filled = int(np.count_nonzero(np.isfinite(minz)))
            self.get_logger().info(
                f'[GROUND SEG] Pitch={self.pitch_deg:.2f} deg | '
                f'Scan={raw_count} x{len(self.scan_buf)} -> {accum_count} | '
                f'GroundZ={g_min:.2f}~{g_max:.2f} m | '
                f'HeightMax={float(np.max(height)) if height.size else 0.0:.2f} m | '
                f'Cells={filled}/{n_cells} ({100.0 * filled / n_cells:.0f}% filled, '
                f'{total / max(1, filled):.1f} pts/cell) | '
                f'Input={total} | Removed={total - kept} | Output={kept}'
            )
            self.last_log_time = now

        filtered_cloud = point_cloud2.create_cloud_xyz32(
            msg.header, filtered_xyz.tolist()
        )
        self.cloud_pub.publish(filtered_cloud)

        # -----------------------------------------------------
        # 디버그: 제거된 점 발행
        # RViz에서 이 토픽을 빨간색으로 띄우면
        # 필터가 실제로 무엇을 지우는지 눈으로 확인 가능
        # -----------------------------------------------------
        if self.publish_removed:
            removed_xyz = xyz_in[~keep]
            self.removed_pub.publish(
                point_cloud2.create_cloud_xyz32(
                    msg.header, removed_xyz.tolist()
                )
            )

        # -----------------------------------------------------
        # clearing 전용 클라우드
        #
        # 지면 점을 포함시켜서 지나온 길을 쓸어주되,
        # 지역 지면 기준 max_obstacle_height_m 위(나무 캐노피 등)는
        # 제외해서 2D raytracing 과잉 clearing을 막는다.
        # -----------------------------------------------------
        if self.publish_clearing:
            keep_clear = (
                (height < self.max_obstacle_height_m)
                & (height > self.clearing_min_height_m)
            )
            self.clearing_pub.publish(
                point_cloud2.create_cloud_xyz32(
                    msg.header, xyz_in[keep_clear].tolist()
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = SlopeCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
