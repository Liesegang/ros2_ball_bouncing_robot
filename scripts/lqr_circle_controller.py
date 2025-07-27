#!/usr/bin/env python3

"""
LQR 版 円軌道フォロワコントローラ
================================

pid_circle_controller.py をベースに、適応 PID ではなく LQR（線形二次レギュレータ）を
用いて制御を行う ROS2 ノードです。

1. ダブルインテグレータモデル x¨ = u を想定し、状態 x = [位置誤差, 速度誤差] に対し
   LQR で最適フィードバックゲイン K を計算（SciPy がある場合）
2. SciPy が無い環境では手動調整済みの既定ゲイン K = [20, 5] を使用
3. 構成・通信トピック・キーボード操作は元スクリプトと同等
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import sys
import termios
import tty
import threading
import math
import time
import numpy as np
import subprocess

from scipy.linalg import solve_continuous_are


class LQRController:
    """シンプルな 1DoF LQR コントローラ (位置+速度 → 角度)"""

    def __init__(self, *, max_output: float = 0.3):
        self.max_output = max_output

        # 状態フィードバックゲイン K を決定
        # 連続時間ダブルインテグレータモデル
        A = np.array([[0, 1],
                        [0, 0]])
        B = np.array([[0],
                        [1]])
        # 重み行列（適宜調整）
        Q = np.diag([50.0, 1.0])
        R = np.array([[1.0]])
        # Riccati 方程式を解く
        P = solve_continuous_are(A, B, Q, R)
        # 最適ゲイン K = R^-1 B^T P
        self.K = np.linalg.inv(R) @ B.T @ P
        self.K = self.K.flatten()

        # 内部状態
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error: float, now: float) -> float:
        """位置誤差から制御入力を計算"""
        if self.prev_time is None:
            self.prev_time = now
            self.prev_error = error
            return 0.0

        dt = now - self.prev_time
        if dt <= 1e-6:
            return 0.0

        error_dot = (error - self.prev_error) / dt
        state = np.array([error, error_dot])
        # u = -K x
        output = -float(self.K @ state)
        # 飽和
        output = max(-self.max_output, min(self.max_output, output))

        self.prev_error = error
        self.prev_time = now
        return output


class CircleFollowingControllerLQR(Node):
    def __init__(self):
        super().__init__('circle_following_controller_lqr')

        # パブリッシャー
        self.x_pub = self.create_publisher(Float64, '/joint_commands', 10)
        self.y_pub = self.create_publisher(Float64, '/joint_commands_y', 10)

        # サブスクライバー（ボール位置）
        self.pose_sub = self.create_subscription(
            Pose,
            '/model/ball/pose',
            self.pose_callback,
            10
        )

        # LQR コントローラ
        self.lqr_x = LQRController(max_output=0.3)
        self.lqr_y = LQRController(max_output=0.3)

        # 円軌道パラメータ
        self.circle_radius = 0.050
        self.circle_center_x = 0.0
        self.circle_center_y = 0.0
        self.angular_velocity = 0.2  # rad/s

        # ポイントトゥポイント切替シナリオ
        #   (0.05,0.05) ↔ (-0.05,-0.05) を 1 s ごとに行き来
        self.toggle_mode = False  # false = circle, true = toggle points
        self.point_a = (0.05, 0.05)
        self.point_b = (-0.05, -0.05)

        # ボール状態
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_z = 0.0
        self.ball_found = False

        # 制御フラグ
        self.control_enabled = False
        self.start_time = None

        # 手動制御
        self.manual_mode = False
        self.x_angle = 0.0
        self.y_angle = 0.0
        self.angle_step = 0.05

        # ---------------- 速度リミット設定 ----------------
        # プラットフォーム角度の変化速度上限 [rad/s]
        self.max_angle_rate = 1.0  # 適宜調整
        self._prev_pub_x = 0.0
        self._prev_pub_y = 0.0
        self._prev_pub_time = time.time()

        # キー入力スレッド
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()

        # タイマ（20Hz）
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('LQR Circle Following Controller started!')

        # ログレベルを DEBUG に
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # SDF （ボールモデル）を同コピー
        self.ball_sdf = '''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="ball">
    <pose>0 0 0.32 0 0 0</pose>
    <link name="ball_link">
      <inertial>
        <mass>0.005</mass>
        <inertia>
          <ixx>0.000008</ixx>
          <iyy>0.000008</iyy>
          <izz>0.000008</izz>
        </inertia>
      </inertial>
      <collision name="ball_collision">
        <geometry>
          <sphere>
            <radius>0.01</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.3</restitution_coefficient>
            <threshold>0.01</threshold>
          </bounce>
        </surface>
      </collision>
      <visual name="ball_visual">
        <geometry>
          <sphere>
            <radius>0.01</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

    # -------------------------- コールバック --------------------------
    def pose_callback(self, msg: Pose):
        self.ball_x = msg.position.x
        self.ball_y = msg.position.y
        self.ball_z = msg.position.z
        self.ball_found = True

    # -------------------------- 制御 --------------------------
    def generate_reference(self, t: float):
        """参照位置生成（モードに応じて円 or トグル）"""

        # --- ポイントトグルモード ---
        if self.toggle_mode:
            # 4 秒ごとに A / B を切り替え
            if (int(t) // 4) % 2 == 0:
                return self.point_a
            else:
                return self.point_b

        # --- 円軌道モード ---
        if self.ball_z < 0.25:
            effective_radius = min(self.circle_radius, 0.03)
        else:
            effective_radius = self.circle_radius
        ref_x = self.circle_center_x + effective_radius * math.cos(self.angular_velocity * t)
        ref_y = self.circle_center_y + effective_radius * math.sin(self.angular_velocity * t)
        return ref_x, ref_y

    def control_loop(self):
        # 手動モード
        if self.manual_mode:
            self.publish_angles(self.x_angle, self.y_angle)
            return

        # 自動制御無効
        if not self.control_enabled:
            self.publish_angles(0.0, 0.0)
            return

        if not self.ball_found:
            return

        if self.start_time is None:
            self.start_time = time.time()

        now = time.time()
        t = now - self.start_time

        # 参照位置（モードに応じて生成）
        ref_x, ref_y = self.generate_reference(t)

        # 誤差
        error_x = ref_x - self.ball_x
        error_y = ref_y - self.ball_y

        # LQR 制御入力
        ctrl_x = self.lqr_x.update(error_x, now)
        ctrl_y = self.lqr_y.update(error_y, now)

        # デバッグ
        self.get_logger().debug(
            f"t={t:.2f}s | ErrX={error_x:.4f} ErrY={error_y:.4f} CtrlX={ctrl_x:.4f} CtrlY={ctrl_y:.4f}")

        # 出力（符号調整）
        self.publish_angles(ctrl_y, -ctrl_x)

    def publish_angles(self, x_angle: float, y_angle: float):
        """角度コマンドを速度リミット付きでパブリッシュ"""

        now = time.time()
        dt = now - self._prev_pub_time if now > self._prev_pub_time else 0.0
        # 最大角度変化
        max_delta = self.max_angle_rate * dt if dt > 0 else 0.0

        # X 軸
        delta_x = x_angle - self._prev_pub_x
        if abs(delta_x) > max_delta:
            x_angle = self._prev_pub_x + math.copysign(max_delta, delta_x)

        # Y 軸
        delta_y = y_angle - self._prev_pub_y
        if abs(delta_y) > max_delta:
            y_angle = self._prev_pub_y + math.copysign(max_delta, delta_y)

        # メッセージ送信
        x_msg = Float64()
        y_msg = Float64()
        x_msg.data = x_angle
        y_msg.data = y_angle
        self.x_pub.publish(x_msg)
        self.y_pub.publish(y_msg)

        # 状態更新
        self._prev_pub_x = x_angle
        self._prev_pub_y = y_angle
        self._prev_pub_time = now

    # -------------------------- キーボード入力 --------------------------
    def keyboard_input_loop(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)

                if key == 's':
                    self.control_enabled = not self.control_enabled
                    if self.control_enabled:
                        if not self.ball_found:
                            self.get_logger().warn('ボール位置が見つかりません')
                            self.control_enabled = False
                        else:
                            self.start_time = None
                            self.lqr_x.reset()
                            self.lqr_y.reset()
                            self.get_logger().info('円軌道制御（LQR）を開始')
                    else:
                        self.get_logger().info('円軌道制御を停止')

                elif key == 'p':
                    # 参照モード切替
                    self.toggle_mode = not self.toggle_mode
                    mode_str = 'ポイントトグル' if self.toggle_mode else '円軌道'
                    self.get_logger().info(f'参照モードを {mode_str} に切替えました')

                elif key == 'm':
                    self.manual_mode = not self.manual_mode
                    if self.manual_mode:
                        self.control_enabled = False
                        self.get_logger().info('手動制御モード (wasd)')
                    else:
                        self.get_logger().info('自動制御モード')

                elif key == 'w' and self.manual_mode:
                    self.x_angle = max(-0.3, self.x_angle - self.angle_step)
                elif key == 's' and self.manual_mode:
                    self.x_angle = min(0.3, self.x_angle + self.angle_step)
                elif key == 'a' and self.manual_mode:
                    self.y_angle = max(-0.3, self.y_angle - self.angle_step)
                elif key == 'd' and self.manual_mode:
                    self.y_angle = min(0.3, self.y_angle + self.angle_step)

                elif key == 'q':
                    self.get_logger().info('終了します...')
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # -------------------------- 便利関数 --------------------------
    def remove_ball(self):
        try:
            cmd = [
                'gz', 'service', '-s', '/world/ball_world/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', 'name: "ball", type: MODEL'
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            return result.returncode == 0 and 'data: true' in result.stdout
        except Exception as e:
            self.get_logger().warning(f'ボール削除でエラー: {e}')
            return False

    def create_ball(self):
        try:
            sdf_escaped = self.ball_sdf.replace('\n', '').replace('"', '\\"')
            cmd = [
                'gz', 'service', '-s', '/world/ball_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', f'sdf: "{sdf_escaped}"'
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            return result.returncode == 0 and 'data: true' in result.stdout
        except Exception as e:
            self.get_logger().warning(f'ボール作成でエラー: {e}')
            return False

    def reset_ball_position(self):
        if not self.remove_ball():
            self.get_logger().warning('ボール削除失敗')
            return
        time.sleep(0.3)
        if not self.create_ball():
            self.get_logger().warning('ボール再作成失敗')
            return
        time.sleep(0.3)
        self.ball_found = False
        self.ball_x = self.ball_y = 0.0
        self.ball_z = 0.32
        self.lqr_x.reset()
        self.lqr_y.reset()
        self.start_time = None
        self.get_logger().info('ボール位置をリセットしました')


def main(args=None):
    rclpy.init(args=args)
    node = CircleFollowingControllerLQR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 