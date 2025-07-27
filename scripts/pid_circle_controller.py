#!/usr/bin/env python3

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


class AdaptivePIDController:
    def __init__(self, kp, ki, kd, *, max_output=0.3, max_integral=0.1,
                 deadband=0.005, deriv_alpha=0.8):
        """適応型 PID コントローラ

        Args:
            kp, ki, kd: 基本 PID ゲイン
            max_output: 出力飽和値 (rad)
            max_integral: 積分項上限
            deadband: デッドバンド幅 (m)
            deriv_alpha: D 項低域フィルタ係数 (0〜1). 大きいほど平滑化
        """

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.max_output = max_output
        self.max_integral = max_integral
        self.deadband = deadband  # デッドバンド：小さなエラーは無視
        self.deriv_alpha = deriv_alpha

        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None
        self.error_history = []  # エラー履歴（適応ゲイン用）

        # D 項フィルタ用
        self.last_d_raw = 0.0

        # 直近の計算結果を保持（デバッグ用）
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0
        self.last_adaptive_factor = 1.0
        
    def update(self, error, current_time):
        if self.previous_time is None:
            self.previous_time = current_time
            return 0.0
            
        dt = current_time - self.previous_time
        if dt <= 0:
            return 0.0
        
        # デッドバンド処理：小さなエラーは無視
        if abs(error) < self.deadband:
            error = 0.0
            
        # エラー履歴を更新（適応制御用）
        self.error_history.append(abs(error))
        if len(self.error_history) > 10:  # 直近10回分を保持
            self.error_history.pop(0)
            
        # 適応ゲイン：エラーが大きい場合はゲインを緩めるが下限を 0.8 とする
        avg_error = np.mean(self.error_history) if self.error_history else 0
        adaptive_factor = 1.0
        if avg_error > 0.05:  # 大きなエラーの場合
            adaptive_factor = 0.8
        elif avg_error > 0.02:  # 中程度のエラーの場合
            adaptive_factor = 0.9
            
        # Proportional term
        p_term = self.kp * error * adaptive_factor
        
        # Integral term with windup protection
        if abs(error) < 0.1:  # エラーが小さい時のみ積分項を使用
            self.integral += error * dt
            # 積分項の制限
            self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
            i_term = self.ki * self.integral * adaptive_factor
        else:
            # 大きなエラーの時は積分項をリセット
            self.integral *= 0.9  # ゆっくりリセット
            i_term = 0.0
        
        # Derivative term with low-pass filtering
        raw_d = (error - self.previous_error) / dt
        filtered_d = self.deriv_alpha * self.last_d_raw + (1 - self.deriv_alpha) * raw_d
        self.last_d_raw = filtered_d

        d_term = self.kd * filtered_d * adaptive_factor
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # 段階的な出力制限
        if abs(output) > self.max_output:
            output = self.max_output if output > 0 else -self.max_output
            # 出力が飽和した場合は積分項をリセット
            self.integral *= 0.5
        
        # Update previous values
        self.previous_error = error
        self.previous_time = current_time
        
        # デバッグ用に直近値を保存
        self.last_p_term = p_term
        self.last_i_term = i_term if abs(error) < 0.1 else 0.0
        self.last_d_term = d_term
        self.last_adaptive_factor = adaptive_factor

        return output
        
    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None
        self.error_history = []
        self.last_d_raw = 0.0


class CircleFollowingController(Node):
    def __init__(self):
        super().__init__('circle_following_controller')
        
        # パブリッシャーの作成
        self.x_pub = self.create_publisher(Float64, '/joint_commands', 10)
        self.y_pub = self.create_publisher(Float64, '/joint_commands_y', 10)
        
        # サブスクライバーの作成（ボール位置情報 - Pose）
        self.pose_sub = self.create_subscription(
            Pose,
            '/model/ball/pose',
            self.pose_callback,
            10
        )
        
        # PID コントローラーの設定を高速応答向けに変更
        self.pid_x = AdaptivePIDController(
            kp=5.0, ki=0.1, kd=0.3,
            max_output=0.5, max_integral=0.3, deadband=0.003)

        self.pid_y = AdaptivePIDController(
            kp=5.0, ki=0.1, kd=0.3,
            max_output=0.5, max_integral=0.3, deadband=0.003)
        
        # 円軌道のパラメータ
        self.circle_radius = 0.050
        self.circle_center_x = 0.0
        self.circle_center_y = 0.0
        self.angular_velocity = 0.2  # rad/s（より遅く）
        
        # 現在のボール位置
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_z = 0.0
        self.ball_found = False
        
        # 制御フラグ
        self.control_enabled = False
        self.start_time = None
        
        # 制御品質監視
        self.control_performance = {
            'last_error': 0.0,
            'stable_count': 0,
            'oscillation_count': 0
        }
        
        self.get_logger().info('Adaptive Circle Following Controller started!')
        self.get_logger().info('Controls:')
        self.get_logger().info('  s: 円軌道制御を開始/停止')
        self.get_logger().info('  r: PIDコントローラーをリセット')
        self.get_logger().info('  b: ボールの位置をリセット')
        self.get_logger().info('  m: 手動制御モード（wasd）')
        self.get_logger().info('  t: PIDパラメータをチューニング')
        self.get_logger().info('  q: 終了')
        
        # 手動制御用の角度
        self.manual_mode = False
        self.x_angle = 0.0
        self.y_angle = 0.0
        self.angle_step = 0.05  # より小さなステップ
        
        # キーボード入力を別スレッドで処理
        self.input_thread = threading.Thread(target=self.keyboard_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

        # デフォルトでDEBUGレベルに
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # 制御ループ
        self.timer = self.create_timer(0.01, self.control_loop)  # 20Hz（より低い周波数）
        
        # ボールリセット用のSDF定義
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

    def pose_callback(self, msg):
        """/model/ball/pose から Pose を受信して位置を更新"""
        # 位置を取得
        self.ball_x = msg.position.x
        self.ball_y = msg.position.y
        self.ball_z = msg.position.z
        self.ball_found = True

        # デバッグ用：3 秒ごとに位置をログ
        if not hasattr(self, '_last_ball_log_time') or time.time() - self._last_ball_log_time > 3.0:
            status = "板上" if self.ball_z > 0.25 else "落下"
            self.get_logger().info(
                f'ボール位置: ({self.ball_x:.3f}, {self.ball_y:.3f}, {self.ball_z:.3f}) [{status}]'
            )
            self._last_ball_log_time = time.time()

    def generate_circle_reference(self, t):
        """円軌道の参照位置を生成（適応的な半径）"""
        # ボールが落下している場合は小さな円から始める
        if self.ball_z < 0.25:
            effective_radius = min(self.circle_radius, 0.03)  # 小さな円から開始
        else:
            effective_radius = self.circle_radius
            
        # ref_x = self.circle_center_x + effective_radius * math.cos(self.angular_velocity * t)
        # ref_y = self.circle_center_y + effective_radius * math.sin(self.angular_velocity * t)
        ref_x = 0.000
        ref_y = 0.050
        return ref_x, ref_y

    def control_loop(self):
        """改善されたメイン制御ループ"""
        if self.manual_mode:
            # 手動制御モード
            x_msg = Float64()
            y_msg = Float64()
            x_msg.data = self.x_angle
            y_msg.data = self.y_angle
            self.x_pub.publish(x_msg)
            self.y_pub.publish(y_msg)
            return
            
        if not self.control_enabled:
            # 制御無効時は水平を保つ
            x_msg = Float64()
            y_msg = Float64()
            x_msg.data = 0.0
            y_msg.data = 0.0
            self.x_pub.publish(x_msg)
            self.y_pub.publish(y_msg)
            return
            
        if not self.ball_found:
            return
            
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time()
        t = current_time - self.start_time
        
        # 参照軌道の生成
        ref_x, ref_y = self.generate_circle_reference(t)
        
        # エラーの計算
        error_x = ref_x - self.ball_x
        error_y = ref_y - self.ball_y
        
        # 総エラー
        total_error = math.sqrt(error_x**2 + error_y**2)
        
        # 制御品質の監視
        self.monitor_control_performance(total_error)
        
        # PID制御
        control_x = self.pid_x.update(error_x, current_time)
        control_y = self.pid_y.update(error_y, current_time)

        # 詳細デバッグログ（毎ループ）
        self.get_logger().debug(
            f"t={t:.2f}s | ErrX={error_x:.4f} (P={self.pid_x.last_p_term:.4f} I={self.pid_x.last_i_term:.4f} D={self.pid_x.last_d_term:.4f}) "
            f"ErrY={error_y:.4f} (P={self.pid_y.last_p_term:.4f} I={self.pid_y.last_i_term:.4f} D={self.pid_y.last_d_term:.4f})"
        )
        
        # 制御出力を送信（符号を確認・調整）
        x_msg = Float64()
        y_msg = Float64()
        # プラットフォームの回転方向とボール移動の関係から、X/Y ともに符号を反転
        x_msg.data = -control_y  # X軸制御（符号反転）
        y_msg.data = control_x  # Y軸制御（符号反転）
        self.x_pub.publish(x_msg)
        self.y_pub.publish(y_msg)
        
        # デバッグ情報（10秒毎に出力）
        if int(t) % 10 == 0 and int(t * 20) % 200 == 0:
            self.get_logger().info(
                f'制御状態: Ball({self.ball_x:.3f}, {self.ball_y:.3f}, {self.ball_z:.3f}) '
                f'Ref({ref_x:.3f}, {ref_y:.3f}) Error({error_x:.3f}, {error_y:.3f}) '
                f'Control({control_x:.3f}, {control_y:.3f}) Performance: {self.control_performance["stable_count"]}/10'
            )

    def monitor_control_performance(self, total_error):
        """制御性能を監視し、必要に応じてパラメータを調整"""
        if total_error < 0.02:  # 安定状態
            self.control_performance['stable_count'] += 1
        else:
            self.control_performance['stable_count'] = 0
            
        # 振動検出
        if abs(total_error - self.control_performance['last_error']) > 0.05:
            self.control_performance['oscillation_count'] += 1
        else:
            self.control_performance['oscillation_count'] = max(0, self.control_performance['oscillation_count'] - 1)
            
        # 自動調整
        if self.control_performance['oscillation_count'] > 5:
            # 振動が多い場合はゲインを下げる
            self.pid_x.kp *= 0.9
            self.pid_y.kp *= 0.9
            self.control_performance['oscillation_count'] = 0
            self.get_logger().info('振動検出: PIDゲインを自動調整しました')
            
        self.control_performance['last_error'] = total_error

    def tune_pid_parameters(self):
        """PIDパラメータの手動調整"""
        current_kp = self.pid_x.kp
        new_kp = current_kp * 1.1 if current_kp < 1.0 else current_kp * 0.9
        
        self.pid_x.kp = new_kp
        self.pid_y.kp = new_kp
        
        self.get_logger().info(f'PIDゲイン調整: Kp = {new_kp:.3f}')

    def remove_ball(self):
        """ボールモデルを削除"""
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
        """ボールモデルを作成"""
        try:
            # SDFの改行と特殊文字をエスケープ
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
        """ボールを削除→再作成して位置と速度を完全にリセット"""
        try:
            # ステップ1: ボールを削除
            if not self.remove_ball():
                self.get_logger().warning('ボールの削除に失敗しました')
                return
            
            time.sleep(0.3)  # 削除を確実にするための待機
            
            # ステップ2: ボールを再作成
            if not self.create_ball():
                self.get_logger().warning('ボールの再作成に失敗しました')
                return
            
            time.sleep(0.3)  # 作成を確実にするための待機
            
            # ステップ3: 制御状態をリセット
            self.ball_found = False
            self.ball_x = 0.0
            self.ball_y = 0.0
            self.ball_z = 0.32
            self.pid_x.reset()
            self.pid_y.reset()
            self.start_time = None
            self.control_performance = {'last_error': 0.0, 'stable_count': 0, 'oscillation_count': 0}
            
            self.get_logger().info('ボールの位置と制御状態を完全にリセットしました')
            
        except Exception as e:
            self.get_logger().error(f'ボールリセットでエラー: {e}')

    def keyboard_input_loop(self):
        """キーボード入力を処理するループ"""
        # 端末の設定を変更
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                key = sys.stdin.read(1)
                
                if key == 's':
                    self.control_enabled = not self.control_enabled
                    if self.control_enabled:
                        if not self.ball_found:
                            self.get_logger().warn('ボールの位置が見つかりません。まず位置を確認してください。')
                            self.control_enabled = False
                        else:
                            self.start_time = None
                            self.pid_x.reset()
                            self.pid_y.reset()
                            self.control_performance = {'last_error': 0.0, 'stable_count': 0, 'oscillation_count': 0}
                            self.get_logger().info('改善された円軌道制御を開始しました')
                    else:
                        self.get_logger().info('円軌道制御を停止しました')
                        
                elif key == 'r':
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.start_time = None
                    self.control_performance = {'last_error': 0.0, 'stable_count': 0, 'oscillation_count': 0}
                    self.get_logger().info('PIDコントローラーをリセットしました')
                    
                elif key == 'b':
                    # ボールリセット機能を追加
                    self.reset_ball_position()
                    
                elif key == 't':
                    # PIDパラメータ調整
                    self.tune_pid_parameters()
                    
                elif key == 'm':
                    self.manual_mode = not self.manual_mode
                    if self.manual_mode:
                        self.control_enabled = False
                        self.get_logger().info('手動制御モードに切り替えました（wasd）')
                    else:
                        self.get_logger().info('自動制御モードに切り替えました')
                        
                elif key == 'w' and self.manual_mode:
                    self.x_angle = max(-0.3, self.x_angle - self.angle_step)
                    self.get_logger().info(f'X軸角度: {math.degrees(self.x_angle):.1f}°')
                    
                elif key == 's' and self.manual_mode:
                    self.x_angle = min(0.3, self.x_angle + self.angle_step)
                    self.get_logger().info(f'X軸角度: {math.degrees(self.x_angle):.1f}°')
                    
                elif key == 'a' and self.manual_mode:
                    self.y_angle = max(-0.3, self.y_angle - self.angle_step)
                    self.get_logger().info(f'Y軸角度: {math.degrees(self.y_angle):.1f}°')
                    
                elif key == 'd' and self.manual_mode:
                    self.y_angle = min(0.3, self.y_angle + self.angle_step)
                    self.get_logger().info(f'Y軸角度: {math.degrees(self.y_angle):.1f}°')
                    
                elif key == 'q':
                    self.get_logger().info('終了します...')
                    rclpy.shutdown()
                    break
                    
        finally:
            # 端末の設定を復元
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    
    controller = CircleFollowingController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 