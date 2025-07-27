#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import threading
import math
import subprocess
import time


class PlatformController(Node):
    def __init__(self):
        super().__init__('platform_controller')
        
        # パブリッシャーの作成
        self.x_pub = self.create_publisher(Float64, '/joint_commands', 10)
        self.y_pub = self.create_publisher(Float64, '/joint_commands_y', 10)
        
        # 初期角度
        self.x_angle = 0.0
        self.y_angle = 0.0
        
        # ボールのエンティティID（起動時に取得）
        self.ball_entity_id = None
        
        # ボールのSDF定義
        self.ball_sdf = '''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="ball">
    <pose>0 0 0.32 0 0 0</pose>
    <link name="ball_link">
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.000016</ixx>
          <iyy>0.000016</iyy>
          <izz>0.000016</izz>
        </inertia>
      </inertial>
      <collision name="ball_collision">
        <geometry>
          <sphere>
            <radius>0.01</radius>
          </sphere>
        </geometry>
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
        
        # 角度の増分（ラジアン）
        self.angle_step = 0.1
        
        self.get_logger().info('Platform Controller started!')
        self.get_logger().info('Controls:')
        self.get_logger().info('  w/s: X軸の回転')
        self.get_logger().info('  a/d: Y軸の回転')
        self.get_logger().info('  r: 角度をリセット')
        self.get_logger().info('  b: ボールの位置をリセット')
        self.get_logger().info('  q: 終了')
        
        # Gazeboの起動を待ってからボールのエンティティIDを取得
        self.get_logger().info('Gazeboの起動を待機中...')
        self.get_ball_entity_id_with_retry()
        
        # キーボード入力を別スレッドで処理
        self.input_thread = threading.Thread(target=self.keyboard_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        # 定期的にコマンドを送信
        self.timer = self.create_timer(0.1, self.publish_commands)

    def get_ball_entity_id_with_retry(self):
        """リトライ機能付きでボールのエンティティIDを取得"""
        max_retries = 10
        retry_interval = 2  # 秒
        
        for attempt in range(max_retries):
            if attempt > 0:
                self.get_logger().info(f'Gazebo起動待機中... ({attempt + 1}/{max_retries})')
                time.sleep(retry_interval)
                
            try:
                # まずGazeboが応答するかをテスト
                test_cmd = ['gz', 'model', '--list']
                test_result = subprocess.run(test_cmd, capture_output=True, text=True, timeout=10)
                
                if test_result.returncode != 0:
                    continue
                    
                # ボールのポーズを取得
                cmd = ['gz', 'model', '-m', 'ball', '--pose']
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0:
                    # 出力からエンティティIDを抽出
                    lines = result.stdout.split('\n')
                    for line in lines:
                        if 'Model: [' in line:
                            # "Model: [19]" から19を抽出
                            entity_id = line.split('[')[1].split(']')[0]
                            self.ball_entity_id = entity_id
                            self.get_logger().info(f'ボールのエンティティID: {entity_id}')
                            return
                            
            except Exception as e:
                self.get_logger().info(f'エンティティID取得試行 {attempt + 1}: {e}')
                
        self.get_logger().warning('ボールのエンティティID取得に失敗しました。手動でリセットを試してください。')

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
            self.get_logger().info('ボールを削除中...')
            if not self.remove_ball():
                self.get_logger().warning('ボールの削除に失敗しました')
                return
            
            time.sleep(0.2)  # 削除を確実にするための待機
            
            # ステップ2: ボールを再作成
            self.get_logger().info('ボールを再作成中...')
            if not self.create_ball():
                self.get_logger().warning('ボールの再作成に失敗しました')
                return
            
            time.sleep(0.2)  # 作成を確実にするための待機
            
            # ステップ3: 新しいエンティティIDを取得
            self.ball_entity_id = None
            self.get_ball_entity_id_with_retry()
            
            self.get_logger().info('ボールの位置と速度を完全にリセットしました')
            
        except Exception as e:
            self.get_logger().warning(f'ボールリセットでエラー: {e}')

    def keyboard_input_loop(self):
        """キーボード入力を処理するループ"""
        # 端末の設定を変更
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                key = sys.stdin.read(1)
                
                if key == 'w':
                    self.x_angle = max(-0.785, self.x_angle - self.angle_step)
                    self.get_logger().info(f'X軸角度: {self.x_angle:.2f} rad ({math.degrees(self.x_angle):.1f}°)')
                elif key == 's':
                    self.x_angle = min(0.785, self.x_angle + self.angle_step)
                    self.get_logger().info(f'X軸角度: {self.x_angle:.2f} rad ({math.degrees(self.x_angle):.1f}°)')
                elif key == 'a':
                    self.y_angle = max(-0.785, self.y_angle - self.angle_step)
                    self.get_logger().info(f'Y軸角度: {self.y_angle:.2f} rad ({math.degrees(self.y_angle):.1f}°)')
                elif key == 'd':
                    self.y_angle = min(0.785, self.y_angle + self.angle_step)
                    self.get_logger().info(f'Y軸角度: {self.y_angle:.2f} rad ({math.degrees(self.y_angle):.1f}°)')
                elif key == 'r':
                    self.x_angle = 0.0
                    self.y_angle = 0.0
                    self.get_logger().info('角度をリセットしました')
                elif key == 'b':
                    self.reset_ball_position()
                elif key == 'q':
                    self.get_logger().info('終了します...')
                    rclpy.shutdown()
                    break
                    
        finally:
            # 端末の設定を復元
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def publish_commands(self):
        """ジョイントコマンドを送信"""
        x_msg = Float64()
        y_msg = Float64()
        
        x_msg.data = self.x_angle
        y_msg.data = self.y_angle
        
        self.x_pub.publish(x_msg)
        self.y_pub.publish(y_msg)


def main(args=None):
    rclpy.init(args=args)
    
    controller = PlatformController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 