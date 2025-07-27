# Ball Gazebo Simulation with Tilting Platform

このパッケージは、Gazeboで傾斜可能な板の上にボールを配置したシミュレーションです。手動制御とPIDコントローラーによる自動円軌道追従の両方をサポートします。

## 必要な依存関係

- ROS 2 (Humble以降)
- Gazebo Garden以降
- ros_gz_sim
- ros_gz_bridge
- tf2_msgs
- python3-numpy

## ビルド方法

```bash
cd ~/ros2_ws
colcon build --packages-select ball_gazebo_sim
source install/setup.bash
```

## 実行方法

### 1. シミュレーションを起動

```bash
ros2 launch ball_gazebo_sim ball_simulation.launch.py
```

### 2a. 手動制御（別ターミナルで実行）

```bash
ros2 run ball_gazebo_sim platform_controller.py
```

**手動制御キー:**
- `w`/`s`: X軸の回転（前後に傾ける）
- `a`/`d`: Y軸の回転（左右に傾ける）
- `r`: 角度をリセット（水平に戻す）
- `b`: ボールの位置をリセット（削除→再作成）
- `q`: 制御プログラムを終了

### 2b. PID円軌道制御（別ターミナルで実行）

```bash
ros2 run ball_gazebo_sim pid_circle_controller.py
```

**PID制御キー:**
- `s`: 円軌道制御を開始/停止
- `r`: PIDコントローラーをリセット
- `m`: 手動制御モード（wasd）に切り替え
- `q`: 制御プログラムを終了

## 機能

### 手動制御モード
ユーザーがキーボードで板を直接制御し、ボールの動きを観察できます。ボールリセット機能により、何度でも実験を繰り返せます。

### PID円軌道追従制御
- **目標軌道**: 直径150mm（半径75mm）の円軌道
- **制御方式**: X軸・Y軸独立のPIDコントローラー
- **制御周波数**: 50Hz
- **ボール位置フィードバック**: Gazeboのpose情報をROS2ブリッジ経由で取得

#### PIDパラメータ（調整可能）
- Kp（比例ゲイン）: 1.5
- Ki（積分ゲイン）: 0.1  
- Kd（微分ゲイン）: 0.3
- 最大出力: ±45度（±0.785ラジアン）

## パッケージ構成

- `worlds/ball.sdf`: 傾斜板とボールのあるGazebo世界ファイル
- `launch/ball_simulation.launch.py`: シミュレーション起動用のlaunchファイル
- `config/bridge_config.yaml`: ROS2-Gazeboブリッジ設定
- `scripts/platform_controller.py`: 手動制御用のPythonスクリプト
- `scripts/pid_circle_controller.py`: PID円軌道追従制御用のPythonスクリプト

## 世界の内容

- 地面（100x100mの平面）
- 傾斜可能な板（200mm x 200mm、高さ300mm）
  - X軸とY軸に±45度まで傾斜可能
  - 中心が固定されている
  - **支柱は透明で非表示**
- 赤い球体（直径20mm、質量0.01kg）
- 環境光

## 仕様

- **板のサイズ**: 200mm x 200mm
- **板の高さ**: 300mm
- **ボールの直径**: 20mm
- **傾斜範囲**: ±45度（±0.785ラジアン）
- **円軌道直径**: 150mm（半径75mm）
- **制御周期**: 50Hz
- **角速度**: 0.5 rad/s（約30秒で1周）

## PID制御の仕組み

1. **参照軌道生成**: 時間tに応じて円軌道上の目標位置(x_ref, y_ref)を計算
2. **位置フィードバック**: Gazeboからボールの現在位置(x, y)を取得
3. **エラー計算**: error_x = x_ref - x, error_y = y_ref - y
4. **PID制御**: 各軸で独立してPID制御を実行
5. **板傾斜**: 制御出力に応じて板を傾けてボールを目標位置に誘導

ボールは物理シミュレーションにより、板の傾きに応じて自然に転がり、PIDコントローラーが継続的に軌道修正を行います。 