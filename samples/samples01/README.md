# サンプル集01 ロボットのサーボモータを動かす

- [サンプル集01 ロボットのサーボモータを動かす](#サンプル集01-ロボットのサーボモータを動かす)
  - [サンプルのビルド](#サンプルのビルド)
  - [通信設定](#通信設定)
  - [サーボモータのトルクをON/OFFする](#サーボモータのトルクをonoffする)
    - [解説](#解説)
  - [サーボモータの現在角度を読み取る](#サーボモータの現在角度を読み取る)
    - [解説](#解説-1)
  - [サーボモータの目標角度を書き込む](#サーボモータの目標角度を書き込む)
    - [解説](#解説-2)
  - [スレッドでサーボモータの角度を読み書きする](#スレッドでサーボモータの角度を読み書きする)
    - [解説](#解説-3)
  - [サーボモータの速度、電流、入力電圧、温度を読み取る](#サーボモータの速度電流入力電圧温度を読み取る)
    - [解説](#解説-4)
  - [サーボモータの目標速度を書き込む](#サーボモータの目標速度を書き込む)
  - [サーボモータの目標電流を書き込む](#サーボモータの目標電流を書き込む)

## サンプルのビルド

次のコマンドを実行して、サンプル集をビルドします。

```sh
$ ./build_samples.bash
```

ビルドに成功すると`samples01/bin/`ディレクトリに実行ファイルが生成されます。

## 通信設定

ロボットとの通信遅延を最小にするため次のコマンドを実行します。

```sh
$ sudo chmod a+rw /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
$ sudo echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Sciurus17を使用する場合は、
[Sciurus17入門ガイド](https://rt-net.jp/products/sciurus17/)に記載されている手順で
Sciurus17制御基板の通信タイムアウト機能を解除してください。

サンプル内にスリープ処理によって通信タイムアウト機能が働き、サーボが脱力します。

## サーボモータのトルクをON/OFFする

次のコマンドを実行します。
サーボモータのトルクがON / OFFします。

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_onoff
# Sciurus17の場合
$ ./s17_onoff
```

実行結果（CRANE-X7の場合）

```sh
CRANE-X7のトルクをON/OFFするサンプルです.
CRANE-X7(ポート:/dev/ttyUSB0 ボーレート:3000000)に接続します.
コンフィグファイル:../config/crane-x7.yamlを読み込みます.
Config file '../config/crane-x7.yaml' loaded.
arm
        joint1, id:2, mode:3
        joint2, id:3, mode:3
        joint3, id:4, mode:3
        joint4, id:5, mode:3
        joint5, id:6, mode:3
        joint6, id:7, mode:3
        joint7, id:8, mode:3
hand
        joint_hand, id:9, mode:3
ジョイントグループ:armのトルクをONにします.
10秒間スリープします.
サーボモータの制御モードが位置制御モードのとき、ロボットに触れるとトルクがONになっていることがわかります.
ジョイントグループ:armのトルクをOFFにします.
CRANE-X7との接続を解除します.
```

### 解説

RTマニピュレータC++ライブラリを使用する場合は`rt_manipulators_cpp/hardware.hpp`をincludeします。

```cpp
#include "rt_manipulators_cpp/hardware.hpp"
```

ロボットと通信するために、`rt_manipulators_cpp::Hardware(port_name)`クラスのインスタンスを生成します。
インスタンスの引数には通信ポート名を入力します。

```cpp
rt_manipulators_cpp::Hardware hardware("/dev/ttyUSB0");
```

ロボットと接続するために、`Hardware.connect(baudrate)`を実行します。
引数には通信ボーレート（bps）を入力します。

```cpp
hardware.connect(3000000);
```

ロボットのサーボモータ構成を読み込むために、`Hardware.load_config_file(file_path)`を実行します。
引数にはコンフィグファイルのパスを入力します。


```cpp
hardware.load_config_file("../config/crane-x7.yaml");
```

コンフィグファイルはYAMLフォーマットで次のように作成します。

```yaml
joint_groups:
  ジョイントグループ名(1):
    joints:
      - ジョイント名(1)
      - ジョイント名(2)
      - ジョイント名(3)
  ジョイントグループ名(2):
    joints:
      - ジョイント名(4)
      - ジョイント名(5)

ジョイント名(1): { id : 0, operating_mode: 3 }
ジョイント名(2): { id : 1, operating_mode: 3 }
ジョイント名(3): { id : 2, operating_mode: 3 }
ジョイント名(4): { id : 3, operating_mode: 3 }
ジョイント名(5): { id : 4, operating_mode: 3 }
```

トルクをONするために`Hardware.torque_on(group_name)`を実行します。
引数にはジョイントグループ名を入力します。
ジョイントグループのすべてのモータのトルクがONされます。

```cpp
hardware.torque_on("arm");
```

トルクをOFFするために`Hardware.torque_off(group_name)`を実行します。
引数にはジョイントグループ名を入力します。
ジョイントグループのすべてのモータのトルクがFFされます。

```cpp
hardware.torque_off("arm");
```

ロボットとの接続を解除するために`Hardware.disconnect()`を実行します。

```cpp
hardware.disconnect();
```

## サーボモータの現在角度を読み取る

次のコマンドを実行します。
サーボモータの現在角度がターミナルに表示されます。

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_read_position
# Sciurus17の場合
$ ./s17_read_position
```

実行結果（CRANE-X7の場合）

```sh
...
ID:2のサーボ角度は1.491029radです.
joint_handのサーボ角度は1.259398radです.
armグループの0番目のサーボ角度は1.491029radです.
armグループの1番目のサーボ角度は-0.466330radです.
armグループの2番目のサーボ角度は0.087437radです.
armグループの3番目のサーボ角度は-1.848447radです.
armグループの4番目のサーボ角度は-0.044485radです.
armグループの5番目のサーボ角度は0.233165radです.
armグループの6番目のサーボ角度は0.961806radです.
...
```

### 解説

サーボモータの現在角度を取得するため、
コンフィグファイルのジョイントグループに`sync_read:position`を追加します。

```yaml
joint_groups:
  ジョイントグループ名(1):
    joints:
      - ジョイント名(1)
      - ジョイント名(2)
      - ジョイント名(3)
    sync_read:
      - position
  ジョイントグループ名(2):
    joints:
      - ジョイント名(4)
      - ジョイント名(5)
    sync_read:
      - position
```

サーボモータの現在情報を読み取るため、`Hardware.sync_read(group_name)`を実行します。
引数にはジョイントグループ名を入力します。

この関数を実行すると、ロボットとの通信が発生します。

```cpp
hardware.sync_read("arm");
```

サーボモータの現在角度を取得するため、`Hardware.get_position(id, position)`を実行します。
引数にはサーボモータのIDと、角度(radian)の格納先を入力します。

```cpp
double position;
hardware.get_position(2, position);
```

ジョイント名で指定することも可能です。

```cpp
double position;
hardware.get_position("joint1", position);
```

ジョイントグループの現在角度を一括で取得する場合は、`Hardware.get_positions(group_name, positions)`を実行します。
引数にはジョイントグループ名と、角度の格納先を入力します。

```cpp
std::vector<double> positions;
hardware.get_positions("arm", positions);
```

## サーボモータの目標角度を書き込む

次のコマンドを実行します。
CRANE-X7は垂直姿勢に移行し、ハンドを開閉します。
Sciurus17は胴体が正面を向く姿勢に移行し、右ハンドを開閉します。

***安全のためロボットの周りに物や人を近づけないでください。***

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_write_position
# Sciurus17の場合
$ ./s17_write_position
```

実行結果（CRANE-X7の場合）

```sh
...
armグループのサーボ最大加速度を0.5pi rad/s^2、最大速度を0.5pi rad/sに設定します.
handグループのサーボ最大加速度を0.5pi rad/s^2、最大速度を0.5pi rad/sに設定します.
arm、handグループのサーボ位置制御PIDゲインに(800, 0, 0)を書き込みます.
armグループのサーボ目標角度に0.0 radを書き込みます.
5秒後にX7が垂直姿勢へ移行するため、X7の周りに物や人を近づけないで下さい.
5秒間スリープして動作完了を待ちます.
ID:5のサーボ目標角度に-2.094395 radを書き込みます.
5秒間スリープして動作完了を待ちます.
joint_handジョイントのサーボ目標角度に0.523599 radを書き込みます.
5秒間スリープして動作完了を待ちます.
joint_handジョイントのサーボ目標角度に0.000000 radを書き込みます.
5秒間スリープして動作完了を待ちます.
arm、handグループのサーボ位置制御PIDゲインに(5, 0, 0)を書き込み、脱力させます.
10秒間スリープします.
CRANE-X7との接続を解除します.
```

### 解説

サーボモータに目標角度を書き込むため、
コンフィグファイルのジョイントグループに`sync_write:position`を追加します。

```yaml
joint_groups:
  ジョイントグループ名(1):
    joints:
      - ジョイント名(1)
      - ジョイント名(2)
      - ジョイント名(3)
    sync_write:
      - position
  ジョイントグループ名(2):
    joints:
      - ジョイント名(4)
      - ジョイント名(5)
    sync_write:
      - position
```

サーボモータの最大動作加速度を設定するため、`Hardware.write_max_acceleration_to_group(group_name, acceleration)`を実行します。
引数にはジョイントグループ名と、加速度(radian / s^2)を入力します。

```cpp
hardware.write_max_acceleration_to_group("arm", 0.5 * M_PI);
```

サーボモータの最大動作速度を設定するため、`Hardware.write_max_velocity_to_group(group_name, velocity)`を実行します。
引数にはジョイントグループ名と、速度(radian / s)を入力します。

```cpp
hardware.write_max_velocity_to_group("arm", 0.5* M_PI);
```

サーボモータの位置制御PIDゲインを設定するため、`Hardware.write_position_pid_gain_to_group(group_name, p, i, d)`を実行します。
引数にはジョイントグループ名と、PIDゲインを入力します。

```cpp
hardware.write_position_pid_gain_to_group("arm", 800, 0, 0);
```

指定したサーボモータにPIDゲインを設定する場合は、`Hardware.write_position_pid_gain(id, p, i, d)`を実行します。

```cpp
hardware.write_position_pid_gain(2, 800, 0, 0);
```

ジョイント名で指定することも可能です。

```cpp
hardware.write_position_pid_gain("joint1", 800, 0, 0);
```

サーボモータの目標角度を設定するため、`Hardware.set_position(id, position)`を実行します。
引数にはサーボモータのIDと、目標角度(radian)を入力します。
設定した目標角度をサーボモータへ書き込むためには、`Hardware.sync_write()`を実行します。

```cpp
double position = 0.0;
hardware.set_position(2, position);
```

ジョイント名で指定することも可能です。

```cpp
double position = 0.0;
hardware.set_position("joint1", position);
```

ジョイントグループの目標角度を一括で設定する場合は、`Hardware.set_positions(group_name, positions)`を実行します。
引数にはジョイントグループ名と、目標角度を入力します。

```cpp
std::vector<double> positions(7, 0.0);
hardware.get_positions("arm", positions);
```

サーボモータへ目標値を書き込むため、`Hardware.sync_write(group_name)`を実行します。
引数にはジョイントグループ名を入力します。

この関数を実行すると、ロボットとの通信が発生します。

```cpp
hardware.sync_write("arm");
```

## スレッドでサーボモータの角度を読み書きする

次のコマンドを実行します。
CRANE-X7は、肘の現在角度に合わせてハンドを開閉します。
Sciurus17は、右肘の現在角度に合わせて右ハンドを開閉します。

***安全のためロボットの周りに物や人を近づけないでください。***

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_thread
# Sciurus17の場合
$ ./s17_thread
```

実行結果（CRANE-X7の場合）

```sh
handジョイントグループのトルクをONにします.
read/writeスレッドを起動します.
5秒後にX7のハンドが開くので、ハンドに触れないでください.
armグループの0番目のサーボ角度は-1.070719radです.
armグループの1番目のサーボ角度は-1.632156radです.
armグループの2番目のサーボ角度は0.897379radです.
armグループの3番目のサーボ角度は-2.686000radです.
armグループの4番目のサーボ角度は-0.684155radです.
armグループの5番目のサーボ角度は1.141282radです.
armグループの6番目のサーボ角度は-0.954136radです.
handグループの0番目のサーボ角度は-0.007670radです.
armグループの0番目のサーボ角度は-1.070719radです.
armグループの1番目のサーボ角度は-1.632156radです.
armグループの2番目のサーボ角度は0.897379radです.
armグループの3番目のサーボ角度は-2.686000radです.
armグループの4番目のサーボ角度は-0.684155radです.
...
```

### 解説

サーボモータのデータを読み書きするスレッドを起動するため、`Hardware.start_thread(group_names, update_cycle_ms)`を実行します。
引数にはジョイントグループ名と、スレッドの更新周期(msec)を入力します。

スレッド内では`Hardware.sync_read()`と`Hardware.sync_write()`が実行されます。

更新周期を短くする場合は[通信設定](#通信設定)を参考にUSB通信ポートの遅延を最小にしてください。

```cpp
std::vector<std::string> group_names = {"arm", "hand"};
hardware.start_thread(group_names, std::chrono::milliseconds(10));
```

スレッドを停止する場合は、`Hardware.stop_thread()`を実行します。

```cpp
hardware.stop_thread();
```

## サーボモータの速度、電流、入力電圧、温度を読み取る

次のコマンドを実行します。サーボモータの現在角度、速度、電流、入力電圧、温度がターミナルに表示されます。

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_read_present_values
# Sciurus17の場合
$ ./s17_read_present_values
```

実行結果（CRANE-X7の場合）

```sh
...
サーボモータの電流値を観察しやすくするため、
5秒後にhandグループのトルクがONします.
arm : index, position[rad], velocity[rad/s], current[A], voltage[V], temperature[deg]
0, -0.174874, 0.000000, 0.000000, 12.200000, 31, 
1, -1.601476, 0.000000, 0.000000, 12.100000, 31, 
2, -1.187301, 0.000000, 0.000000, 12.000000, 31, 
3, -2.468175, 0.000000, -0.005380, 11.800000, 32, 
4, 1.883728, -0.095923, -0.002690, 11.700000, 32, 
5, 1.144350, 0.023981, 0.000000, 11.700000, 34, 
6, -0.809942, -0.503597, -0.008070, 11.700000, 34, 
hand : index, position[rad], velocity[rad/s], current[A], voltage[V], temperature[deg]
0, 0.757787, 0.000000, 0.123740, 11.700000, 36, 
...
```

### 解説

サーボモータの現在値を取得するため、
コンフィグファイルのジョイントグループに`sync_read:{position, velocity, current, voltage, temperature}`を追加します。

読み取りたいデータ項目を`sync_read`に追加してください。

```yaml
joint_groups:
  ジョイントグループ名(1):
    joints:
      - ジョイント名(1)
      - ジョイント名(2)
      - ジョイント名(3)
    sync_read:
      - position
      - velocity
      - current
      - voltage
      - temperature 
  ジョイントグループ名(2):
    joints:
      - ジョイント名(4)
      - ジョイント名(5)
    sync_read:
      - current
      - temperature 
```

サーボモータの現在情報を読み取るため、`Hardware.sync_read(group_name)`を実行します。
引数にはジョイントグループ名を入力します。

この関数を実行すると、ロボットとの通信が発生します。

```cpp
hardware.sync_read("arm");
```

サーボモータの速度を取得するため、`Hardware.get_velocity(id, velocity)`を実行します。
引数にはサーボモータのIDと、速度(radian per second)の格納先を入力します。

```cpp
double velocity;
hardware.get_velocity(2, velocity);
```

サーボモータの電流(ampere)は、`Hardware.get_current(id, current)`で取得できます。

```cpp
double current;
hardware.get_current(2, current);
```

サーボモータの入力電圧(volt)は、`Hardware.get_voltage(id, voltage)`で取得できます。

```cpp
double voltage;
hardware.get_voltage(2, voltage);
```

サーボモータの温度(degree Celsius)は、`Hardware.get_temperature(id, temperature)`で取得できます。

```cpp
int8_t temperature;
hardware.get_temperature(2, temperature);
```

ジョイント名で指定することも可能です。

```cpp
double velocity;
double current;
double voltage;
int8_t temperature;
hardware.get_velocity("joint1", velocity);
hardware.get_current("joint1", current);
hardware.get_voltage("joint1", voltage);
hardware.get_temperature("joint1", temperature);
```

ジョイントグループの現在値を一括で取得することも可能です。

```cpp
std::vector<double> velocities;
std::vector<double> currents;
std::vector<double> voltages;
std::vector<int8_t> temperatures;
hardware.get_velocities("arm", velocities);
hardware.get_currents("arm", currents);
hardware.get_voltages("arm", voltages);
hardware.get_temperatures("arm", temperatures);
```

## サーボモータの目標速度を書き込む

未実装です

## サーボモータの目標電流を書き込む

未実装です
