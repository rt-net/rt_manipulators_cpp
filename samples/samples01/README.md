# サンプル集01 ロボットのサーボモータを動かす

次のコマンドを実行して、サンプル集をビルドします。

```sh
$ ./build_samples.bash
```

ビルドに成功すると`samples01/bin/`ディレクトリに実行ファイルが生成されます。

## 目次

- [通信設定](#通信設定)
- [サーボモータのトルクをON/OFFする](#サーボモータのトルクをonoffする)
  - [解説](#解説)
- [サーボモータの現在角度を読み取る](#サーボモータの現在角度を読み取る)
  - [解説](#解説-1)

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

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_onoff
# Sciurus17の場合
$ ./s17_onoff
```

実行結果

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
10秒間スリープします.ロボットに触れるとトルクがONになっていることがわかります.
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

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_read_position
# Sciurus17の場合
$ ./s17_read_position
```

実行結果

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

サーボモータの現在情報を取得するため、`Hardware.sync_read(group_name)`を実行します。
引数にはジョイントグループ名を入力します。

この関数を実行すると、ロボットとの通信が発生します。

```cpp
hardware.sync_read("arm");
```

サーボモータの現在角度を取得するため、`Hardware.get_position(id, position)`を実行します。
引数にはサーボモータのIDと、角度の格納先を入力します

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
