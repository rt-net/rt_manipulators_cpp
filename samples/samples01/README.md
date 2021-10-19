# サンプル集01 ロボットのサーボモータを動かす

次のコマンドを実行して、サンプル集をビルドします。

```sh
$ ./build_samples.bash
```

ビルドに成功すると`samples01/bin/`ディレクトリに実行ファイルが生成されます。

## サーボのトルクをON/OFFする

次のコマンドを実行します。
```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_onoff
# Sciurus17の場合
$ ./s17_onoff
```

サンプル内にスリープ処理が含まれているため、Sciurus17制御基板の通信タイムアウト機能が働きます。
[Sciurus17入門ガイド](https://rt-net.jp/products/sciurus17/)に記載されている手順でタイムアウト機能を解除してください。


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
  サーボグループ名(1):
    joints:
      - ジョイント名(1)
      - ジョイント名(2)
      - ジョイント名(3)
  サーボグループ名(2):
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
引数にはサーボグループ名を入力します。
サーボグループのすべてのモータのトルクがONされます。

```cpp
hardware.torque_on("arm");
```

トルクをOFFするために`Hardware.torque_off(group_name)`を実行します。
引数にはサーボグループ名を入力します。
サーボグループのすべてのモータのトルクがFFされます。

```cpp
hardware.torque_off("arm");
```

ロボットとの接続を解除するために`Hardware.disconnect()`を実行します。

```cpp
hardware.disconnect();
```
