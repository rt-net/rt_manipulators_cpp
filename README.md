# RTマニピュレータC++ライブラリとサンプル集（rt_manipulators_cpp）

本リポジトリは、株式会社アールティが販売している
[アームロボット**CRANE-X7(クラインエックスセブン)**](https://rt-net.jp/products/crane-x7/)
と
[上半身人型ロボット**Sciurus17(シューラスセブンティーン)**](https://rt-net.jp/products/sciurus17/)
を動かすための、C++ライブラリおよびサンプルプログラムを提供します。


## 動作環境

- CMake (>= 3.1.0)
- g++ (>= 7.5.0)
- [DYNAMIXEL SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) (linux64向けにビルド&インストール)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- Linux OS
    - Ubuntu 18.04
    - Ubuntu 20.04
- ロボット
    - [CRANE-X7](https://rt-net.jp/products/crane-x7/)
    - [Sciurus17](https://rt-net.jp/products/sciurus17/)

## インストール方法

### DYNAMIXEL SDKのインストール

[DYNAMIXEL SDKのe-manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#cpp-linux)
を参考に、linux 64bit環境用にDYNAMIXEL SDKをビルド＆インストールします。

```sh
$ sudo apt install build-essential
$ cd ~
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ cd ~/DynamixelSDK/c++/build/linux64
$ make
$ sudo make install
```

### その他依存関係のインストール

次のコマンドを実行します。

```sh
$ sudo apt install libyaml-cpp-dev 
```

### RTマニピュレータC++ライブラリのビルド&インストール

詳細は[rt_manipulators_lib/README.md](./rt_manipulators_lib/README.md)を参照してください。

```sh
$ git clone https://github.com/rt-net/rt_manipulators_cpp
$ cd rt_manipuators_cpp/rt_manipulators_lib
$ ./build_install_library.bash
```

## サンプルプログラムの実行

詳細は[samples/README.md](./samples/README.md)を参照してください。

```sh
$ cd rt_manipulators_cpp/samples/samples01
$ ./build_samples.bash

$ cd bin/
$ ./x7_onoff
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
サーボグループ:armのトルクをONにします.
...
```

## ライセンス

(C) 2021 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。
特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)
または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)
から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
本ソフトウェアに関する無償サポートはありません。  
バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、
それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
