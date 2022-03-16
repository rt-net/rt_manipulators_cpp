# RTマニピュレータC++ライブラリ

## ライブラリのインストール

[build_install_library.bash](./build_install_library.bash)
を実行すると、ライブラリをビルド＆インストールできます。

```sh
$ ./build_install_library.bash
ライブラリをビルドします
-- The C compiler identification is GNU 7.5.0
-- The CXX compiler identification is GNU 7.5.0
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
...
[100%] Built target rt_manipulators_cpp
ライブラリをビルドしました
ライブラリをインストールします
[100%] Built target rt_manipulators_cpp
Install the project...
-- Install configuration: ""
-- Installing: /usr/local/lib/librt_manipulators_cpp.so.1.0.0
-- Installing: /usr/local/lib/librt_manipulators_cpp.so.1
-- Installing: /usr/local/lib/librt_manipulators_cpp.so
-- Up-to-date: /usr/local/include/rt_manipulators_cpp
-- Installing: /usr/local/include/rt_manipulators_cpp/hardware.hpp
-- Installing: /usr/local/include/rt_manipulators_cpp/joint.hpp
ライブラリをインストールしました
```

ビルドに成功すると、CMakeのデフォルトインストールディレクトリ(例：`/usr/local/lib`、`/usr/local/include`)に共有ライブラリ(`librt_manipulators_cpp.so`)とヘッダーファイル(`rt_manipulators_cpp/*.hpp`)がインストールされます。

共有ライブラリは次のように使用できます。

```sh
$ g++ test.cpp -lrt_manipulators_cpp
```

## ライブラリのアンインストール

[uninstall_library.bash](./uninstall_library.bash)
を実行すると、ライブラリをアンインストールできます。

```sh
$ ./uninstall_library.bash
ライブラリをアンインストールするため、次のファイルを削除します
/usr/local/lib/librt_manipulators_cpp.so.1.0.0
/usr/local/lib/librt_manipulators_cpp.so.1
/usr/local/lib/librt_manipulators_cpp.so
/usr/local/include/rt_manipulators_cpp/hardware.hpp
/usr/local/include/rt_manipulators_cpp/joint.hpp
アンインストールしました
```

## ライブラリの使い方

ライブラリの使い方は[サンプル集のREADME.md](../samples/README.md)を参照してください。

## ライブラリのファイル構成

ライブラリは、ロボットのハードウェアを動かすために必要なファイルと、
運動学を計算するために必要なファイルで構成されています。

### ハードウェア関連

- `hardware.hpp/cpp` : `Hardware`クラスを実装しています
- `hardware_communicator.hpp/cpp` : `Hardware`クラスのうち、Dynamixelとの通信機能を実装しています
- `hardware_joints.hpp/cpp` : `Hardware`クラスのうち、ジョイント情報を扱う機能を実装しています
- `joints.hpp/cpp` : ジョイント情報を定義しています
- `config_file_parser.hpp/cpp` : コンフィグファイルの読み取りを担います
- `dynamixel_*` : 各Dynamixelと通信するためのデータ変換を担います

### 運動学関連

- `kinematics.hpp/cpp` : 順運動学、逆運動学を解く関数を実装しています
- `kinematics_utils.hpp/cpp` : 運動学計算を補助する関数を実装しています
- `link.hpp` : リンク情報を定義しています

## ライブラリのテスト

### GoogleTestのインストール

```sh
$ mkdir ~/gtest
$ cd ~/gtest
$ curl -OL https://github.com/google/googletest/archive/release-1.11.0.tar.gz
$ tar -xvf release-1.11.0.tar.gz
$ mkdir googletest-release-1.11.0/build
$ cd googletest-release-1.11.0/build
$ cmake ..
$ sudo make install
```

### テストの実行

```sh
$ ./run_test_library.bash
ライブラリをテストします
-- The C compiler identification is GNU 7.5.0
-- The CXX compiler identification is GNU 7.5.0
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
...
    Start 1: JointTest.initialize_id
1/1 Test #1: JointTest.initialize_id ..........   Passed    0.00 sec

100% tests passed, 0 tests failed out of 1

Total Test time (real) =   0.00 sec
ライブラリをテストしました
```
