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
