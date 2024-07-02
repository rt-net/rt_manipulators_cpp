# RTマニピュレータC++ライブラリとサンプル集（rt_manipulators_cpp）

[![industrial_ci](https://github.com/rt-net/rt_manipulators_cpp/actions/workflows/industrial_ci.yaml/badge.svg?branch=ros2)](https://github.com/rt-net/rt_manipulators_cpp/actions/workflows/industrial_ci.yaml)

本リポジトリは、株式会社アールティが販売している
[アームロボット**CRANE-X7(クラインエックスセブン)**](https://rt-net.jp/products/crane-x7/)
と
[上半身人型ロボット**Sciurus17(シューラスセブンティーン)**](https://rt-net.jp/products/sciurus17/)
を動かすための、C++ライブラリおよびサンプルプログラムを提供します。

[<img src=https://rt-net.github.io/images/crane-x7/CRANE-X7-500x500.png width=400px />](https://rt-net.jp/products/crane-x7/)
[<img src=https://rt-net.github.io/images/sciurus17/Sciurus17-500x500.png width=400px />](https://rt-net.jp/products/sciurus17)

## 動作環境

- ROS Foxy
- ROS Galactic
- ROS Humble
- ROS Jazzy
- ROS Rolling

## インストール方法

### RTマニピュレータC++ライブラリのビルド&インストール

```sh
$ cd ~/ros2_ws/src  # workspaceを~/ros2_wsに作成している場合
$ git clone -b ros2 https://github.com/rt-net/rt_manipulators_cpp
$ rosdep install -r -y --from-paths . --ignore-src
$ cd ..
$ colcon build --symlink-install
```

## 免責事項

当該製品および当ソフトウェアの使用中に生じたいかなる損害も株式会社アールティでは一切の責任を負いかねます。
ユーザー自身で作成されたプログラムに適切な制限動作が備わっていない場合、本体の損傷や、本体が周囲や作業者に接触、あるいは衝突し、思わぬ重大事故が発生する危険があります。
ユーザーの責任において十分に安全に注意した上でご使用下さい。

## ライセンス

(C) 2021 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。
特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)
または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)
から確認できます。

## 開発について

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。  
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。  
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、
それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](./CONTRIBUTING.md)に従ってください。

