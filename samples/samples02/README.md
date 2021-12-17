# サンプル集02 リンクの位置・姿勢を変更する

- [サンプル集02 リンクの位置・姿勢を変更する](#サンプル集02-リンクの位置姿勢を変更する)
  - [サンプルのビルド](#サンプルのビルド)
  - [ロボットのサーボモータの動かし方について](#ロボットのサーボモータの動かし方について)
  - [順運動学を解いてリンクの位置・姿勢を求める](#順運動学を解いてリンクの位置姿勢を求める)
    - [解説](#解説)

## サンプルのビルド

次のコマンドを実行して、サンプル集をビルドします。

```sh
$ ./build_samples.bash
```

ビルドに成功すると`samples02/bin/`ディレクトリに実行ファイルが生成されます。

## ロボットのサーボモータの動かし方について

あらかじめ、["samples01のREADME"](../samples01/README.md)をよく読み、
ロボットのサーボモータの動かし方を理解してください。

## 順運動学を解いてリンクの位置・姿勢を求める

次のコマンドを実行します。
ロボットの手先リンクの位置・姿勢（Z-Y-Xオイラー角）を表示します。

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_forward_kinematics
# Sciurus17の場合
$ ./s17_forward_kinematics
```

実行結果（CRANE-X7の場合）

```sh
./x7_forward_kinematics
CRANE-X7のサーボモータ角度を読み取り、順運動学を解くサンプルです.
リンク情報ファイル:../config/crane-x7_links.csvを読み込みます
リンクID:1リンク名:CRANE-X7_Base
親:0, 子:2, 姉妹兄弟:0
親リンクに対する関節軸ベクトル a:
0
0
0
親リンクに対する相対位置 b:
...
リンク名: CRANE-X7_Link7
位置 X: 0.0053478       [m]
位置 Y: 0.00468035      [m]
位置 Z: 0.185839        [m]
姿勢 Z:139.47   [deg]
姿勢 Y:80.9454  [deg]
姿勢 X:132.577  [deg]
リンク名: CRANE-X7_Link7
位置 X: 0.0053478       [m]
位置 Y: 0.00468035      [m]
位置 Z: 0.185839        [m]
姿勢 Z:139.47   [deg]
姿勢 Y:80.9454  [deg]
姿勢 X:132.577  [deg]
...
```

### 解説

RTマニピュレータc++ライブラリの運動学機能を使用する場合は`rt_manipulators_cpp/kinematics.hpp`、
`rt_manipulators_cpp/kinematics_utils.hpp`、`rt_manipulators_cpp/link.hpp`をincludeします。

`kinematics.hpp`には運動学を計算する関数が定義されています。
`kinematics_utils.hpp`には運動学計算を補助する関数が定義されています。
`link.hpp`にはリンク情報を持つ`manipulators_link::Link`クラスが定義されています。

```cpp
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"
```

`manipulators_link::Link`クラスは次のようなパラメータを保持します。
3次元空間のベクトルや行列は[Eigenライブラリ](https://eigen.tuxfamily.org/index.php?title=Main_Page)
を使用して表現しています。

```cpp
class Link{
 public:
  std::string name;  // リンク名
  int sibling;  // 姉妹兄弟リンクID
  int child;  // 子リンクID
  int parent;  // 親リンクID
  Eigen::Vector3d p;  // ワールド座標系での位置
  Eigen::Matrix3d R;  // ワールド座標系での姿勢
  Eigen::Vector3d v;  // ワールド座標系での速度
  Eigen::Vector3d w;  // ワールド座標系での角速度ベクトル
  double q;  // 関節位置
  double dq;  // 関節速度
  double ddq;  // 関節加速度
  Eigen::Vector3d a;  // 親リンクに対する関節軸ベクトル
  Eigen::Vector3d b;  // 親リンクに対する相対位置
  double m;  // 質量
  Eigen::Vector3d c;  // 自リンクに対する重心位置
  Eigen::Matrix3d I;  // 自リンクに対する慣性テンソル
};
```

ロボットのリンク構成は`manipulators_link::Link`クラスを`std::vector`に格納して表現します。

```cpp
// ロボットのリンク構成を表現
std::vector<manipulators_link::Link> links;
```

リンク構成を表現したCSVファイルを`kinematics_utils::parse_link_config_file(file_path)`で読み込むことで、
リンク構成を取得できます。

**※重心位置、慣性テンソルの読み込みは未実装です**

```cpp
// CSVファイルを解析してリンク構成を取得する
std::vector<manipulators_link::Link> links = kinematics_utils::parse_link_config_file("../config/crane-x7_links.csv");
```

`config`ディレクトリに用意されているCSVファイルは、
公開されているCRANE-X7とSciurus17のリンク情報リファレンスをCSV形式でダウンロードしたものです。

- [CRANE-X7リンク情報リファレンス](https://docs.google.com/spreadsheets/d/1I268mnab4m-f6us0Au3AGd64-2iGkSwxaLrDplSjHY8/edit#gid=735472399)
- [Sciurus17リンク情報リファレンス](https://docs.google.com/spreadsheets/d/1Q4z3M3cS1pQOEn3iXKLiIQIOr6czvECxSXEPS2-PGvA/edit#gid=1687288769)

各リンクのローカル座標系の姿勢は、ジョイント角度(q)が0 degのとき、
ワールド座標系の姿勢(正面がX軸正方向、鉛直上向きがZ軸正方向となる[右手系](https://ja.wikipedia.org/wiki/%E5%8F%B3%E6%89%8B%E7%B3%BB))と一致します。

リンク番号とリンク構成(`std::vector<manipulators_link::Link>`)のインデックスは一致するため、次のようにリンク情報へアクセスできます。

```cpp
// ダミーリンクの情報を取得
auto dummy_link = links[0];
// ベースリンクの情報を取得
auto link_base_position = links[1];
// リンクID4の位置を取得
auto position = links[4].p;
// リンクID5を動かすジョイントの現在角度を書き込む
links[5].q = 0.0;
```

リンクの位置姿勢を更新するため、`kinematics::forward_kinematics(links, start_id)`を実行し、順運動学を解きます。
引数にはリンク構成と、順運動学の計算開始リンク番号を入力します。

```cpp
// ベースリンクから逐次的に順運動学を解き、各リンクの位置・姿勢を更新する
kinematics::forward_kinematics(links, 1);
```

リンク構成の情報を一括で出力するため、`kinematics_utils::print_links(links, start_id)`を実行します。
引数にはリンク構成と、リンク情報の出力開始リンク番号を入力します。

```cpp
// ベースリンクから終端リンクまでのリンク情報を出力する
kinematics_utils::print_links(links, 1);
```

## 逆運動学を解いて手先を任意の位置・姿勢に移動させる

次のコマンドを実行します。
ロボットの手先が0.2m前方の5点に向かって移動します。

***安全のためロボットの周りに物や人を近づけないでください。***

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_inverse_kinematics
# Sciurus17の場合
# 未実装
```

実行結果（CRANE-X7の場合）

```sh
./x7_inverse_kinematics
手先目標位置・姿勢をもとに逆運動学を解き、CRANE-X7を動かすサンプルです.
リンク情報ファイル:../config/crane-x7_links.csvを読み込みます
リンクID:1リンク名:CRANE-X7_Base
親:0, 子:2, 姉妹兄弟:0
親リンクに対する関節軸ベクトル a:
0
0
0
親リンクに対する相対位置 b:
...
5秒後にX7が垂直姿勢へ移行するため、X7の周りに物や人を近づけないで下さい.
正面へ移動
目標位置:
0.2
  0
0.3
目標姿勢:
2.22045e-16           0           1
          0           1           0
         -1           0 2.22045e-16
----------------------
左上へ移動
目標位置:
0.2
0.2
0.5
目標姿勢:
1 0 0
0 1 0
0 0 1
----------------------
左下へ移動
...
```

### 解説

逆運動学を解く前に、リンク構成の各リンク関節位置の下限値`q_min`(radian)と上限値`q_max`(radian)を入力します。

```cpp
// リンクID2関節位置の下限値を設定する
links[2].q_min = -M_PI;
// リンクID2関節位置の上限値を設定する
links[2].q_max = -M_PI;
```

`Hardware.get_min_position_limit(id, min_position_limit)`、
`Hardware.get_max_position_limit(id, max_position_limit)`、
を使用することで、各サーボモータ関節位置の下限値と上限値を取得できます。
引数にはサーボモータのIDを限界値の格納先を入力します。

```cpp
// リンクID2に対応するサーボモータの限界値を取得し、リンクパラメータに設定する
hardware.get_max_position_limit(links[2].dxl_id, links[2].max_q);
hardware.get_min_position_limit(links[2].dxl_id, links[2].min_q);
```

任意のリンクを任意の位置・姿勢へ移動させるため、
`kinematics::inverse_kinematics_LM(links, target_id, target_p, target_R, q_list)`を実行し、逆運動学を解きます。
引数にはリンク構成と、移動させたいリンク、目標位置(`Eigen::Vector3d`)、目標姿勢(`Eigen::Matrix3d`)、
関節位置の格納先(`std::map<unsigned int, double>` または `kinematics_utils::q_list_t`)を入力します。

`kinematics_utils::rotation_from_euler_ZYX(z, y, x)`を使用すると、
オイラー各ZYXから回転行列を生成できます。

```cpp
Eigen::Vector3d target_p;
Eigen::Matrix3d target_R;
kinematics_utils::q_list_t q_list;

// 目標位置(X方向に0.2m、Y方向に0.0m、Z方向に0.3m)
target_p << 0.2, 0.0, 0.3;
// 目標姿勢(Y軸周りにpi/2回転)
target_R = kinematics_utils::rotation_from_euler_ZYX(0, M_PI_2, 0);

// 逆運動学を解き、関節位置を取得する
kinematics::inverse_kinematics_LM(links, 8, target_p, target_R, q_list);
// 関節位置をサーボモータへ設定する
for (const auto & [target_id, q_value] : q_list) {
  hardware.set_position(links[target_id].dxl_id, q_value);
}
```

`kinematics::inverse_kinematics_LM()`には、
[杉原 知道.
Levenberg-Marquardt法による可解性を問わない逆運動学.
日本ロボット学会誌 Vol.29, 2011](
https://www.jstage.jst.go.jp/article/jrsj/29/3/29_3_269/_pdf)
に記載されたアルゴリズムを実装しています。
