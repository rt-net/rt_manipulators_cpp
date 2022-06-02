# サンプル集03 解析的な逆運動学、トルク制御

- [サンプル集03 解析的な逆運動学、トルク制御](#サンプル集03-解析的な逆運動学トルク制御)
  - [サンプルのビルド](#サンプルのビルド)
  - [ロボットのサーボモータの動かし方について](#ロボットのサーボモータの動かし方について)
  - [運動学計算で使用するライブラリについて](#運動学計算で使用するライブラリについて)
  - [解析的に逆運動学を解いて手先を任意の位置・姿勢に移動させる](#解析的に逆運動学を解いて手先を任意の位置姿勢に移動させる)
    - [動画](#動画)
    - [解説](#解説)
  - [重力補償トルクをサーボモータに入力する](#重力補償トルクをサーボモータに入力する)
    - [動画](#動画-1)
    - [解説](#解説-1)

## サンプルのビルド

次のコマンドを実行して、サンプル集をビルドします。

```sh
$ ./build_samples.bash
```

ビルドに成功すると`samples03/bin/`ディレクトリに実行ファイルが生成されます。

## ロボットのサーボモータの動かし方について

あらかじめ、["samples01のREADME"](../samples01/README.md)をよく読み、
ロボットのサーボモータの動かし方を理解してください。

## 運動学計算で使用するライブラリについて

運動学計算で使用するライブラリ
(`kinematics.hpp`、`kinematics_utils.hpp`、`link.hpp`)
の使い方は["samples02のREADME"](../samples02/../README.md)
を参照してください。

## 解析的に逆運動学を解いて手先を任意の位置・姿勢に移動させる

次のコマンドを実行します。
CRANE-X7では、手先が0.3m前方の5点に向かって移動します。
Sciurus17では、右手先が0.4m前方の4点に向かって移動します。

***安全のためロボットの周りに物や人を近づけないでください。***

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_3dof_inverse_kinematics
# Sciurus17の場合
$ ./s17_3dof_inverse_kinematics
```

実行結果（CRANE-X7の場合）

```sh
$ ./x7_3dof_inverse_kinematics
手先目標位置をもとに解析的に逆運動学を解き、CRANE-X7を動かすサンプルです.
リンク情報ファイル:../config/crane-x7_links.csvを読み込みます
...

5秒後にX7が垂直姿勢へ移行するため、X7の周りに物や人を近づけないで下さい.
----------------------
目標位置:
0.2
  0
0.2
IKに成功しました
左奥へ移動
...

ピッキング姿勢のIKを解きます
左奥へ移動
----------------------
目標位置:
 0.3
 0.1
0.05
IKに成功しました
左手前へ移動
...
```

### 動画

[![](https://img.youtube.com/vi/amp-ql9TS6w/sddefault.jpg)](https://youtu.be/amp-ql9TS6w)

### 解説

解析的に逆運動学を解く関数を使用する場合、
`samples03/include/rt_manipulators_ik.hpp`をincludeします。

```cpp
#include "rt_manipulators_ik.hpp"
```

CRANE-X7向けの逆運動学計算関数として、
`samples03::x7_3dof_inverse_kinematics(links, target_p, q_list)`と
`samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list)`
があります。
引数にはリンク構成と、目標位置(`Eigen::Vector3d`)、
関節角度の格納先(`std::map<unsigned int, double>` または `kinematics_utils::q_list_t`)を入力します。

`samples03::x7_3dof_inverse_kinematics(links, target_p, q_list)`
は、第6リンクの原点を目標位置(`target_p`)として受け取り、
第1、2、4関節の目標角度を出力する関数です。

`samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list)`
は、ハンドの先端を目標位置(`target_p`)として受け取り、
第1、2、4関節と、第6、7関節の目標角度を出力する関数です。
手先が地面(-Z)方向を向くように逆運動学を解きます。

```cpp
Eigen::Vector3d target_p;
kinematics_utils::q_list_t q_list;

// 目標位置(X方向に0.2m、Y方向に0.0m、Z方向に0.3m)
target_p << 0.2, 0.0, 0.3;

// 逆運動学を解き、関節角度を取得する
samples03::x7_3dof_inverse_kinematics(links, target_p, q_list);
// 関節角度をサーボモータへ設定する
for (const auto & [target_id, q_value] : q_list) {
  hardware.set_position(links[target_id].dxl_id, q_value);
}
```

```cpp
Eigen::Vector3d target_p;
kinematics_utils::q_list_t q_list;
// ピッキング姿勢の逆運動学
// 手先の目標位置(X方向に0.2m、Y方向に0.0m、Z方向に0.0m)
target_p << 0.2, 0.0, 0.0;

// 逆運動学を解き、関節角度を取得する
samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list);
// 関節角度をサーボモータへ設定する
for (const auto & [target_id, q_value] : q_list) {
  hardware.set_position(links[target_id].dxl_id, q_value);
}
```

Sciurus17向けの逆運動学計算関数には、
`samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list)`と
`samples03::s17_3dof_right_arm_picking_inverse_kinematics(links, target_p, q_list)`
を用意しています。
引数にはリンク構成と、目標位置(`Eigen::Vector3d`)、
関節角度の格納先(`std::map<unsigned int, double>` または `kinematics_utils::q_list_t`)を入力します。

`samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list)`
は、右腕の第6リンクの原点を目標位置(`target_p`)として受け取り、
右腕の第1、2、4関節の目標角度を出力する関数です。

`samples03::s17_3dof_right_arm_picking_inverse_kinematics(links, target_p, q_list)`
は、右ハンドの先端を目標位置(`target_p`)として受け取り、
右腕の第1、2、4関節と、第5、6、7関節の目標角度を出力する関数です。
手先が地面(-Z)方向を向くように逆運動学を解きます。

```cpp
Eigen::Vector3d target_p;
kinematics_utils::q_list_t q_list;

// 右手目標位置(X方向に0.2m、Y方向に-0.3m、Z方向に0.2m)
target_p << 0.2, -0.3, 0.2;

// 逆運動学を解き、関節角度を取得する
samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list);
// 関節角度をサーボモータへ設定する
for (const auto & [target_id, q_value] : q_list) {
  hardware.set_position(links[target_id].dxl_id, q_value);
}
```

```cpp
Eigen::Vector3d target_p;
kinematics_utils::q_list_t q_list;
// ピッキング姿勢の逆運動学
// 手先の目標位置(X方向に0.2m、Y方向に-0.3m、Z方向に0.0m)
target_p << 0.2, -0.3, 0.0;

// 逆運動学を解き、関節角度を取得する
samples03::s17_3dof_right_arm_picking_inverse_kinematics(links, target_p, q_list);
// 関節角度をサーボモータへ設定する
for (const auto & [target_id, q_value] : q_list) {
  hardware.set_position(links[target_id].dxl_id, q_value);
}
```

## 重力補償トルクをサーボモータに入力する

次のコマンドを実行します。
サンプルを終了する場合はキーボードのEscキーを入力してください。

***安全のためいつでもブレーキモードボタンを押せるように準備してください***

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_gravity_compensation
# Sciurus17の場合
$ ./s17_gravity_compensation
```

実行結果（CRANE-X7の場合）

```sh
$ ./x7_gravity_compensation
現在姿勢をもとに重力補償トルクを計算し、CRANE-X7のサーボモータに入力するサンプルです
リンク情報ファイル:../config/crane-x7_links.csvを読み込みます
...

5秒後に重力補償トルクをサーボモータへ入力します
終了する場合はEscキーを押してください
...
```

### 動画

[![](https://img.youtube.com/vi/H_gSnJDFhqs/sddefault.jpg)](https://youtu.be/H_gSnJDFhqs)

### 解説

重力補償トルクを計算する関数を使用する場合、
`samples03/include/rt_manipulators_dynamics.hpp`をincludeします。

```cpp
#include "rt_manipulators_dynamics.hpp"
```

重力補償トルク計算関数として、
`samples03_dynamics::gravity_compensation(links, target_id, torque_to_current, q_list)`
があります。
引数にはリンク構成と、手先リンク番号、
電流トルク比(`std::map<unsigned int, double>` または `samples03_dynamics::torque_to_current_t`)、
関節電流の格納先(`std::map<unsigned int, double>` または `kinematics_utils::q_list_t`)を入力します。

```cpp
kinematics_utils::link_id_t target_id = 8;
samples03_dynamics::torque_to_current_t torque_to_current = {
  {2, 1.0 / 2.20},
  {3, 1.0 / 3.60},
  {4, 1.0 / 2.20},
  {5, 1.0 / 2.20},
  {6, 1.0 / 2.20},
  {7, 1.0 / 2.20},
  {8, 1.0 / 2.20}
};

kinematics_utils::q_list_t q_list;

// 重力補償トルクを計算する
samples03_dynamics::gravity_compensation(links, target_id, torque_to_current, q_list);
// 関節電流をサーボモータへ設定する
for (const auto & [target_id, q_value] : q_list) {
  hardware.set_current(links[target_id].dxl_id, q_value);
}
```
