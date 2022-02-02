# サンプル集03 目標軌道に沿ってリンクを動かす

- [サンプル集03 目標軌道に沿ってリンクを動かす](#サンプル集03-目標軌道に沿ってリンクを動かす)
  - [サンプルのビルド](#サンプルのビルド)
  - [ロボットのサーボモータの動かし方について](#ロボットのサーボモータの動かし方について)
  - [運動学計算で使用するライブラリについて](#運動学計算で使用するライブラリについて)
  - [解析的に逆運動学を解いて手先を任意の位置・姿勢に移動させる](#解析的に逆運動学を解いて手先を任意の位置姿勢に移動させる)
    - [解説](#解説)

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

***安全のためロボットの周りに物や人を近づけないでください。***

```sh
# CRANE-X7の場合
$ cd bin/
$ ./x7_3dof_inverse_kinematics
```

実行結果（CRANE-X7の場合）

```sh
./x7_3dof_inverse_kinematics
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

### 解説

解析的に逆運動学を解く関数を使用する場合、
`samples03/include/inverse_kinematics.hpp`をincludeします。

```cpp
#include "inverse_kinematics.hpp"
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
