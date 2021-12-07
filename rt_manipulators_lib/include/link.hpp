// Copyright 2021 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RT_MANIPULATORS_LIB_INCLUDE_LINK_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_LINK_HPP_

#include <eigen3/Eigen/Dense>
#include <string>

namespace manipulators_link {

// Linkクラスの参考情報：
// 梶田秀司 編著, ヒューマノイドロボット 改訂２版, オーム社, 2020
class Link{
 public:
  Link():
    name("ダミーリンク"), sibling(0), child(0), parent(0), q(0), dq(0), ddq(0), m(0) {
    // 0ベクトル、単位行列で初期化
    p.setZero();
    R.setIdentity();
    v.setZero();
    w.setZero();
    a.setZero();
    b.setZero();
    c.setZero();
    I.setIdentity();
  }
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

}  // namespace manipulators_link

#endif  // RT_MANIPULATORS_LIB_INCLUDE_LINK_HPP_
