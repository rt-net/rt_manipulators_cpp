// Copyright 2022 RT Corporation
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

#include <iostream>
#include "rt_manipulators_dynamics.hpp"

namespace samples03_dynamics {

using pos_t = Eigen::Vector3d;
using center_of_mass_t = Eigen::Vector3d;
using force_t = Eigen::Vector3d;
using moment_t = Eigen::Vector3d;
using pos_map_t = std::map<unsigned int, pos_t>;
using center_of_mass_map_t = std::map<unsigned int, center_of_mass_t>;
using force_map_t = std::map<unsigned int, force_t>;
using moment_map_t = std::map<unsigned int, moment_t>;

bool x7_gravity_compensation(
  const kinematics_utils::links_t & links,
  const torque_to_current_t & torque_to_current,
  kinematics_utils::q_list_t & q_list) {
  // 運動方程式から重力補償項のみを計算し、出力トルクを求める
  // 参考：細田耕. 「実践ロボット制御 -基礎から動力学まで-」. オーム社, p137, 2019

  // 計算に使用する変数
  pos_map_t dd_p;  // 位置の2回微分
  center_of_mass_map_t dd_s;  // 重心の加速度
  force_map_t f_hat;  // 外力
  force_map_t f;  // リンクの力
  moment_map_t n;  // リンクのモーメント

  // 根本から手先までのリンク経路を取得
  auto route = kinematics_utils::find_route(links, 8);

  // トルク電流比がセットされているか検証
  for (const auto & link_i : route) {
    if (torque_to_current.count(link_i) == 0) {
      std::cout << "リンクID:" << link_i << "のトルク電流比がセットされていません" << std::endl;
      return false;
    }
  }

  // 変数の初期化
  for (const auto & link_i : route) {
    dd_p[link_i] << 0, 0, 0;
    dd_s[link_i] << 0, 0, 0;
    f_hat[link_i] << 0, 0, 0;
    f[link_i] << 0, 0, 0;
    n[link_i] << 0, 0, 0;
  }
  // 根本リンクを追加する
  auto root_i = links[route[0]].parent;
  dd_p[root_i] << 0, 0, 0;
  // 子リンクを追加する
  auto end_i = links[route.back()].child;
  f[end_i] << 0, 0, 0;
  n[end_i] << 0, 0, 0;

  // 重力加速度を根本リンクの加速度に設定
  Eigen::Vector3d g;
  g << 0, 0, -9.8;
  dd_p[root_i] = -g;

  for (const auto & link_i : route) {
    const auto parent_i = links[link_i].parent;
    const auto link = links[link_i];
    const Eigen::Matrix3d RfromParent = links[parent_i].R.transpose() * link.R;

    // 加速度を求める
    dd_p[link_i] = RfromParent.transpose() * dd_p[parent_i];

    // 外力を求める
    dd_s[link_i] = dd_p[link_i];
    f_hat[link_i] = link.m * dd_s[link_i];
  }

  // 手先から根本に向かってリンクの力とモーメントを求める
  for (auto itr = route.rbegin(); itr != route.rend(); ++itr) {
    const auto link_i = *itr;
    const auto child_i = links[link_i].child;
    const Eigen::Matrix3d RtoChild = links[link_i].R.transpose() * links[child_i].R;

    f[link_i] = RtoChild * f[child_i] + f_hat[link_i];
    n[link_i] = RtoChild * n[child_i]
      + links[child_i].b.cross(RtoChild * f[child_i])
      + links[link_i].c.cross(f_hat[link_i]);

    // モーメントを回転トルクに分解する
    q_list[link_i] = links[link_i].a.transpose() * n[link_i];
    // トルクを電流値に変換する
    q_list[link_i] *= torque_to_current.at(link_i);
  }

  return true;
}

}  // namespace samples03_dynamics
