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

#include <iostream>

#include "kinematics.hpp"
#include "kinematics_utils.hpp"

namespace kinematics {

namespace utils = kinematics_utils;

void forward_kinematics(std::vector<manipulators_link::Link> & links, const int & start_id) {
  // 指定されたリンクIDからchild、siblingに向かって逐次的に順運動学を解き、
  // リンクの位置・姿勢を更新する
  if (start_id == 0) {
    return;
  }

  if (start_id >= links.size()) {
    std::cerr << "引数start_idには引数linksの要素数より小さい数字を入力して下さい" << std::endl;
    return;
  }

  if (start_id != 1) {
      auto parent_id = links[start_id].parent;
      links[start_id].p = links[parent_id].R * links[start_id].b + links[parent_id].p;
      links[start_id].R = links[parent_id].R *
        utils::rodrigues(links[start_id].a, links[start_id].q);
  }

  forward_kinematics(links, links[start_id].sibling);
  forward_kinematics(links, links[start_id].child);
}

bool inverse_kinematics_LM(
  const kinematics_utils::links_t & links, const kinematics_utils::link_id_t & target_id,
  const Eigen::Vector3d & target_p, const Eigen::Matrix3d & target_R,
  kinematics_utils::q_list_t & result_q_list) {
  // 参考: https://www.jstage.jst.go.jp/article/jrsj/29/3/29_3_269/_pdf/-char/ja
  auto calc_links = links;
  auto route = kinematics_utils::find_route(links, target_id);
  auto q_list = kinematics_utils::get_q_list(links, route);

  // We : 拘束条件に対する重み行列
  double we_pos = 10 * 1/0.1;
  double we_ang = 1/(2*M_PI);
  auto We_vec = Eigen::VectorXd(6);
  We_vec << we_pos, we_pos, we_pos, we_ang, we_ang, we_ang;
  Eigen::MatrixXd We = We_vec.asDiagonal();

  // qを0にリセットする
  for (auto q_i = q_list.begin(); q_i != q_list.end(); ++q_i) {
    q_i->second = 0;
  }

  // qをセットしてリンク情報を更新する
  kinematics_utils::set_q_list(calc_links, q_list);
  forward_kinematics(calc_links, 1);

  auto error = kinematics_utils::calc_error(target_p, target_R, calc_links[target_id]);

  for (int n=0; n < 100; n++) {
    // 基礎ヤコビ行列を計算
    auto J = kinematics_utils::calc_basic_jacobian(calc_links, target_id);

      // 位置・姿勢の誤差を計算
    error = kinematics_utils::calc_error(target_p, target_R, calc_links[target_id]);

    // 誤差が小さければ終了
    if (error.norm() < 1.0E-6) {
      // std::cout<<"IK成功しました"<<std::endl;
      // std::cout << "error: " << error.norm() << std::endl;
      result_q_list = q_list;
      return true;
    }

    // 減衰因子の計算
    auto E = 0.5 * error.transpose() * We * error;
    double omega = 0.01;
    auto Wn_ = omega * Eigen::MatrixXd::Identity(q_list.size(), q_list.size());
    auto Wn = E * Eigen::MatrixXd::Identity(q_list.size(), q_list.size()) + Wn_;

    // LM法の更新則
    auto H = J.transpose() * We * J + Wn;
    auto dq = H.inverse() * J.transpose() * We * error;  // H-1 * g

    // dqをqに加算
    int dq_i = 0;
    for (auto q_i = q_list.begin(); q_i != q_list.end(); ++q_i) {
      q_i->second += dq(dq_i);
      dq_i++;
    }

    // qをセットしてリンク情報を更新する
    kinematics_utils::set_q_list(calc_links, q_list);
    forward_kinematics(calc_links, 1);
  }

  std::cerr << "逆運動学の計算に失敗しました" << std::endl;
  std::cerr << "誤差ベクトル:" << std::endl << error << std::endl;
  std::cerr << "誤差ノルム:" << error.norm() << std::endl;
  result_q_list = q_list;
  return false;
}

}  // namespace kinematics
