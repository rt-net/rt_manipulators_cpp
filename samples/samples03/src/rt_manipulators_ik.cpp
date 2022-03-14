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
#include "rt_manipulators_ik.hpp"
#include "rt_manipulators_cpp/kinematics.hpp"

namespace samples03 {

bool q_list_are_in_range(
  const kinematics_utils::links_t & links, const kinematics_utils::q_list_t & q_list) {
  // 目標角度が可動範囲内にあるかチェックする
  for (const auto & [target_id, q_value] : q_list) {
    if (std::isnan(q_value)) {
      std::cerr << "リンクID:" << target_id << " の目標角度が異常値です"  << std::endl;
      return false;
    }

    if (q_value < links[target_id].min_q || q_value > links[target_id].max_q) {
      std::cerr << "リンクID:" << target_id << " の目標角度:" << q_value;
      std::cerr << " が可動範囲外です" << std::endl;
      return false;
    }
  }
  return true;
}

bool x7_3dof_inverse_kinematics(const kinematics_utils::links_t & links,
                                const Eigen::Vector3d & target_p,
                                kinematics_utils::q_list_t & q_list) {
  // 目標位置をもとに、CRANE-X7の第1, 2, 4関節の目標角度を計算する

  // リンク相対位置から、関節間のリンク長を求める
  const double L1 = links[2].b.z() + links[3].b.z();
  const double L2 = links[4].b.z() + links[5].b.z();
  const double L3 = links[6].b.z() + links[7].b.z();
  const double L2_2 = std::pow(L2, 2);
  const double L3_2 = std::pow(L3, 2);

  // 計算式
  // ※CRANE-X7の軸の正回転方向を考慮して、θ2' = -θ2, θ3' = -θ3としている
  // ※θ2,θ3が求まり次第、符号を反転する
  // PX = (L2 * S2 + L3 * S23) * C1
  // PY = (L2 * S2 + L3 * S23) * S1
  // PZ = L1 + L2 * C2 + L3 * C23
  // ※以下、PZ' = PZ - L1 として実装している

  const double PX = target_p.x();
  const double PY = target_p.y();
  const double PZ = target_p.z() - L1;
  const double PX_2 = std::pow(PX, 2);
  const double PY_2 = std::pow(PY, 2);
  const double PZ_2 = std::pow(PZ, 2);
  const double PXYZ_2 = PX_2 + PY_2 + PZ_2;

  // 目標位置が可動範囲内にあるかチェック
  if (std::pow(L2 - L3, 2) > PXYZ_2 || std::pow(L2 + L3, 2) < PXYZ_2) {
    std::cerr << "目標位置が可動範囲外です" << std::endl;
    return false;
  }
  double theta3 = std::acos((PXYZ_2 - L2_2 - L3_2) / (2 * L2 * L3));
  double theta1 = std::atan2(PY, PX);

  const double C3 = std::cos(theta3);
  const double S3 = std::sin(theta3);
  const double L2_L3C3 = L2 + L3 * C3;
  const double L3S3 =  L3 * S3;
  const double SQRT_PXY = std::sqrt(PX_2 + PY_2);

  double theta2 = std::atan2(
    +L2_L3C3 * SQRT_PXY - L3S3 * PZ,
    +L3S3 * SQRT_PXY + L2_L3C3 * PZ);

  // θ2とθ3の符号を反転する
  theta2 *= -1.0;
  theta3 *= -1.0;

  q_list[2] = theta1;
  q_list[3] = theta2;
  q_list[5] = theta3;

  // 関節角度が可動範囲内にあるかチェック
  if (q_list_are_in_range(links, q_list) == false) {
    return false;
  }

  return true;
}

bool x7_3dof_picking_inverse_kinematics(const kinematics_utils::links_t & links,
                                        const Eigen::Vector3d & target_p,
                                        kinematics_utils::q_list_t & q_list) {
  // 手先先端の目標位置をもとに、CRANE-X7の第1, 2, 4, 6, 7関節の目標角度を計算する
  // 手先は常に地面（-Z）方向を向く

  // 目標位置Zに手先リンク長を加え、それを手首の目標位置とする
  const double HAND_LENGTH = 0.06;  // Ref: https://rt-net.jp/products/crane-x7/
  Eigen::Vector3d wrist_target_p = target_p;
  wrist_target_p[2] += links[8].b.z() + links[9].b.z() + HAND_LENGTH;

  if (x7_3dof_inverse_kinematics(links, wrist_target_p, q_list) == false) {
    return false;
  }

  // 第2, 4関節の大きさから、手先を下に向けるための第6関節の目標角度を求める
  q_list[7] = -M_PI - q_list[3] - q_list[5];

  // 手先姿勢を維持するため、第1関節と第7関節の目標角度を同じにする
  q_list[8] = q_list[2];

  // 関節角度が可動範囲内にあるかチェック
  if (q_list_are_in_range(links, q_list) == false) {
    return false;
  }

  return true;
}

bool s17_3dof_right_arm_inverse_kinematics(const kinematics_utils::links_t & links,
                                           const Eigen::Vector3d & target_p,
                                           kinematics_utils::q_list_t & q_list) {
  // 目標位置をもとに、Sciurus17の右腕の第1, 2, 4関節の目標角度を計算する

  // リンク相対位置から、関節間のリンク長を求める
  // 右腕の原点：P1 = Body + R_Link1 + R_Link2
  const Eigen::Vector3d P1 = links[2].b + links[5].b + links[6].b;
  const double L2 = std::fabs(links[7].b.y() + links[8].b.y());
  const double L3 = std::fabs(links[9].b.y() + links[10].b.y());
  const double L2_2 = std::pow(L2, 2);
  const double L3_2 = std::pow(L3, 2);

  // 順運動学の計算式
  // PX = P1 + -L3*(-C1*S3 + S1*S2*C3) - S1*S2*L2
  // PY = P1 + -C2*C3*L3 -C2*L2
  // PZ = P1 + -L3*(-S1*S3 + C1*S2*C3) - C1*S2*L2
  // 以下、PX'=PX-L1, PY'=PY-L1, PZ'=PZ-L1 として実装している
  const double PX = target_p.x() - P1.x();
  const double PY = target_p.y() - P1.y();
  const double PZ = target_p.z() - P1.z();
  const double PX_2 = std::pow(PX, 2);
  const double PY_2 = std::pow(PY, 2);
  const double PZ_2 = std::pow(PZ, 2);
  const double PXYZ_2 = PX_2 + PY_2 + PZ_2;

  // 目標位置が可動範囲内にあるかチェック
  if (std::pow(L2 - L3, 2) > PXYZ_2 || std::pow(L2 + L3, 2) < PXYZ_2) {
    std::cerr << "目標位置が可動範囲外です" << std::endl;
    return false;
  }
  const double theta3 = std::acos((PXYZ_2 - L2_2 - L3_2) / (2 * L2 * L3));

  const double C3 = std::cos(theta3);
  const double theta2 = -std::acos(-PY / (C3 * L3 + L2));

  const double S2 = std::sin(theta2);
  const double S3 = std::sin(theta3);
  const double A = S2 * C3 * L3 + S2 * L2;
  const double B = S3 * L3;
  const double theta1 = std::atan2(-A * PX + B * PZ,
                             B * PX + A * PZ);

  q_list[5] = theta1;
  q_list[6] = theta2;
  q_list[8] = theta3;

  // 関節角度が可動範囲内にあるかチェック
  if (q_list_are_in_range(links, q_list) == false) {
    return false;
  }
  return true;
}

bool s17_3dof_right_arm_picking_inverse_kinematics(const kinematics_utils::links_t & links,
                                                   const Eigen::Vector3d & target_p,
                                                   kinematics_utils::q_list_t & q_list) {
  // 手先先端の目標位置をもとに、Sciurus17の右腕の第1, 2, 4, 5, 6, 7関節の目標角度を計算する
  // 手先は常に地面（-Z）方向を向く

  // 目標位置Zに手先リンク長を加え、それを手首の目標位置とする
  Eigen::Vector3d wrist_target_p = target_p;
  wrist_target_p[2] += 0.103;  // Ref: https://rt-net.jp/products/sciurus17/

  if (s17_3dof_right_arm_inverse_kinematics(links, wrist_target_p, q_list) == false) {
    return false;
  }

  // 手先の目標姿勢を定める
  Eigen::Matrix3d wrist_target_R = kinematics_utils::rotation_from_euler_ZYX(0.0, 0.0, M_PI_2);

  // 手先姿勢の回転行列を求める
  auto calc_links = links;
  kinematics_utils::set_q_list(calc_links, q_list);
  kinematics::forward_kinematics(calc_links, 1);
  Eigen::Matrix3d wrist_R = calc_links[8].R;

  // 現在の手先姿勢から目標姿勢に遷移するための回転行列を求める
  const Eigen::Matrix3d diff_R = wrist_R.transpose() * wrist_target_R;
  const double R00 = diff_R(0, 0);
  const double R01 = diff_R(0, 1);
  const double R02 = diff_R(0, 2);
  const double R10 = diff_R(1, 0);
  const double R11 = diff_R(1, 1);
  const double R12 = diff_R(1, 2);
  const double R21 = diff_R(2, 1);
  const double R01_2 = std::pow(R01, 2);
  const double R21_2 = std::pow(R21, 2);

  // diff_R = R(-Y軸でtheta4回転) * R(+Z軸でtheta5回転) * R(-Y軸でtheta6回転)とし、
  // 行列要素から回転角度を求める

  double theta4 = 0;
  double theta5 = 0;
  double theta6 = 0;
  if (std::sqrt(R01_2 + R21_2) == 0) {
    // sin(theta5)が0のとき、
    // R11 (cos(theta5)) は±1になるため、次の式でtheta5, theta6を求める
    theta5 = M_PI / 2 - R11 * M_PI / 2;
    theta6 = atan2(-R02, R00) - R11 * theta4;
  } else {
    theta5 = std::atan2(-std::sqrt(R01_2 + R21_2), R11);
    theta4 = atan2(+R21, +R01);
    theta6 = atan2(+R12, -R10);
    q_list[9] = theta4;
    q_list[10] = theta5;
    q_list[11] = theta6;
    // 解が可動範囲外の場合、theta5の符号を反転した値をもとに解を求める
    if (q_list_are_in_range(links, q_list) == false) {
      theta5 = std::atan2(std::sqrt(R01_2 + R21_2), R11);
      theta4 = atan2(-R21, -R01);
      theta6 = atan2(-R12, +R10);
    }
  }
  q_list[9] = theta4;
  q_list[10] = theta5;
  q_list[11] = theta6;

  // 関節角度が可動範囲内にあるかチェック
  if (q_list_are_in_range(links, q_list) == false) {
    return false;
  }

  return true;
}

}  // namespace samples03
