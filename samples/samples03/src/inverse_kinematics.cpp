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
#include "inverse_kinematics.hpp"

namespace samples03 {

bool x7_3dof_inverse_kinematics(const kinematics_utils::links_t & links,
                                Eigen::Vector3d & target_p,
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

  q_list[links[2].dxl_id] = theta1;
  q_list[links[3].dxl_id] = theta2;
  q_list[links[5].dxl_id] = theta3;

  // 関節角度が可動範囲内にあるかチェック
  for (const auto & link : links) {
    const double dxl_id = link.dxl_id;
    const double q = q_list[dxl_id];
    if (q < link.min_q || q > link.max_q) {
      std::cerr << "ID:" << dxl_id << " の目標角度:" << q << " が可動範囲外です" << std::endl;
      return false;
    }
  }

  return true;
}

}  // namespace samples03
