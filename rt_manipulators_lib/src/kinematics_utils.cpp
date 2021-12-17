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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "kinematics_utils.hpp"
#include "link.hpp"

namespace kinematics_utils {

std::vector<std::string> split(const std::string & input, const char & delimiter) {
  // 1行の文字列をdelimiterで分割し、vectorに格納する
  std::istringstream istream(input);
  std::string str;
  std::vector<std::string> result;
  while (getline(istream, str, delimiter)) {
      result.push_back(str);
  }
  return result;
}

std::vector<manipulators_link::Link> parse_link_config_file(const std::string & file_path) {
  // リンク情報リファレンスからダウンロードしたtscファイルを解析し
  // リンククラスのvectorを返す
  // CRANE-X7: https://docs.google.com/spreadsheets/d/1I268mnab4m-f6us0Au3AGd64-2iGkSwxaLrDplSjHY8/edit#gid=735472399
  // Sciurus17: https://docs.google.com/spreadsheets/d/1Q4z3M3cS1pQOEn3iXKLiIQIOr6czvECxSXEPS2-PGvA/edit#gid=1687288769

  const int COL_LINK_NAME = 0;
  const int COL_MY_LINK = 1;
  const int COL_SIBLING_LINK = 2;
  const int COL_CHILD_LINK = 3;
  const int COL_PARENT_LINK = 4;
  const int COL_RELATIVE_POS_X = 8;
  const int COL_RELATIVE_POS_Y = 9;
  const int COL_RELATIVE_POS_Z = 10;
  const int COL_MASS = 14;
  const int COL_CENTER_OF_MASS_X = 15;
  const int COL_CENTER_OF_MASS_Y = 16;
  const int COL_CENTER_OF_MASS_Z = 17;
  const int COL_INERTIA_XX = 18;
  const int COL_INERTIA_XY = 19;
  const int COL_INERTIA_YY = 20;
  const int COL_INERTIA_XZ = 21;
  const int COL_INERTIA_YZ = 22;
  const int COL_INERTIA_ZZ = 23;
  const int COL_AXIS_OF_ROTATION = 29;
  const int COL_DXL_ID = 32;
  const double MM_TO_METERS = 1e-3;
  const double MM2_TO_METERS2 = 1e-9;
  const double G_TO_KG = 1e-3;

  std::cout << "リンク情報ファイル:" << file_path << "を読み込みます" << std::endl;

  std::vector<manipulators_link::Link> links;
  links.push_back(manipulators_link::Link());  // 0番目には空のリンクをセット
  std::ifstream ifs(file_path);
  std::string str_line;

  // tsvファイルを1行ずつ読み込む
  while (getline(ifs, str_line)) {
    // 行をコンマで区切り、vectorに格納する
    auto str_vec = split(str_line, ',');

    // リンク番号を取得できなければスキップ
    try {
      auto link_number = std::stoi(str_vec[COL_MY_LINK]);
      (link_number);  // コンパイラの警告を消すための処理
    } catch (...) {
      continue;
    }

    manipulators_link::Link link;
    link.name = str_vec[COL_LINK_NAME];
    try {
        link.sibling = std::stoi(str_vec[COL_SIBLING_LINK]);
    } catch (...) {
    }

    try {
        link.child = std::stoi(str_vec[COL_CHILD_LINK]);
    } catch (...) {
    }

    try {
        link.parent = std::stoi(str_vec[COL_PARENT_LINK]);
    } catch (...) {
    }

    // 親リンクに対する相対位置
    link.b << std::stod(str_vec[COL_RELATIVE_POS_X]) * MM_TO_METERS,
              std::stod(str_vec[COL_RELATIVE_POS_Y]) * MM_TO_METERS,
              std::stod(str_vec[COL_RELATIVE_POS_Z]) * MM_TO_METERS;

    // 質量
    try {
      link.m = std::stod(str_vec[COL_MASS]) * G_TO_KG;
    } catch (...) {
    }

    // TODO(ShotaAk) 自リンクに対する重心位置の読み込みを実装する
    // try {
    //   link.c << std::stod(str_vec[COL_CENTER_OF_MASS_X]) * MM_TO_METERS,
    //             std::stod(str_vec[COL_CENTER_OF_MASS_Y]) * MM_TO_METERS,
    //             std::stod(str_vec[COL_CENTER_OF_MASS_Z]) * MM_TO_METERS;
    // } catch (...) {
    // }

    // TODO(ShotaAk) 自リンクに対する慣性テンソルの読み込みを実装する
    // try {
    //   link.I << std::stod(str_vec[COL_INERTIA_XX]) * MM2_TO_METERS2,
    //             std::stod(str_vec[COL_INERTIA_XY]) * MM2_TO_METERS2,
    //             std::stod(str_vec[COL_INERTIA_XZ]) * MM2_TO_METERS2,
    //             std::stod(str_vec[COL_INERTIA_XY]) * MM2_TO_METERS2,
    //             std::stod(str_vec[COL_INERTIA_YY]) * MM2_TO_METERS2,
    //             std::stod(str_vec[COL_INERTIA_YZ]) * MM2_TO_METERS2,
    //             std::stod(str_vec[COL_INERTIA_XZ]) * MM2_TO_METERS2,
    //             std::stod(str_vec[COL_INERTIA_YZ]) * MM2_TO_METERS2,
    //             std::stod(str_vec[COL_INERTIA_ZZ]) * MM2_TO_METERS2;
    // } catch (...) {
    // }

    // 親リンクに対する関節軸ベクトル
    // q=0のとき、ローカル座標系の姿勢をワールド座標系の姿勢に一致させるため、
    // 関節軸ベクトルに合わせて重心位置と慣性テンソルを座標変換させる
    std::string axis = str_vec[COL_AXIS_OF_ROTATION];
    // auto rot = rotation_from_euler(0, 0, 0);
    if (axis == "X+") {
      // rot = rotation_from_euler(0, M_PI_2, 0);
      link.a << 1, 0, 0;
    } else if (axis == "X-") {
      // rot = rotation_from_euler(0, -M_PI_2, 0);
      link.a << -1, 0, 0;
    } else if (axis == "Y+") {
      // rot = rotation_from_euler(-M_PI_2, 0, 0);
      link.a << 0, 1, 0;
    } else if (axis == "Y-") {
      // rot = rotation_from_euler(M_PI_2, 0, 0);
      link.a << 0, -1, 0;
    } else if (axis == "Z+") {
      link.a << 0, 0, 1;
    } else if (axis == "Z-") {
      // rot = rotation_from_euler(M_PI, 0, 0);
      link.a << 0, 0, -1;
    }
    // link.c = rot * link.c;
    // link.I = rot * link.I * rot.transpose();

    try {
      link.dxl_id = std::stoi(str_vec[COL_DXL_ID]);
    } catch (...) {
    }

    links.push_back(link);
  }

  return links;
}

void print_links(const std::vector<manipulators_link::Link> & links, const int & start_id) {
  // 指定されたリンクIDからchild、siblingに向かって逐次的にリンク情報を出力する
  auto link = links[start_id];
  int sibling_id = link.sibling;
  int child_id = link.child;

  std::cout << "リンクID:" << start_id << "リンク名:" << link.name << std::endl;
  std::cout << "親:" << link.parent << ", 子:" << link.child;
  std::cout << ", 姉妹兄弟:" << link.sibling << std::endl;
  std::cout << "親リンクに対する関節軸ベクトル a:" << std::endl;
  std::cout << link.a << std::endl;
  std::cout << "親リンクに対する相対位置 b:" << std::endl;
  std::cout << link.b << std::endl;
  std::cout << "ワールド座標系での位置 p:" << std::endl;
  std::cout << link.p <<std::endl;
  std::cout << "ワールド座標系での姿勢 R:" << std::endl;
  std::cout << link.R <<std::endl;
  std::cout << "質量 m:" << std::endl;
  std::cout << link.m <<std::endl;
  // std::cout << "自リンクに対する重心位置" << std::endl;
  // std::cout << link.c <<std::endl;
  // std::cout << "自リンクに対する慣性テンソル" << std::endl;
  // std::cout << link.I <<std::endl;
  std::cout << "---------------------------" << std::endl;

  if (sibling_id > 0) {
    print_links(links, sibling_id);
  }
  if (child_id > 0) {
    print_links(links, child_id);
  }
}

Eigen::Matrix3d skew_symmetric_matrix_for_cross_product(const Eigen::Vector3d & v) {
  // 3次元ベクトルから歪対称行列をつくる
  Eigen::Matrix3d m;
  m <<     0, -v.z(),  v.y(),
        v.z(),      0, -v.x(),
      -v.y(),  v.x(),       0;
  return m;
}

Eigen::Matrix3d rodrigues(const Eigen::Vector3d & a, const double theta) {
  // ロドリゲスの公式
  // 単位ベクトルa周りにthetaだけ回転する回転行列を返す
  // E + [a×] * sin(theta) + [a×]^2 * (1 - cos(theta))
  auto a_hat = skew_symmetric_matrix_for_cross_product(a);
  return Eigen::Matrix3d::Identity() + a_hat * std::sin(theta) +
    a_hat * a_hat * (1 - std::cos(theta));
}

Eigen::Vector3d rotation_to_euler_ZYX(const Eigen::Matrix3d & mat) {
  // 回転行列をZ軸、Y軸、X軸回りに回転したオイラー角に変換する
  return mat.eulerAngles(2, 1, 0);
}

Eigen::Matrix3d rotation_from_euler_ZYX(
  const double & z, const double & y, const double & x) {
  // Z-Y-Xオイラー角から、回転行列を生成する
  Eigen::Quaterniond q = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX());
  return q.matrix();
}

Eigen::Vector3d rotation_to_omega(const Eigen::Matrix3d & mat) {
  // 姿勢行列から等価各軸ベクトルへの変換
  // 参考：https://www.jstage.jst.go.jp/article/jrsj/29/3/29_3_269/_pdf/-char/ja
  auto l = Eigen::Vector3d(
    mat(2, 1) - mat(1, 2),
    mat(0, 2) - mat(2, 0),
    mat(1, 0) - mat(0, 1));
  auto l_norm = l.norm();

  if (l_norm > std::numeric_limits<double>::epsilon()) {
    // ゼロ除算を防ぐ条件式
    return (std::atan2(l_norm, mat.trace() - 1) / l_norm) * l;
  } else if (mat(0, 0) > 0 && mat(1, 1) > 0 && mat(2, 2) > 0) {
    // 行列の対角成分がすべて1のとき、ゼロベクトルを返す
    return Eigen::Vector3d::Zero();
  }
  // 行列の対角成分が(1, -1, -1), (-1, 1, -1), (-1, -1, 1)のときの処理
  return M_PI_2 * Eigen::Vector3d(mat(0, 0) + 1, mat(1, 1) + 1, mat(2, 2) + 1);
}

std::vector<link_id_t> find_route(const links_t & links, const link_id_t & target_id) {
  // 目標リンク(target_id)までの経路を抽出する
  // 返り値のベクトルの末尾がtarget_idになる
  std::vector<link_id_t> id_list;

  if (target_id <= 1 || target_id >= links.size()) {
    std::cerr << __func__ << ": 目標リンクIDには1より大きく、"
              << "linksのサイズより小さい数値をセットしてください" << std::endl;
    return id_list;
  }

  id_list.push_back(target_id);

  auto parent_id = links[target_id].parent;
  // ベースリンク(ID = 1)は含めない
  while (parent_id != 1) {
    id_list.push_back(parent_id);
    parent_id = links[parent_id].parent;
  }

  // 末尾をtarget_idにするため、ベクトルの並びを逆順にする
  std::reverse(id_list.begin(), id_list.end());
  return id_list;
}

q_list_t get_q_list(const links_t & links, const std::vector<link_id_t> & id_list) {
  // リンク構成から指定されたIDの関節位置qを抽出する
  q_list_t q_list;
  for (auto target_id : id_list) {
    if (target_id < links.size()) {
      q_list[target_id] = links[target_id].q;
    } else {
      std::cerr << __func__ << ": 無効なIDです:" << target_id << std::endl;
    }
  }
  return q_list;
}

bool set_q_list(links_t & links, const q_list_t & q_list, const bool & within_limit) {
  // リンク構成の指定されたIDに関節位置qを書き込む
  bool result = true;
  for (const auto & [target_id, q_value] : q_list) {
    if (target_id < links.size()) {
      if (within_limit) {
        links[target_id].set_q_within_limit(q_value);
      } else {
        links[target_id].q = q_value;
      }
    } else {
      std::cerr << __func__ << ": 無効なIDです:" << target_id << std::endl;
      result = false;
    }
  }
  return result;
}

Eigen::Vector3d calc_error_R(const Eigen::Matrix3d & target, const Eigen::Matrix3d & current) {
  // 回転行列の差を求める
  return rotation_to_omega(target * current.transpose());
}

Eigen::Vector3d calc_error_p(const Eigen::Vector3d & target, const Eigen::Vector3d & current) {
  // 位置の差を求める
  return target - current;
}

Eigen::VectorXd calc_error(
  const Eigen::Vector3d & target_p, const Eigen::Matrix3d & target_R,
  const manipulators_link::Link & current_link) {
  // 目標位置・姿勢と、リンクの現在位置・姿勢との差を求める
  auto error_p = calc_error_p(target_p, current_link.p);
  auto error_omega = calc_error_R(target_R, current_link.R);

  Eigen::VectorXd error(6);
  error << error_p, error_omega;
  return error;
}

Eigen::MatrixXd calc_basic_jacobian(const links_t & links, const link_id_t & target_id) {
  // 基礎ヤコビ行列を求める(各軸は回転関節)
  auto route = find_route(links, target_id);
  auto target_p = links[target_id].p;
  Eigen::MatrixXd J(6, route.size());
  J.setZero();

  for (int i=0; i < route.size(); i++) {
    auto link_id = route[i];
    auto a = links[link_id].R * links[link_id].a;
    J.col(i) << a.cross(target_p - links[link_id].p), a;
  }

  return J;
}

}  // namespace kinematics_utils
