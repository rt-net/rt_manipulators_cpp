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

#ifndef RT_MANIPULATORS_LIB_INCLUDE_KINEMATICS_UTILS_HPP_
#define RT_MANIPULATORS_LIB_INCLUDE_KINEMATICS_UTILS_HPP_

#include <eigen3/Eigen/Dense>
#include <map>
#include <string>
#include <vector>

#include "link.hpp"

namespace kinematics_utils {

using links_t = std::vector<manipulators_link::Link>;
using link_id_t = unsigned int;
using q_list_t = std::map<link_id_t, double>;

std::vector<manipulators_link::Link> parse_link_config_file(const std::string & file_path);
void print_links(const std::vector<manipulators_link::Link> & links, const int & start_id);
Eigen::Matrix3d skew_symmetric_matrix_for_cross_product(const Eigen::Vector3d & v);
Eigen::Matrix3d rodrigues(const Eigen::Vector3d & a, const double theta);
Eigen::Vector3d rotation_to_euler_ZYX(const Eigen::Matrix3d & mat);
Eigen::Matrix3d rotation_from_euler_ZYX(
  const double & z, const double & y, const double & x);
std::vector<link_id_t> find_route(const links_t & links, const link_id_t & target_id);
q_list_t get_q_list(const links_t & links, const std::vector<link_id_t> & id_list);
bool set_q_list(links_t & links, const q_list_t & q_list);
Eigen::Vector3d calc_error(const Eigen::Matrix3d & target, const Eigen::Matrix3d & current);
Eigen::MatrixXd calc_basic_jacobian(const links_t & links, const link_id_t & target_id);

}  // namespace kinematics_utils

#endif  // RT_MANIPULATORS_LIB_INCLUDE_KINEMATICS_UTILS_HPP_
