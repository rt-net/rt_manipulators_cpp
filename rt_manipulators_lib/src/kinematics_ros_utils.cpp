// Copyright 2023 RT Corporation
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

#include "urdf_parser/urdf_parser.h"

#include "kinematics_ros_utils.hpp"
#include <iostream>

namespace kinematics_ros_utils {

kinematics_utils::links_t parse_urdf_file(const std::string & path) {
  std::vector<manipulators_link::Link> links;
  links.push_back(manipulators_link::Link());  // 0番目には空のリンクをセット

  auto parsed_urdf = urdf::parseURDFFile(path);

  manipulators_link::Link link;

  // Set root_link
  auto urdf_root_link = parsed_urdf->getRoot();
  link.name = urdf_root_link->name;
  links.push_back(link);

  for (const auto & child_link : urdf_root->child_links) {
    link.name = child_link->name;
    links.push_back(link);
  }

  return links;
}

}  // kinematics_ros_utils
