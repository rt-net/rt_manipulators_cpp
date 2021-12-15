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

}  // namespace kinematics
