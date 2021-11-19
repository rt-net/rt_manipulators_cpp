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

#include <fstream>
#include <iostream>
#include <string>

#include "kinematics_utils.hpp"

namespace kinematics_utils {

std::vector<std::string> split(const std::string & input, const char & delimiter) {
  // 1行の文字列をdelimiterで分割し、vectorに格納する
  std::istringstream istream(input);
  std::string str;
  std::vector<std::string> result;
  while(getline(istream, str, delimiter)){
      result.push_back(str);
  }
  return result;
}

std::vector<link::Link> parse_link_config_file(const std::string & file_path) {
  std::vector<link::Link> links;
  std::ifstream ifs(file_path);
  std::string str_buf;

  // 1行ずつ読み込み、str_bufに格納する
  int row_count=0;
  while(getline(ifs, str_buf)){
    row_count++;

    auto str_vec = split(str_buf, '\t');

    for(auto str : str_vec){
      std::cout<<str<<",";
    }
    std::cout<<std::endl;
  }

  return links;
}

}  // namespace kinematics_utils
