#include "sdpo_ros_odom/utils.h"

#include <algorithm>
#include <stdexcept>

namespace sdpo_ros_odom {

std::vector<size_t> idx2valueVector(const std::vector<size_t> & vec_ini)
{
  auto min_value_it = std::min_element(vec_ini.begin(), vec_ini.end());
  auto max_value_it = std::max_element(vec_ini.begin(), vec_ini.end());

  if ((vec_ini.size() <= 1) || ((*min_value_it) == (*max_value_it))) {
    throw std::invalid_argument("idx2valueVector received an invalid vector");
  }

  if ((*min_value_it != 0) || (*max_value_it != vec_ini.size() - 1)) {
    throw std::invalid_argument("idx2valueVector expects a 0..n-1 permutation");
  }

  std::vector<size_t> vec_new(vec_ini.size());
  for (size_t i = 0; i < vec_ini.size(); ++i) {
    vec_new[i] = std::find(vec_ini.begin(), vec_ini.end(), i) - vec_ini.begin();
  }

  return vec_new;
}

}  // namespace sdpo_ros_odom
