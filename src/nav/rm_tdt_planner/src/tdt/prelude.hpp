// 编译期前置头：上游 yastar.cpp 用了 std::cout / std::any_of / std::ceil 却没 include。
// 这样做**不修改上游源码**。
#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <ranges>
#include <span>
