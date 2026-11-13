// 插件加载冒烟测试：不经 controller_server，直接用 pluginlib 实例化
#include <cstdio>
#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
    const std::string want = "rm_mpc_controller/MpcController";
    bool found = false;
    for (const auto& c : loader.getDeclaredClasses()) {
      if (c == want) found = true;
    }
    if (!found) {
      std::printf("✗ 未在 nav2_core 插件列表里找到 %s\n", want.c_str());
      rclcpp::shutdown();
      return 1;
    }
    auto inst = loader.createSharedInstance(want);
    std::printf("✓ 插件加载成功: %s (ptr=%p)\n", want.c_str(), (void*)inst.get());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::printf("✗ 加载异常: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
