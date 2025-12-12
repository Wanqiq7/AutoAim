#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <cmath>

#include "io/gimbal/gimbal.hpp"
#include "io/command.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{delta-angle a  |          30         | yaw轴delta角 (单位:度)}"
  "{circle      c  |         0.2         | delta_angle的切片数}"
  "{signal-mode m  |     triangle_wave   | 发送信号的模式}"
  "{axis        x  |         yaw         | 发送信号的轴}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

// 这里的返回值直接理解为“度”
double yaw_cal(double t) {
  double A = 15; // 振幅改为30度，方便观察
  double T = 4;  // 周期 4 秒
  return A * std::sin(2 * M_PI * t / T); 
}

double pitch_cal(double t) {
  double A = 5;
  double T = 4;  
  return A * std::sin(2 * M_PI * t / T + M_PI / 2) + 18; 
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto delta_angle = cli.get<double>("delta-angle");
  auto circle = cli.get<double>("circle");
  auto signal_mode = cli.get<std::string>("signal-mode");
  auto axis = cli.get<std::string>("axis");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  io::Gimbal gimbal(config_path);

  auto init_angle = 0;
  double slice = circle * 100;
  auto dangle = delta_angle / slice;
  double cmd_angle = init_angle;
  int axis_index = axis == "yaw" ? 0 : 1;

  std::cout << "Waiting for gimbal to center (5s)..." << std::endl;
  auto start_wait = std::chrono::steady_clock::now();
  while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_wait).count() < 5) {
      if(exiter.exit()) return 0;
      gimbal.send(true, false, 0, 0, 0, 0, 0, 0); 
      std::this_thread::sleep_for(10ms); 
  }
  std::cout << "Test started! Sending DEGREE units." << std::endl;

  io::Command command{0};
  io::Command last_command{0};

  double t = 0;
  auto t0 = std::chrono::steady_clock::now();
  int count = 0;

  while (!exiter.exit()) {
    nlohmann::json data;
    auto timestamp = std::chrono::steady_clock::now();

    Eigen::Quaterniond q = gimbal.q(timestamp);
    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);

    // 计算当前云台的反馈角度（将弧度转为度，用于绘图对比）
    double gimbal_yaw_deg = eulers[0] * 57.29578;
    double gimbal_pitch_deg = eulers[1] * 57.29578;

    if (signal_mode == "triangle_wave") {
      if (count >= slice) {
        cmd_angle = init_angle;
        dangle = -dangle; 
        count = 0;
      }
      
      cmd_angle += dangle;
      
      // 【修改点 1】直接赋值，不再除以 57.3
      if (axis_index == 0) command.yaw = cmd_angle;
      else command.pitch = cmd_angle;
      count++;
      
      command.control = true;
      gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

      // 【修改点 2】绘图时 cmd_yaw 不再乘以 57.3，因为它已经是度了
      if (axis_index == 0) {
        data["cmd_yaw"] = command.yaw; 
        data["gimbal_yaw"] = gimbal_yaw_deg;
      } else {
        data["cmd_pitch"] = command.pitch;
        data["gimbal_pitch"] = gimbal_pitch_deg;
      }
      
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      plotter.plot(data);
      std::this_thread::sleep_for(10ms);
    }
    else if (signal_mode == "circle") { 
      t = tools::delta_time(std::chrono::steady_clock::now(), t0);
      
      // 【修改点 1】直接赋值，不再除以 57.3
      command.yaw = yaw_cal(t);
      command.pitch = pitch_cal(t);
      command.control = true;
      
      gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

      data["t"] = t;
      // 【修改点 2】绘图时直接使用 command.yaw
      data["cmd_yaw"] = command.yaw;
      data["cmd_pitch"] = command.pitch;
      // 反馈值依然需要从弧度转为度（因为 IMU 读出来永远是弧度）
      data["gimbal_yaw"] = gimbal_yaw_deg;
      data["gimbal_pitch"] = gimbal_pitch_deg;
      
      plotter.plot(data);
      std::this_thread::sleep_for(10ms);
    }
  }
  
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  return 0;
}