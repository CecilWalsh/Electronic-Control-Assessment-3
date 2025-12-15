#include "SerialExample.hpp"
#include <iostream>
#include <random>
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>



int main(int argc, char** argv) {
	std::string com_port;
	if (argc < 2) {
		com_port = "/dev/pts/15";
		std::cout << "No COM port specified. Using default: " << com_port << std::endl;
	}
	else {
		com_port = argv[1];
	}

	// 创建Gimbal对象
	io::Gimbal gimbal(com_port);
	std::cout << "Serial port opened: " << com_port << std::endl;

	// 随机数生成器
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> yaw_dist(-180.0f, 180.0f);
	std::uniform_real_distribution<float> pitch_dist(-45.0f, 45.0f);
	std::uniform_real_distribution<float> vel_dist(-50.0f, 50.0f);
	std::uniform_real_distribution<float> acc_dist(-100.0f, 100.0f);
	std::uniform_int_distribution<int> mode_dist(0, 2);

	std::cout << "\n=== Serial Communication Demo ===" << std::endl;
	std::cout << "Sending random data and receiving gimbal state..." << std::endl;
	std::cout << "Press Ctrl+C to exit\n" << std::endl;

	int count = 0;
	while (true) {
		// 生成随机数据
		bool control = (mode_dist(gen) > 0);
		bool fire = (mode_dist(gen) == 2);
		float yaw = yaw_dist(gen);
		float yaw_vel = vel_dist(gen);
		float yaw_acc = acc_dist(gen);
		float pitch = pitch_dist(gen);
		float pitch_vel = vel_dist(gen);
		float pitch_acc = acc_dist(gen);

		// 发送数据
		gimbal.send(control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc);
		
		// 显示发送的数据
		std::cout << "--- Packet #" << ++count << " ---" << std::endl;
		std::cout << "SENT:" << std::endl;
		std::cout << "  Control: " << (control ? "Yes" : "No") 
				  << ", Fire: " << (fire ? "Yes" : "No") << std::endl;
		std::cout << std::fixed << std::setprecision(2);
		std::cout << "  Yaw: " << yaw << " deg, Vel: " << yaw_vel 
				  << " deg/s, Acc: " << yaw_acc << " deg/s²" << std::endl;
		std::cout << "  Pitch: " << pitch << " deg, Vel: " << pitch_vel 
				  << " deg/s, Acc: " << pitch_acc << " deg/s²" << std::endl;

		// 读取并显示接收的云台状态
		auto mode = gimbal.mode();
		auto state = gimbal.state();
		std::cout << "RECEIVED:" << std::endl;
		std::cout << "  Mode: " << gimbal.str(mode) << std::endl;
		std::cout << "  Yaw: " << state.yaw << " deg, Vel: " << state.yaw_vel << " deg/s" << std::endl;
		std::cout << "  Pitch: " << state.pitch << " deg, Vel: " << state.pitch_vel << " deg/s" << std::endl;
		std::cout << "  Bullet Speed: " << state.bullet_speed << " m/s" << std::endl;
		std::cout << "  Bullet Count: " << state.bullet_count << std::endl;
		std::cout << std::endl;

		// 等待一段时间再发送下一个数据包
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	return 0;
}