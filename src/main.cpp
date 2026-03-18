#include <iostream>
#include <memory>
#include <signal.h>
#include <thread>
#include <chrono>

#include "access_control/access_control_engine.hpp"
#include "common/types.hpp"

using namespace drone_control;

// 全局变量用于关闭
static bool should_exit = false;

void signal_handler(int signal) {
    std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
    should_exit = true;
}

int main(int argc, char* argv[]) {
    std::cout << "Drone Access Control Backend v1.0.0" << std::endl;
    std::cout << "Initializing system..." << std::endl;
    
    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    try {
        // 创建访问控制引擎
        auto access_engine = std::make_unique<AccessControlEngine>();
        
        std::cout << "Access Control Engine initialized successfully" << std::endl;
        std::cout << "System ready. Press Ctrl+C to exit." << std::endl;
        
        // 主循环
        while (!should_exit) {
            // 这里会添加MAVLink消息处理循环
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "System shutdown complete." << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}


