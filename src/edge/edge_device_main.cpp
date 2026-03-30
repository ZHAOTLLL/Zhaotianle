/**
 * 边缘设备主程序
 */
#include "edge/edge_device_manager.hpp"
#include "communication/mavlink_manager.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

using namespace drone_control;

int main(int argc, char* argv[]) {
    std::cout << "边缘设备服务启动..." << std::endl;

    // 解析命令行参数
    std::string device_id = "edge-001";
    std::string device_name = "Edge Device";
    std::string location = "Hangzhou";
    std::string core_service_address = "localhost:50051";
    std::string mavlink_listen_url = "udp://:14540";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--device-id" && i + 1 < argc) {
            device_id = argv[i + 1];
            i++;
        } else if (arg == "--device-name" && i + 1 < argc) {
            device_name = argv[i + 1];
            i++;
        } else if (arg == "--location" && i + 1 < argc) {
            location = argv[i + 1];
            i++;
        } else if (arg == "--core-address" && i + 1 < argc) {
            core_service_address = argv[i + 1];
            i++;
        } else if (arg == "--mavlink-url" && i + 1 < argc) {
            mavlink_listen_url = argv[i + 1];
            i++;
        }
    }

    // 初始化边缘设备管理器
    EdgeDeviceManager edge_manager;
    if (!edge_manager.initialize(device_id, device_name, location, core_service_address)) {
        std::cerr << " 边缘设备初始化失败" << std::endl;
        return 1;
    }

    // 初始化MAVLink管理器
    MAVLinkManager mavlink_manager;

    // 注册MAVLink回调
    mavlink_manager.registerConnectionCallback([&edge_manager](DroneId drone_id, MAVLinkManager::ConnectionStatus status) {
        if (status == MAVLinkManager::ConnectionStatus::CONNECTED) {
            edge_manager.registerDrone(drone_id);
            std::cout << "✓ 无人机 " << drone_id << " 连接成功" << std::endl;
        } else if (status == MAVLinkManager::ConnectionStatus::DISCONNECTED) {
            edge_manager.unregisterDrone(drone_id);
            std::cout << "✗ 无人机 " << drone_id << " 断开连接" << std::endl;
        }
    });

    mavlink_manager.registerTelemetryCallback([&edge_manager](const ExtendedDroneState& state) {
        edge_manager.updateDroneState(state.drone_id, state);
    });

    // 启动MAVLink发现
    if (!mavlink_manager.startDiscovery(mavlink_listen_url)) {
        std::cerr << " MAVLink发现启动失败" << std::endl;
        return 1;
    }

    std::cout << " MAVLink发现已启动，监听: " << mavlink_listen_url << std::endl;

    // 启动边缘设备管理器
    if (!edge_manager.connectToCoreService()) {
        std::cerr << " 连接核心服务失败" << std::endl;
        return 1;
    }

    std::cout << " 边缘设备服务启动成功" << std::endl;
    std::cout << " 设备信息: " << device_name << " (" << device_id << ")" << std::endl;
    std::cout << " 位置: " << location << std::endl;
    std::cout << " 核心服务: " << core_service_address << std::endl;

    // 主循环
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}