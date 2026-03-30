cmake_minimum_required(VERSION 3.16)
project(drone_access_control VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# 依赖
# ------------------------------------------------------------------------------
find_package(PkgConfig QUIET)
find_package(OpenSSL QUIET)
find_package(MAVSDK QUIET)
find_package(nlohmann_json QUIET)
find_package(Boost 1.70 QUIET COMPONENTS system)
find_package(TBB QUIET)
find_package(OpenMP QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(ZMQ QUIET libzmq)
endif()

set(MIRACL_CORE_DIR ${CMAKE_SOURCE_DIR}/third_party/miracl-core/cpp)
set(MIRACL_CORE_LIB ${MIRACL_CORE_DIR}/core.a)
find_library(GMP_LIB gmp)
if(GMP_LIB)
    message(STATUS "GMP: ${GMP_LIB}")
else()
    message(WARNING "GMP not found, install libgmp-dev")
endif()

include_directories(${CMAKE_SOURCE_DIR}/include ${MIRACL_CORE_DIR})

# 后端核心源（访问控制、MAVLink、签名、飞行控制等，不含 main/http）
set(CORE_BACKEND_SOURCES
    src/communication/mavlink_manager.cpp
    src/communication/mavlink_manager_concurrent_runner.cpp
    src/state/drone_state_manager.cpp
    src/access_control/access_control_engine.cpp
    src/access_control/abac_policy_engine.cpp
    src/access_control/expression_evaluator.cpp
    src/access_control/access_decision_generator.cpp
    src/access_control/policy_config_loader.cpp
    src/authentication/authentication_provider.cpp
    src/authentication/authentication_provider_factory.cpp
    src/authentication/x509_certificate_provider.cpp
    src/attributes/attribute_provider.cpp
    src/attributes/attribute_aggregator.cpp
    src/attributes/environment_attribute_provider.cpp
    src/attributes/database_attribute_provider.cpp
    src/resources/airspace_resource_manager.cpp
    src/signature/geofence_signature_service.cpp
    src/signature/dss_ms_signature.cpp
    src/signature/mission_signature_verifier.cpp
    src/flight_control/flight_control_module.cpp
    src/flight_control/flight_plan.cpp
    src/error/error_recovery_manager.cpp
    src/error/security_violation_handler.cpp
)

# 为需要 Miracl+GMP+OpenSSL+JSON 的目标统一链接
macro(link_miracl_target TARGET)
    target_include_directories(${TARGET} PRIVATE ${CMAKE_SOURCE_DIR}/include)
    if(EXISTS ${MIRACL_CORE_LIB})
        target_link_libraries(${TARGET} ${MIRACL_CORE_LIB})
    endif()
    if(GMP_LIB)
        target_link_libraries(${TARGET} ${GMP_LIB})
    endif()
    if(OpenSSL_FOUND)
        target_link_libraries(${TARGET} OpenSSL::SSL OpenSSL::Crypto)
        target_compile_definitions(${TARGET} PRIVATE HAVE_OPENSSL)
    endif()
    if(nlohmann_json_FOUND)
        target_link_libraries(${TARGET} nlohmann_json::nlohmann_json)
        target_compile_definitions(${TARGET} PRIVATE HAVE_NLOHMANN_JSON)
    endif()
endmacro()

# ------------------------------------------------------------------------------
# 主库目标（轻量可执行，仅用于链接测试）
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/communication/mavlink_manager.cpp
    src/state/drone_state_manager.cpp
    src/access_control/access_control_engine.cpp
    src/access_control/abac_policy_engine.cpp
    src/access_control/expression_evaluator.cpp
    src/access_control/access_decision_generator.cpp
    src/access_control/policy_config_loader.cpp
    src/authentication/authentication_provider.cpp
    src/authentication/authentication_provider_factory.cpp
    src/authentication/x509_certificate_provider.cpp
    src/attributes/attribute_provider.cpp
    src/attributes/attribute_aggregator.cpp
    src/attributes/environment_attribute_provider.cpp
    src/attributes/database_attribute_provider.cpp
    src/resources/airspace_resource_manager.cpp
    src/signature/geofence_signature_service.cpp
    src/signature/dss_ms_signature.cpp
    src/signature/mission_signature_verifier.cpp
    src/flight_control/flight_control_module.cpp
    src/flight_control/flight_plan.cpp
    src/error/error_recovery_manager.cpp
    src/error/security_violation_handler.cpp
)
target_link_libraries(${PROJECT_NAME} pthread yaml-cpp)
if(EXISTS ${MIRACL_CORE_LIB})
    target_link_libraries(${PROJECT_NAME} ${MIRACL_CORE_LIB})
endif()
if(GMP_LIB)
    target_link_libraries(${PROJECT_NAME} ${GMP_LIB})
endif()
if(MAVSDK_FOUND)
    target_link_libraries(${PROJECT_NAME} MAVSDK::mavsdk)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_MAVSDK)
endif()
if(OpenSSL_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenSSL::SSL OpenSSL::Crypto)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_OPENSSL)
endif()
if(ZMQ_FOUND)
    target_link_libraries(${PROJECT_NAME} ${ZMQ_LIBRARIES})
    target_compile_options(${PROJECT_NAME} PRIVATE ${ZMQ_CFLAGS_OTHER})
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_ZMQ)
endif()
if(nlohmann_json_FOUND)
    target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_NLOHMANN_JSON)
endif()

# ------------------------------------------------------------------------------
# drone_grpc_server（唯一对外 C++ 服务，局域网发现 + gRPC）
# ------------------------------------------------------------------------------
find_package(Protobuf CONFIG QUIET)
find_package(gRPC CONFIG QUIET)
set(DRONE_GRPC_USE_CONFIG FALSE)
if(Protobuf_FOUND AND gRPC_FOUND AND TARGET protobuf::protoc AND TARGET gRPC::grpc_cpp_plugin)
    set(DRONE_GRPC_USE_CONFIG TRUE)
endif()
if(NOT DRONE_GRPC_USE_CONFIG AND PkgConfig_FOUND)
    if(NOT Protobuf_FOUND)
        find_package(Protobuf QUIET)
    endif()
    pkg_check_modules(GRPC QUIET grpc++ grpc)
    find_program(Grpc_CPP_Plugin NAMES grpc_cpp_plugin)
    if(Protobuf_FOUND AND Protobuf_PROTOC_EXECUTABLE AND GRPC_FOUND AND Grpc_CPP_Plugin)
        set(DRONE_GRPC_USE_PKGCONFIG TRUE)
    endif()
endif()

if(DRONE_GRPC_USE_CONFIG)
    set(PROTO_GEN_DIR "${CMAKE_BINARY_DIR}/gen")
    set(PROTO_FILE "${CMAKE_SOURCE_DIR}/proto/drone_control.proto")
    set(PROTO_PB_CC "${PROTO_GEN_DIR}/drone_control.pb.cc")
    set(PROTO_GRPC_CC "${PROTO_GEN_DIR}/drone_control.grpc.pb.cc")
    add_custom_command(OUTPUT "${PROTO_PB_CC}" "${PROTO_GEN_DIR}/drone_control.pb.h"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${PROTO_GEN_DIR}"
        COMMAND $<TARGET_FILE:protobuf::protoc> -I "${CMAKE_SOURCE_DIR}/proto" --cpp_out="${PROTO_GEN_DIR}" "${PROTO_FILE}"
        DEPENDS "${PROTO_FILE}" COMMENT "Generate C++ protobuf")
    add_custom_command(OUTPUT "${PROTO_GRPC_CC}" "${PROTO_GEN_DIR}/drone_control.grpc.pb.h"
        COMMAND $<TARGET_FILE:protobuf::protoc> -I "${CMAKE_SOURCE_DIR}/proto" --grpc_out="${PROTO_GEN_DIR}" --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin> "${PROTO_FILE}"
        DEPENDS "${PROTO_FILE}" "${PROTO_PB_CC}" COMMENT "Generate C++ gRPC")
    add_custom_target(proto_gen DEPENDS "${PROTO_PB_CC}" "${PROTO_GRPC_CC}")

    add_executable(drone_grpc_server
        src/grpc/grpc_server_main.cpp
        src/backend_init.cpp
        "${PROTO_PB_CC}" "${PROTO_GRPC_CC}"
        src/api/drone_api.cpp src/api/flight_api.cpp src/api/access_api.cpp src/api/response_utils.cpp
        ${CORE_BACKEND_SOURCES}
    )
    add_dependencies(drone_grpc_server proto_gen)
    target_include_directories(drone_grpc_server PRIVATE "${PROTO_GEN_DIR}" ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src ${MIRACL_CORE_DIR})
    target_compile_definitions(drone_grpc_server PRIVATE HAVE_GRPC HAVE_NLOHMANN_JSON)
    target_link_libraries(drone_grpc_server
        gRPC::grpc++ gRPC::grpc++_reflection protobuf::libprotobuf
        pthread yaml-cpp ${MIRACL_CORE_LIB} ${GMP_LIB}
        OpenSSL::SSL OpenSSL::Crypto nlohmann_json::nlohmann_json
    )
    if(Boost_FOUND)
        target_link_libraries(drone_grpc_server Boost::system)
        target_compile_definitions(drone_grpc_server PRIVATE USE_BOOST_ASIO USE_CONCURRENT_MAVLINK)
    endif()
    if(MAVSDK_FOUND)
        target_link_libraries(drone_grpc_server MAVSDK::mavsdk)
        target_compile_definitions(drone_grpc_server PRIVATE HAVE_MAVSDK)
    endif()
    install(TARGETS drone_grpc_server DESTINATION bin)
    message(STATUS "drone_grpc_server: enabled (CONFIG)")
elseif(DRONE_GRPC_USE_PKGCONFIG)
    set(PROTO_GEN_DIR "${CMAKE_BINARY_DIR}/gen")
    set(PROTO_FILE "${CMAKE_SOURCE_DIR}/proto/drone_control.proto")
    set(PROTO_PB_CC "${PROTO_GEN_DIR}/drone_control.pb.cc")
    set(PROTO_GRPC_CC "${PROTO_GEN_DIR}/drone_control.grpc.pb.cc")
    add_custom_command(OUTPUT "${PROTO_PB_CC}" "${PROTO_GEN_DIR}/drone_control.pb.h"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${PROTO_GEN_DIR}"
        COMMAND ${Protobuf_PROTOC_EXECUTABLE} -I "${CMAKE_SOURCE_DIR}/proto" --cpp_out="${PROTO_GEN_DIR}" "${PROTO_FILE}"
        DEPENDS "${PROTO_FILE}" COMMENT "Generate C++ protobuf (pkg-config)")
    add_custom_command(OUTPUT "${PROTO_GRPC_CC}" "${PROTO_GEN_DIR}/drone_control.grpc.pb.h"
        COMMAND ${Protobuf_PROTOC_EXECUTABLE} -I "${CMAKE_SOURCE_DIR}/proto" --grpc_out="${PROTO_GEN_DIR}" --plugin=protoc-gen-grpc=${Grpc_CPP_Plugin} "${PROTO_FILE}"
        DEPENDS "${PROTO_FILE}" "${PROTO_PB_CC}" COMMENT "Generate C++ gRPC (pkg-config)")
    add_custom_target(proto_gen_pc DEPENDS "${PROTO_PB_CC}" "${PROTO_GRPC_CC}")

    add_executable(drone_grpc_server
        src/grpc/grpc_server_main.cpp
        src/backend_init.cpp
        "${PROTO_PB_CC}" "${PROTO_GRPC_CC}"
        src/api/drone_api.cpp src/api/flight_api.cpp src/api/access_api.cpp src/api/response_utils.cpp
        ${CORE_BACKEND_SOURCES}
    )
    add_dependencies(drone_grpc_server proto_gen_pc)
    target_include_directories(drone_grpc_server PRIVATE "${PROTO_GEN_DIR}" ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src ${MIRACL_CORE_DIR} ${GRPC_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS})
    target_compile_definitions(drone_grpc_server PRIVATE HAVE_GRPC HAVE_NLOHMANN_JSON)
    target_link_libraries(drone_grpc_server
        ${GRPC_LIBRARIES} ${Protobuf_LIBRARIES}
        pthread yaml-cpp ${MIRACL_CORE_LIB} ${GMP_LIB}
        OpenSSL::SSL OpenSSL::Crypto nlohmann_json::nlohmann_json
    )
    if(Boost_FOUND)
        target_link_libraries(drone_grpc_server Boost::system)
        target_compile_definitions(drone_grpc_server PRIVATE USE_BOOST_ASIO USE_CONCURRENT_MAVLINK)
    endif()
    if(MAVSDK_FOUND)
        target_link_libraries(drone_grpc_server MAVSDK::mavsdk)
        target_compile_definitions(drone_grpc_server PRIVATE HAVE_MAVSDK)
    endif()
    install(TARGETS drone_grpc_server DESTINATION bin)
    message(STATUS "drone_grpc_server: enabled (pkg-config)")
else()
    message(STATUS "drone_grpc_server: skipped (need gRPC/Protobuf). Install: sudo apt install libgrpc++-dev protobuf-compiler-grpc libprotobuf-dev")
endif()

# ------------------------------------------------------------------------------
# 工具与测试可执行文件
# ------------------------------------------------------------------------------
add_executable(test_decrypt_ground tools/test_decrypt_ground.cpp src/signature/geofence_signature_service.cpp src/signature/dss_ms_signature.cpp)
target_link_libraries(test_decrypt_ground pthread yaml-cpp)
link_miracl_target(test_decrypt_ground)

add_executable(gen_ed25519_certs tools/gen_ed25519_certs.cpp)
target_link_libraries(gen_ed25519_certs pthread)
if(OpenSSL_FOUND)
    target_link_libraries(gen_ed25519_certs OpenSSL::SSL OpenSSL::Crypto)
    target_compile_definitions(gen_ed25519_certs PRIVATE HAVE_OPENSSL)
endif()

add_executable(ta_service tools/ta_dss_service/ta_main.cpp tools/ta_dss_service/ta_service.cpp src/signature/dss_ms_signature.cpp)
target_link_libraries(ta_service pthread)
link_miracl_target(ta_service)

add_executable(test_mission_signature_verifier tools/test_mission_signature_verifier.cpp src/signature/dss_ms_signature.cpp src/signature/mission_signature_verifier.cpp)
target_link_libraries(test_mission_signature_verifier pthread)
link_miracl_target(test_mission_signature_verifier)

add_executable(test_geofence_sanitizer tools/test_geofence_sanitizer.cpp src/signature/dss_ms_signature.cpp src/signature/geofence_signature_service.cpp)
target_link_libraries(test_geofence_sanitizer pthread)
link_miracl_target(test_geofence_sanitizer)

add_executable(test_fcsacm_complete_flow tools/test_fcsacm_complete_flow.cpp ${CORE_BACKEND_SOURCES})
target_link_libraries(test_fcsacm_complete_flow pthread yaml-cpp)
if(MAVSDK_FOUND)
    target_link_libraries(test_fcsacm_complete_flow MAVSDK::mavsdk)
    target_compile_definitions(test_fcsacm_complete_flow PRIVATE HAVE_MAVSDK)
endif()
link_miracl_target(test_fcsacm_complete_flow)

add_executable(test_fcsacm_star_complete_flow tools/test_fcsacm_star_complete_flow.cpp
    src/access_control/access_control_engine.cpp src/access_control/abac_policy_engine.cpp src/access_control/expression_evaluator.cpp src/access_control/access_decision_generator.cpp src/access_control/policy_config_loader.cpp
    src/authentication/authentication_provider.cpp src/authentication/authentication_provider_factory.cpp src/authentication/x509_certificate_provider.cpp
    src/attributes/attribute_provider.cpp src/attributes/attribute_aggregator.cpp src/attributes/environment_attribute_provider.cpp src/attributes/database_attribute_provider.cpp
    src/signature/geofence_signature_service.cpp src/signature/dss_ms_signature.cpp src/signature/mission_signature_verifier.cpp
    src/resources/airspace_resource_manager.cpp src/communication/mavlink_manager.cpp src/flight_control/flight_control_module.cpp src/flight_control/flight_plan.cpp
    src/state/drone_state_manager.cpp src/error/error_recovery_manager.cpp src/error/security_violation_handler.cpp
)
target_link_libraries(test_fcsacm_star_complete_flow pthread yaml-cpp)
if(MAVSDK_FOUND)
    target_link_libraries(test_fcsacm_star_complete_flow MAVSDK::mavsdk)
    target_compile_definitions(test_fcsacm_star_complete_flow PRIVATE HAVE_MAVSDK)
endif()
link_miracl_target(test_fcsacm_star_complete_flow)

add_executable(generate_px4_geofence_tokens tools/generate_px4_geofence_tokens.cpp src/signature/geofence_signature_service.cpp src/signature/dss_ms_signature.cpp)
target_link_libraries(generate_px4_geofence_tokens pthread)
link_miracl_target(generate_px4_geofence_tokens)

add_executable(generate_enhanced_baseline_token tools/generate_enhanced_baseline_token.cpp src/signature/geofence_signature_service.cpp src/signature/dss_ms_signature.cpp)
target_link_libraries(generate_enhanced_baseline_token pthread)
link_miracl_target(generate_enhanced_baseline_token)

add_executable(analyze_token_sizes tools/analyze_token_sizes.cpp src/signature/geofence_signature_service.cpp src/signature/dss_ms_signature.cpp)
target_link_libraries(analyze_token_sizes pthread)
link_miracl_target(analyze_token_sizes)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# 边缘设备程序
# ------------------------------------------------------------------------------
if(DRONE_GRPC_USE_CONFIG OR DRONE_GRPC_USE_PKGCONFIG)
    add_executable(edge_device_app
        src/edge/edge_device_main.cpp
        src/edge/edge_device_manager.cpp
        src/communication/mavlink_manager.cpp
        src/state/drone_state_manager.cpp
        src/access_control/access_control_engine.cpp
        src/access_control/abac_policy_engine.cpp
        src/access_control/expression_evaluator.cpp
        src/access_control/access_decision_generator.cpp
        src/access_control/policy_config_loader.cpp
        src/authentication/authentication_provider.cpp
        src/authentication/authentication_provider_factory.cpp
        src/authentication/x509_certificate_provider.cpp
        src/attributes/attribute_provider.cpp
        src/attributes/attribute_aggregator.cpp
        src/attributes/environment_attribute_provider.cpp
        src/attributes/database_attribute_provider.cpp
        src/resources/airspace_resource_manager.cpp
        src/signature/geofence_signature_service.cpp
        src/signature/dss_ms_signature.cpp
        src/signature/mission_signature_verifier.cpp
        src/flight_control/flight_control_module.cpp
        src/flight_control/flight_plan.cpp
        src/error/error_recovery_manager.cpp
        src/error/security_violation_handler.cpp
        "${PROTO_PB_CC}" "${PROTO_GRPC_CC}"
    )
    if(DRONE_GRPC_USE_CONFIG)
        add_dependencies(edge_device_app proto_gen)
        target_include_directories(edge_device_app PRIVATE "${PROTO_GEN_DIR}" ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src ${MIRACL_CORE_DIR})
        target_link_libraries(edge_device_app
            gRPC::grpc++ gRPC::grpc++_reflection protobuf::libprotobuf
            pthread yaml-cpp ${MIRACL_CORE_LIB} ${GMP_LIB}
            OpenSSL::SSL OpenSSL::Crypto nlohmann_json::nlohmann_json
        )
    else()
        add_dependencies(edge_device_app proto_gen_pc)
        target_include_directories(edge_device_app PRIVATE "${PROTO_GEN_DIR}" ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src ${MIRACL_CORE_DIR} ${GRPC_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS})
        target_link_libraries(edge_device_app
            ${GRPC_LIBRARIES} ${Protobuf_LIBRARIES}
            pthread yaml-cpp ${MIRACL_CORE_LIB} ${GMP_LIB}
            OpenSSL::SSL OpenSSL::Crypto nlohmann_json::nlohmann_json
        )
    endif()
    target_compile_definitions(edge_device_app PRIVATE HAVE_GRPC HAVE_NLOHMANN_JSON)
    if(Boost_FOUND)
        target_link_libraries(edge_device_app Boost::system)
        target_compile_definitions(edge_device_app PRIVATE USE_BOOST_ASIO USE_CONCURRENT_MAVLINK)
    endif()
    if(MAVSDK_FOUND)
        target_link_libraries(edge_device_app MAVSDK::mavsdk)
        target_compile_definitions(edge_device_app PRIVATE HAVE_MAVSDK)
    endif()
    install(TARGETS edge_device_app DESTINATION bin)
    message(STATUS "edge_device_app: enabled")
else()
    message(STATUS "edge_device_app: skipped (need gRPC/Protobuf)")
endif()

# ------------------------------------------------------------------------------
# 数据库服务程序
# ------------------------------------------------------------------------------
if(DRONE_GRPC_USE_CONFIG OR DRONE_GRPC_USE_PKGCONFIG)
    add_executable(database_service_app
        src/database/database_service_main.cpp
        src/database/database_manager.cpp
        "${PROTO_PB_CC}" "${PROTO_GRPC_CC}"
    )
    if(DRONE_GRPC_USE_CONFIG)
        add_dependencies(database_service_app proto_gen)
        target_include_directories(database_service_app PRIVATE "${PROTO_GEN_DIR}" ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src ${MIRACL_CORE_DIR})
        target_link_libraries(database_service_app
            gRPC::grpc++ gRPC::grpc++_reflection protobuf::libprotobuf
            pthread yaml-cpp ${MIRACL_CORE_LIB} ${GMP_LIB}
            OpenSSL::SSL OpenSSL::Crypto nlohmann_json::nlohmann_json
        )
    else()
        add_dependencies(database_service_app proto_gen_pc)
        target_include_directories(database_service_app PRIVATE "${PROTO_GEN_DIR}" ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src ${MIRACL_CORE_DIR} ${GRPC_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS})
        target_link_libraries(database_service_app
            ${GRPC_LIBRARIES} ${Protobuf_LIBRARIES}
            pthread yaml-cpp ${MIRACL_CORE_LIB} ${GMP_LIB}
            OpenSSL::SSL OpenSSL::Crypto nlohmann_json::nlohmann_json
        )
    endif()
    target_compile_definitions(database_service_app PRIVATE HAVE_GRPC HAVE_NLOHMANN_JSON)
    install(TARGETS database_service_app DESTINATION bin)
    message(STATUS "database_service_app: enabled")
else()
    message(STATUS "database_service_app: skipped (need gRPC/Protobuf)")
endif()

install(TARGETS ${PROJECT_NAME} ta_service gen_ed25519_certs test_decrypt_ground
    test_mission_signature_verifier test_geofence_sanitizer test_fcsacm_complete_flow test_fcsacm_star_complete_flow
    generate_px4_geofence_tokens generate_enhanced_baseline_token analyze_token_sizes
    DESTINATION bin)
