cmake_minimum_required(VERSION 3.24)
project(spd_teleop_native VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

get_filename_component(SPD_WORKSPACE "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(SPD_ARM_PREFIX "${SPD_WORKSPACE}/.pixi/envs/teleop-native")
list(PREPEND CMAKE_PREFIX_PATH "${SPD_ARM_PREFIX}")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${SPD_WORKSPACE}/.teleop/install" CACHE PATH "Native install prefix" FORCE)
endif()
# Isolate the arm ABI even when a Python launcher has activated another environment.
set(CMAKE_BUILD_RPATH "${SPD_ARM_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib;$ORIGIN/../../../.pixi/envs/teleop-native/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
add_link_options("-Wl,--disable-new-dtags")

# Keep the validated viewer API and Eigen/Pinocchio ABI separate from Hand2.
find_package(mujoco 3.10.0 EXACT CONFIG REQUIRED)
find_package(Eigen3 3.4 CONFIG REQUIRED NO_MODULE)
find_package(pinocchio 3 CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(Threads REQUIRED)
if(Eigen3_VERSION VERSION_GREATER_EQUAL 4 OR pinocchio_VERSION VERSION_GREATER_EQUAL 4)
  message(FATAL_ERROR "Arm teleop requires Eigen 3 and Pinocchio 3 from teleop-native")
endif()

# Pinned offline trajectory generator; no solver downloads or policy runtime.
add_subdirectory(third_party/ruckig)

function(tianji_enable_warnings target)
  target_compile_options(${target} PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wshadow>)
endfunction()

add_library(tianji_qp_ik STATIC
  src/ruckig_trajectory_limiter.cpp
  src/dls_pinocchio_arm_kinematics.cpp
  src/controller_dls.cpp
  src/franka_iterative_pose_dls.cpp
  src/pico_ee_franka_dls.cpp
  src/config.cpp
  src/controller.cpp
  src/interactive_marker.cpp
  src/joint_kinematics_plot.cpp
  src/joint_command.cpp
  src/mujoco_marker_adapter.cpp
  src/mujoco_joint_plot.cpp
  src/mujoco_robot.cpp
  src/pinocchio_arm_kinematics.cpp
  src/pico_teleop_protocol.cpp
  src/pico_skeleton_overlay.cpp
  src/pico_teleop_session.cpp
  src/pico_trace_recorder.cpp
  src/pico_udp_receiver.cpp
  src/safety.cpp
  src/shared_root_retarget.cpp
  src/shared_root_input.cpp
  src/shared_root_morphology.cpp
  src/shared_root_target_builder.cpp
  src/shared_root_closure.cpp
  src/shared_root_continuity.cpp
  src/shared_root_options.cpp
  src/shared_root_sha256.cpp
  src/shared_root_pipeline.cpp
  src/shared_root_guidance.cpp
  src/so3.cpp
  src/target_manager.cpp
  src/telemetry.cpp
  src/wuji_hand_teleop_protocol.cpp
  src/wuji_hand_udp_receiver.cpp)
target_include_directories(tianji_qp_ik PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(tianji_qp_ik PUBLIC
  Eigen3::Eigen yaml-cpp::yaml-cpp mujoco::mujoco pinocchio::pinocchio ruckig::ruckig)
tianji_enable_warnings(tianji_qp_ik)

# Private stdin/stdout absolute-TCP worker; no hardware/network authority.
add_executable(pico2_dls_worker apps/pico2_dls_worker.cpp)
target_link_libraries(pico2_dls_worker PRIVATE tianji_qp_ik)
tianji_enable_warnings(pico2_dls_worker)

add_executable(tianji_qp_ik_viewer apps/run_qp_ik_viewer.cpp)
target_link_libraries(tianji_qp_ik_viewer PRIVATE tianji_qp_ik glfw Threads::Threads)
tianji_enable_warnings(tianji_qp_ik_viewer)

install(TARGETS pico2_dls_worker tianji_qp_ik_viewer RUNTIME DESTINATION bin)
# Keep the profile-relative description paths valid in the installed tree too.
install(DIRECTORY config description DESTINATION share/teleop_native)
install(FILES third_party/ruckig/LICENSE DESTINATION share/licenses/ruckig)
