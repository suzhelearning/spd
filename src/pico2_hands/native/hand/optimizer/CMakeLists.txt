cmake_minimum_required(VERSION 3.22)
project(tianji_hand_optimizer LANGUAGES CXX)

get_filename_component(SPD_WORKSPACE "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)
set(HAND_PREFIX "${SPD_WORKSPACE}/tools/wuji_hand_native/.pixi/envs/default")
list(PREPEND CMAKE_PREFIX_PATH "${HAND_PREFIX}")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${SPD_WORKSPACE}/.teleop/install" CACHE PATH "Native install prefix" FORCE)
endif()
# Pinocchio 4 and its Eigen ABI must never resolve through the arm environment.
set(CMAKE_BUILD_RPATH "$ORIGIN;${HAND_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;$ORIGIN/../../../tools/wuji_hand_native/.pixi/envs/default/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
add_link_options("-Wl,--disable-new-dtags")

# Use the Eigen ABI selected by the isolated Pinocchio 4 lock.
find_package(Eigen3 CONFIG REQUIRED)
find_package(pinocchio 4.0.0 EXACT REQUIRED)
find_library(NLOPT_LIBRARY nlopt REQUIRED)
find_path(NLOPT_INCLUDE_DIR nlopt.h REQUIRED)
get_filename_component(HAND_DEPENDENCY_LIB_DIR "${NLOPT_LIBRARY}" DIRECTORY)
add_library(tianji_hand_optimizer SHARED ../optimizer.cpp)
target_compile_features(tianji_hand_optimizer PRIVATE cxx_std_17)
target_compile_options(tianji_hand_optimizer PRIVATE -O3 -ffp-contract=off -Wall -Wextra)
target_include_directories(tianji_hand_optimizer PRIVATE ${NLOPT_INCLUDE_DIR})
# The pinned dependencies require the environment's newer libstdc++, not the
# system compiler's default runtime. Keep this local to these optional targets.
target_link_directories(tianji_hand_optimizer PRIVATE ${HAND_DEPENDENCY_LIB_DIR})
target_link_libraries(tianji_hand_optimizer PRIVATE pinocchio::pinocchio_default pinocchio::pinocchio_parsers ${NLOPT_LIBRARY})
add_executable(tianji_hand_native_worker ../worker.cpp ../lowpass.cpp ../geometry.cpp)
target_compile_features(tianji_hand_native_worker PRIVATE cxx_std_17)
target_compile_options(tianji_hand_native_worker PRIVATE -O3 -ffp-contract=off -Wall -Wextra -Wpedantic)
target_link_directories(tianji_hand_native_worker PRIVATE ${HAND_DEPENDENCY_LIB_DIR})
target_link_libraries(tianji_hand_native_worker PRIVATE tianji_hand_optimizer Eigen3::Eigen)
add_executable(tianji_hand_native_scheduler ../scheduler_main.cpp ../lowpass.cpp ../geometry.cpp)
target_compile_features(tianji_hand_native_scheduler PRIVATE cxx_std_17)
target_compile_options(tianji_hand_native_scheduler PRIVATE -O3 -ffp-contract=off -Wall -Wextra -Wpedantic)
target_link_directories(tianji_hand_native_scheduler PRIVATE ${HAND_DEPENDENCY_LIB_DIR})
target_link_libraries(tianji_hand_native_scheduler PRIVATE tianji_hand_optimizer Eigen3::Eigen)
install(TARGETS tianji_hand_optimizer tianji_hand_native_worker tianji_hand_native_scheduler
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES "${SPD_WORKSPACE}/tools/wuji_hand_native/third_party/wuji_hand_retargeting/LICENSE"
  DESTINATION share/licenses/wuji_hand_retargeting)
