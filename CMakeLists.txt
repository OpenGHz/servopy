cmake_minimum_required(VERSION 3.20)
project(servo_py VERSION 0.3.0 LANGUAGES CXX)

option(SERVO_PY_BUILD_PYTHON "Build the Python extension" ON)
option(SERVO_PY_BUILD_TESTS "Build standalone C++ smoke test" OFF)

if(SERVO_PY_BUILD_PYTHON)
  find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
  # cmeel-eigen supplies headers during isolated Python builds. A system Eigen
  # installation also works for standalone C++ builds via CMAKE_PREFIX_PATH.
  execute_process(COMMAND "${Python_EXECUTABLE}" -c
    "import sys,pathlib; print(';'.join(str(pathlib.Path(p)/'cmeel.prefix') for p in sys.path if (pathlib.Path(p)/'cmeel.prefix').is_dir()))"
    OUTPUT_VARIABLE _cmeel_prefixes OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
  list(APPEND CMAKE_PREFIX_PATH ${_cmeel_prefixes})
  find_package(pybind11 CONFIG REQUIRED)
endif()
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_library(servo_core STATIC cpp/kinematics.cpp cpp/servo.cpp cpp/solvers.cpp)
add_library(servo_py::core ALIAS servo_core)
set_target_properties(servo_core PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME core)
target_compile_features(servo_core PUBLIC cxx_std_17)
target_link_libraries(servo_core PUBLIC Eigen3::Eigen)
target_include_directories(servo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(servo_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(SERVO_PY_BUILD_PYTHON)
  pybind11_add_module(_core MODULE NO_EXTRAS cpp/bindings.cpp)
  target_link_libraries(_core PRIVATE servo_core)
  install(TARGETS _core LIBRARY DESTINATION servo_py RUNTIME DESTINATION servo_py)
  # Keep one source for checkout examples and installed Python modules. Include
  # the URDFs, pinned model archive, provenance and licenses in every wheel.
  install(DIRECTORY examples/ DESTINATION servo_py/examples
    FILES_MATCHING PATTERN "*.py" PATTERN "*.urdf"
    PATTERN "*.zip" PATTERN "*.json" PATTERN "*.txt" PATTERN "*.md"
    PATTERN "__pycache__" EXCLUDE)
endif()

if(SERVO_PY_BUILD_TESTS)
  enable_testing()
  add_executable(servo_cpp_smoke tests/cpp_smoke.cpp)
  target_link_libraries(servo_cpp_smoke PRIVATE servo_core)
  add_test(NAME servo_cpp_smoke COMMAND servo_cpp_smoke)
endif()

if(NOT SKBUILD)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)
  install(TARGETS servo_core EXPORT servo_pyTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(EXPORT servo_pyTargets NAMESPACE servo_py:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/servo_py)
  configure_package_config_file(cmake/servo_pyConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/servo_pyConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/servo_py)
  write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/servo_pyConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/servo_pyConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/servo_pyConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/servo_py)
endif()
