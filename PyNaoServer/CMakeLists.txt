# CMake file for PyPepperServer

cmake_minimum_required(VERSION 2.8)
project(PyPepperServer)
find_package(qibuild)

add_definitions( -DUSE_ENCRYPTION )

qi_create_lib( pypepperserver
  SRC main.cpp
      PepperProxyManager.cpp
      PyPepperServer.cpp
      PyPepperModule.cpp
  SHARED
)

qi_use_lib(pypepperserver ALCOMMON ALVISION ALAUDIO pyridecore tinyxml crypto)
# qi_create_bin(...)

