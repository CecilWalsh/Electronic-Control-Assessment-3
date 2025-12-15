cmake_minimum_required (VERSION 3.12)
project ("SerialExample")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

# 如果支持，请为 MSVC 编译器启用热重载。
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

add_subdirectory("serial")
# find_package(OpenCV REQUIRED)

# include_directories(${OpenCV_INCLUDE_DIRS})

set(SRCFILES
    "SerialExample.cpp" 
    "SerialExample.hpp" 
    "main.cpp" 
    "crc.hpp"
    "crc.cpp")

# 将源代码添加到此项目的可执行文件。
add_executable (SerialExample ${SRCFILES})

# target_link_libraries(SerialExample PRIVATE serial ${OpenCV_LIBS})
target_link_libraries(SerialExample PRIVATE serial)