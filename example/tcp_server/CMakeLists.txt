project(tcp_server)

message(STATUS "[example] 项目名称 ${PROJECT_NAME}")

file(GLOB_RECURSE SRCS *.cpp)
file(GLOB_RECURSE INCS *.hpp *.h)

add_executable(${PROJECT_NAME} ${SRCS} ${INCS})

# 设置编译器选项
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  target_compile_options(${PROJECT_NAME} PRIVATE /utf-8)
endif()
