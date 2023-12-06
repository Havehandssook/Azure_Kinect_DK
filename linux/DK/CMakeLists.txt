# $表示取出变量中的值
# CMAKE_CURRENT_SOURCE_DIR 这表示当前处理的CMakeLists.txt所在的目录
# link_libraries用在add_executable之前，target_link_libraries用在add_executable之后

#1.指定cmake版本 
cmake_minimum_required(VERSION 3.2)
 
#2.指定项目的名称
PROJECT(DK)

#查找到指定的包，并将它的路径存储在变量中	
find_package(OpenCV REQUIRED)
find_package(k4a REQUIRED)
#指定要链接的库文件的路径 
# link_directories(路径)

#3.设置包含的目录，用来提供找头文件路径 
#3.的具体解释https://blog.csdn.net/weixin_39393741/article/details/85070299
INCLUDE_DIRECTORIES(
./include 
#${OpenCV_INCLUDE_DIRS}
)

 
#4.搜索./src文件夹下的所有源文件并命名为DIR_SRCS
AUX_SOURCE_DIRECTORY(./src DIR_SRCS)
#明确指明包含哪些源文件
#add_library(项目名称 明确的源文件)

# c++11的多线程
find_package(Threads)

#5设置变量的值
SET(TEST_MATH ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})
SET(DEPENDENCIES realsense2)
# 链接多线程库和数学库
LINK_LIBRARIES(-lpthread -lm)

#6.生成可执行的文件
ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS})

#7.生成库
#add_library(库名 生成库的类型 依赖的源文件)

#8.添加可执行文件所需要的库，TARGET_LINK_LIBRARIES(项目名 项目所需要链接生成的库)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${TEST_MATH} ${DEPENDENCIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${TEST_MATH})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} k4a::k4a ${OpenCV_LIBS})
