

#添加文件
set(file_list 
	LModel.cpp
	LModel.h
	mystruct.h
	reeds_shepp.cpp
	reeds_shepp.h
	ssconfig.hpp
	cppModel_cppModel.h
	cppModel_cppModel.cpp
)

include_directories(../include)
find_package(Boost 1.65.0)

if(Boost_FOUND)
include_directories(${Boost_INCLUDE_DIRS})
message(${Boost_INCLUDE_DIRS})

endif()
#lib文件所存在的位置
set(PROJECT_LIB_DIR ${PROJECT_SOURCE_DIR}/src)

#输出包含文件信息
message(${file_list})
message(${PROJECT_BINARY_DIR})
message(${PROJECT_LIB_DIR})


#生成DLL
add_library(test_model SHARED ${file_list})
#add_library(test_pathplan  SHARED ${PROJECT_LIB_DIR}/${file_list})


#设置输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/b2)

#设置输出名称
set_target_properties(test_model PROPERTIES OUTPUT_NAME "libAnyModel")

if (UNIX)
    target_compile_options(pathplan PRIVATE -std=c++11)
    target_compile_options(test_pathplan PRIVATE -std=c++11)
    message(STATUS "Using -std=c++11")
endif()
