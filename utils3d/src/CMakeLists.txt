file(GLOB SRC "./*.h" "./*.hpp" "./*.cpp")
add_library(${CMAKE_PROJECT_NAME} ${SRC})

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
