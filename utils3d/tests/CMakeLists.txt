include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/6910c9d9165801d8827d628cb72eb7ea9dd538c5.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(TESTS_EXECUTABLE ${CMAKE_PROJECT_NAME}_tests)

file(GLOB SRC "./*.h" "./*.hpp" "./*.cpp")
add_executable(${TESTS_EXECUTABLE} ${SRC})

target_link_libraries(
  ${TESTS_EXECUTABLE}
  GTest::gmock
  utils3d
)

include(GoogleTest)
gtest_discover_tests(${TESTS_EXECUTABLE})
