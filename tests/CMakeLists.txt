include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
FetchContent_MakeAvailable(googletest)

add_executable(sim_tests
    test_math.cpp
    test_quaternion.cpp
    test_integrator.cpp
    test_atmosphere.cpp
    test_gravity.cpp
    test_propulsion.cpp
    test_aero.cpp
)

target_link_libraries(sim_tests PRIVATE
    sim_core
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(sim_tests)