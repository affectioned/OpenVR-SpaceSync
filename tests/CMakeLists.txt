# Test binaries for SpaceSync. Built only when ENABLE_SPACESYNC_TESTS is on
# (default off so stock builds don't drag a test dependency in).
#
#   cmake --preset x64-release -DENABLE_SPACESYNC_TESTS=ON
#   cmake --build out\build\x64-release --target framelink_sim_test
#   ctest --test-dir out\build\x64-release --output-on-failure

add_executable(framelink_sim_test
    framelink_sim_test.cpp
)

target_include_directories(framelink_sim_test
    PRIVATE
        "${CMAKE_SOURCE_DIR}/include/shared"
        "${CMAKE_SOURCE_DIR}/include/driver"
        "${CMAKE_SOURCE_DIR}/src/driver"
)

if (MSVC)
    target_compile_options(framelink_sim_test PRIVATE /EHsc)
endif()

if (WIN32)
    target_link_libraries(framelink_sim_test PRIVATE Ws2_32)
endif()

set_target_properties(framelink_sim_test PROPERTIES CXX_STANDARD 20)

# One ctest per behaviour group would be nicer, but the sim is already
# fast (under a second) and prints per-check progress, so registering it
# as one test is enough — a failure's line number tells you which case.
add_test(NAME framelink_sim COMMAND framelink_sim_test)

# Python wire-protocol round trip. Skipped automatically if Python 3 isn't
# on PATH — don't want CI to fail just because of that.
find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_Interpreter_FOUND)
    add_test(
        NAME framelink_wire
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/framelink_wire_test.py"
    )
endif()
