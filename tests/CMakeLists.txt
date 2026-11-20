# Test binaries for SpaceSync. Built only when ENABLE_SPACESYNC_TESTS is on
# (default off so stock builds don't drag a test dependency in).
#
#   cmake --preset x64-release -DENABLE_SPACESYNC_TESTS=ON
#   cmake --build out\build\x64-release --target framelink_sim_test
#   ctest --test-dir out\build\x64-release --output-on-failure

function(spacesync_framelink_test name)
    add_executable(${name} ${name}.cpp)
    target_include_directories(${name}
        PRIVATE
            "${CMAKE_SOURCE_DIR}/include/shared"
            "${CMAKE_SOURCE_DIR}/include/driver"
            "${CMAKE_SOURCE_DIR}/src/driver"
    )
    if (MSVC)
        target_compile_options(${name} PRIVATE /EHsc)
    endif()
    if (WIN32)
        target_link_libraries(${name} PRIVATE Ws2_32)
    endif()
    set_target_properties(${name} PROPERTIES CXX_STANDARD 20)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

spacesync_framelink_test(framelink_sim_test)
spacesync_framelink_test(framelink_background_test)

# Pure-math calibration tests (CalibrationMath.h). Separate from the
# framelink helper because it needs the overlay include path + Eigen,
# not the driver include path or Winsock.
add_executable(calibration_solver_test calibration_solver_test.cpp)
target_include_directories(calibration_solver_test
    PRIVATE
        "${CMAKE_SOURCE_DIR}/include/overlay"
        "${CMAKE_SOURCE_DIR}/3rdparty/Eigen"
)
target_compile_definitions(calibration_solver_test
    PRIVATE
        _HAS_DEPRECATED_NEGATORS=1
        _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
if (MSVC)
    target_compile_options(calibration_solver_test PRIVATE /EHsc /bigobj)
endif()
set_target_properties(calibration_solver_test PROPERTIES CXX_STANDARD 20)
add_test(NAME calibration_solver_test COMMAND calibration_solver_test)

# Python wire-protocol round trip. Skipped automatically if Python 3 isn't
# on PATH — don't want CI to fail just because of that.
find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_Interpreter_FOUND)
    add_test(
        NAME framelink_wire
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/framelink_wire_test.py"
    )
endif()
