cmake_minimum_required(VERSION 3.16)
project(rss1u_kinematics VERSION 1.0.0 LANGUAGES CXX)

if(MSVC)
    add_compile_options(/utf-8)
endif()

option(RSS1U_BUILD_EXAMPLES "Build the standalone example" ON)
option(RSS1U_BUILD_TESTS "Build regression tests" ON)
option(RSS1U_BUILD_STUDIO "构建 Qt 6/QML 运动学工作台" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(rss1u src/solver.cpp)
add_library(rss1u::rss1u ALIAS rss1u)
target_compile_features(rss1u PUBLIC cxx_std_17)
target_include_directories(rss1u
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(rss1u PUBLIC Eigen3::Eigen)

add_library(rss1u_studio_state apps/studio/interactive_state.cpp)
add_library(rss1u::studio_state ALIAS rss1u_studio_state)
target_compile_features(rss1u_studio_state PUBLIC cxx_std_17)
target_include_directories(rss1u_studio_state
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/apps/studio)
target_link_libraries(rss1u_studio_state PUBLIC rss1u::rss1u)

if(MSVC)
    target_compile_options(rss1u PRIVATE /W4 /permissive-)
    target_compile_options(rss1u_studio_state PRIVATE /W4 /permissive-)
else()
    target_compile_options(rss1u PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(rss1u_studio_state PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(RSS1U_BUILD_EXAMPLES)
    add_executable(rss1u_demo examples/basic_demo.cpp)
    target_link_libraries(rss1u_demo PRIVATE rss1u::rss1u)
    target_include_directories(rss1u_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples)
endif()

if(RSS1U_BUILD_TESTS)
    enable_testing()
    add_executable(rss1u_tests tests/test_solver.cpp)
    target_link_libraries(rss1u_tests PRIVATE rss1u::rss1u)
    target_include_directories(rss1u_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/examples
        ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME rss1u_tests COMMAND rss1u_tests)

    add_executable(rss1u_failure_tests tests/test_solver_failures.cpp)
    target_link_libraries(rss1u_failure_tests PRIVATE rss1u::rss1u)
    target_include_directories(rss1u_failure_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/examples
        ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME rss1u_failure_tests COMMAND rss1u_failure_tests)

    add_executable(rss1u_studio_state_tests tests/test_interactive_state.cpp)
    target_link_libraries(rss1u_studio_state_tests PRIVATE rss1u::studio_state)
    target_include_directories(rss1u_studio_state_tests
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples)
    add_test(NAME rss1u_studio_state_tests COMMAND rss1u_studio_state_tests)
endif()

if(RSS1U_BUILD_STUDIO)
    find_package(Qt6 6.5 QUIET COMPONENTS Quick QuickControls2 Quick3D)
    if(Qt6_FOUND)
        qt_standard_project_setup(REQUIRES 6.5)

        qt_add_executable(rss1u_studio WIN32 MANUAL_FINALIZATION
            apps/studio/main.cpp
            apps/studio/studio_state_manager.cpp
            apps/studio/studio_state_manager.hpp)
        target_compile_features(rss1u_studio PRIVATE cxx_std_17)
        target_include_directories(rss1u_studio PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/apps/studio
            ${CMAKE_CURRENT_SOURCE_DIR}/examples)
        target_link_libraries(rss1u_studio PRIVATE
            rss1u::studio_state
            Qt6::Quick
            Qt6::QuickControls2
            Qt6::Quick3D)

        set(RSS1U_STUDIO_QML_FILES
            apps/studio/qml/Main.qml
            apps/studio/qml/AngleControl.qml
            apps/studio/qml/AxisControl.qml
            apps/studio/qml/BranchEditor.qml
            apps/studio/qml/ConfigurationDialog.qml
            apps/studio/qml/Expression.js
            apps/studio/qml/ExpressionField.qml
            apps/studio/qml/MechanismView.qml
            apps/studio/qml/Rod3D.qml
            apps/studio/qml/Joint3D.qml)
        foreach(RSS1U_STUDIO_QML_FILE IN LISTS RSS1U_STUDIO_QML_FILES)
            get_filename_component(RSS1U_STUDIO_QML_NAME
                "${RSS1U_STUDIO_QML_FILE}" NAME)
            set_source_files_properties("${RSS1U_STUDIO_QML_FILE}"
                PROPERTIES QT_RESOURCE_ALIAS "${RSS1U_STUDIO_QML_NAME}")
        endforeach()

        qt_add_qml_module(rss1u_studio
            URI RSS1UStudio
            VERSION 1.0
            QML_FILES ${RSS1U_STUDIO_QML_FILES})

        set(RSS1U_STUDIO_CONFIGURATION_FILES
            configs/validation-I.json
            configs/validation-II.json
            configs/validation-III.json)
        foreach(RSS1U_STUDIO_CONFIGURATION_FILE IN LISTS
                RSS1U_STUDIO_CONFIGURATION_FILES)
            get_filename_component(RSS1U_STUDIO_CONFIGURATION_NAME
                "${RSS1U_STUDIO_CONFIGURATION_FILE}" NAME)
            set_source_files_properties("${RSS1U_STUDIO_CONFIGURATION_FILE}"
                PROPERTIES QT_RESOURCE_ALIAS
                    "configs/${RSS1U_STUDIO_CONFIGURATION_NAME}")
        endforeach()
        qt_add_resources(rss1u_studio "studio_configurations"
            PREFIX "/rss1u"
            FILES ${RSS1U_STUDIO_CONFIGURATION_FILES})

        qt_finalize_target(rss1u_studio)
    else()
        message(STATUS
            "未找到 Qt 6.5 Quick/QuickControls2/Quick3D；"
            "跳过 rss1u_studio 图形目标，核心状态测试仍可构建。")
    endif()
endif()

include(GNUInstallDirs)
install(TARGETS rss1u EXPORT rss1uTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT rss1uTargets
    FILE rss1uTargets.cmake
    NAMESPACE rss1u::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rss1u)

