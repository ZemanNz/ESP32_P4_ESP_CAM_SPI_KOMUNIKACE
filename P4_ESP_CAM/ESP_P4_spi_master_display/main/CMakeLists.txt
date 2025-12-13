idf_component_register(SRC_DIRS "."
                    INCLUDE_DIRS "." "include/"
                    REQUIRES "esp_mm" "esp_timer" "nvs_flash" esp32_p4_function_ev_board)

set(LV_DEMO_DIR ../managed_components/lvgl__lvgl/demos)
file(GLOB_RECURSE LV_DEMOS_SOURCES ${LV_DEMO_DIR}/*.c)

set_source_files_properties(
    "cv_detect.cpp"
    PROPERTIES
        COMPILE_OPTIONS
        "-Wno-deprecated-enum-enum-conversion"
    )
idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
#set(MODIFY_EXE ${PROJECT_DIR}/modify_opencv
#set(OPENCV_CMAKE_TARGET "${PROJECT_DIR}/managed_components/espressif__opencv/CMakeLists.txt")
#set(OPENCV_CMAKE_SOURCE "CMakeLists.txt.opencv")

#add_custom_command(
#    OUTPUT ${OPENCV_CMAKE_TARGET}
#    COMMENT "Updating OpenCV CMakeLists.txt using modify_opencv.py"
 #   COMMAND python ${MODIFY_EXE}
#    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#    DEPENDS ${OPENCV_CMAKE_SOURCE}
#    VERBATIM)

#add_custom_target(update_opencv ALL DEPENDS ${OPENCV_CMAKE_TARGET})
#add_dependencies(app update_opencv)
