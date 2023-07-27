add_library(visualizer visualizer.cpp)
target_link_libraries(visualizer params)
set_property(TARGET visualizer PROPERTY CXX_STANDARD 20)
target_include_directories(visualizer PRIVATE ${PYTHON_INCLUDE_DIRS} ${PYTHON_SITE_PACKAGES})
target_include_directories(visualizer PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )
install(TARGETS visualizer)