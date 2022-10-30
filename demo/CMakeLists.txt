cmake_minimum_required(VERSION 3.12)

######################################################

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
IF (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
ELSE ()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g")
ENDIF ()

######################################################

project(14_SimSampledNewGl)

add_definitions(-DPATH_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# glfw
set(CMAKE_PREFIX_PATH ../external/glfwlib)
find_package(glfw3 REQUIRED)

# opengl
find_package(OpenGL REQUIRED)

# eigen
set(CMAKE_PREFIX_PATH ../external/eigenlib)
find_package(Eigen3 REQUIRED)

# glad
set(GLAD_INCLUDE_DIR "../external/glad4/include")
get_filename_component(GLAD_INCLUDE_DIR ${GLAD_INCLUDE_DIR} ABSOLUTE)

set(CMAKE_PREFIX_PATH ../external/Imathlib)
FIND_PACKAGE(Imath)

set(CMAKE_PREFIX_PATH ../external/alembiclib)
FIND_PACKAGE(Alembic)

set(DELFEM2_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/delfem2/include)

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GLAD)
set(IMGUI_DIR "../external/imgui")

# ImGui-Addons
set(IMGUI_ADDONS_DIR "../external/ImGui-Addons/FileBrowser")

######################################################

include_directories(
    ${DELFEM2_INCLUDE_DIR}
    ${GLAD_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/pugixml/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../external
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMGUI_ADDONS_DIR}
)

add_executable(${PROJECT_NAME}
    main.cpp
    ${PROJECT_SOURCE_DIR}/../external/glad4/include/glad/glad.h
    ${PROJECT_SOURCE_DIR}/../external/glad4/src/glad.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/pugixml/src/pugixml.cpp
    ${IMGUI_DIR}/imgui.h
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.h
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.h
    ${IMGUI_ADDONS_DIR}/ImGuiFileBrowser.cpp
    )

target_link_libraries(${PROJECT_NAME}
    OpenGL::GL
    glfw
    Eigen3::Eigen
    Alembic::Alembic
    )