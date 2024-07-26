if(TARGET igl::core)
    return()
endif()

option(LIBIGL_WITH_TRIANGLE "Compile libigl with triangle." ON)

# option(LIBIGL_COPYLEFT_CGAL "Build target igl_copyleft::cgal" ON)
include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.4.0
)
FetchContent_MakeAvailable(libigl)
