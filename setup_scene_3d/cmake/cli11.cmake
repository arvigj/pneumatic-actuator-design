# CLI11 (https://github.com/CLIUtils/CLI11)
# BSD license

if(TARGET CLI11::CLI11)
    return()
endif()

message(STATUS "Third-party: creating target 'CLI11::CLI11'")

include(FetchContent)
FetchContent_Declare(
    CLI11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11
    GIT_TAG 6c7b07a878ad834957b98d0f9ce1dbe0cb204fc9
    GIT_SHALLOW FALSE
)
FetchContent_MakeAvailable(CLI11)

