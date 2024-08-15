FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/examples/IRsendDemoIDF/*.*)
FILE(GLOB_RECURSE lib_sources ${CMAKE_SOURCE_DIR}/src/*.*)
list(FILTER lib_sources EXCLUDE REGEX "IRtimer.*")

idf_component_register(SRCS ${app_sources} ${lib_sources}
                       INCLUDE_DIRS
                       "."
                       "../../src")
