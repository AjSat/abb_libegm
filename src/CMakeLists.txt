#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#


orocos_component( EGMDriver EGMDriver.hpp EGMDriver.cpp) #

target_compile_definitions(EGMDriver PRIVATE -D_OS_UNIX)


target_link_libraries(EGMDriver ${catkin_LIBRARIES})

orocos_install_headers(EGMDriver.hpp)
