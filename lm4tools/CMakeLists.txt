find_package(libusb-1.0 REQUIRED)

add_subdirectory(lm4flash)
add_subdirectory(lmicdiusb)

add_custom_target(lm4tools)
add_dependencies(lm4tools lm4flash lmicdi)
