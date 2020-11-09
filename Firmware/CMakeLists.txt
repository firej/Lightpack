cmake_minimum_required(VERSION 3.6)
project(Firmware)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        datatypes.h
        Descriptors.c
        Descriptors.h
        flags.h
        iodefs.h
        LedDriver.c
        LedDriver.h
        LedManager.c
        LedManager.h
        Lightpack.c
        Lightpack.h
        LightpackUSB.c
        LightpackUSB.h
        LUFAConfig.h
        version.h
        ws2811.h)

add_executable(Firmware ${SOURCE_FILES})