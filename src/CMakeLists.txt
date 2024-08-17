# pdk-owb-slave - A OneWire slave implementation for Padauk microcontrollers.
# Copyright (C) 2024 David "Alemarius Nexus" Lerch
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.20)
project(pdk-owb-slave C)

set(CMAKE_C_FLAGS "--opt-code-size" CACHE STRING "")

set(PDK_ARCH "pdk13" CACHE STRING "PDK architecture used. This is used for -m{PDK_ARCH} when calling SDCC.")
set(PDK_DEVICE "PMS150C" CACHE STRING "PDK device used. Used as compiler flag -D{PDK_DEVICE} and for easypdkprog -n.")
set(PDK_TARGET_VDD_MV "3300" CACHE STRING "Target VDD voltage in millivolts.")

set(OWB_ROM_CODE "" CACHE STRING "1-Wire ROM code for the device. Only used for programming with easypdkprog.")

add_executable(${PROJECT_NAME} main.c owb.c)
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_SOURCE_DIR}/std")
target_compile_options(${PROJECT_NAME} PUBLIC "-m${PDK_ARCH}" "-D${PDK_DEVICE}" "-DTARGET_VDD_MV=${PDK_TARGET_VDD_MV}")
target_link_options(${PROJECT_NAME} PUBLIC "-m${PDK_ARCH}")

# Print statistics about the output file after build.
# Inspired by: https://github.com/free-pdk/free-pdk-examples/blob/master/BlinkLED/Makefile
add_custom_command (
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND makebin -p "$<TARGET_FILE:${PROJECT_NAME}>"
                "$<TARGET_FILE_PREFIX:${PROJECT_NAME}>$<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.bin"
        COMMAND ${CMAKE_COMMAND} -E echo "---------- Segments ----------"
        COMMAND grep "\\(ABS,CON\\)\\|\\(REL,CON\\)"
                "$<TARGET_FILE_PREFIX:${PROJECT_NAME}>$<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.map"
        COMMAND ${CMAKE_COMMAND} -E echo "-------- End Segments --------"
        COMMAND stat --printf "Size of $<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.bin: %s bytes\\n"
                "$<TARGET_FILE_PREFIX:${PROJECT_NAME}>$<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.bin"
        VERBATIM
        )

# Target for programming using easypdkprog
set(EASYPDKPROG_SERIAL_OPTS "")
if(OWB_ROM_CODE)
    set(EASYPDKPROG_SERIAL_OPTS -s "0x${OWB_ROM_CODE}")
endif()
add_custom_target (
        program
        COMMAND easypdkprog -n "${PDK_DEVICE}" ${EASYPDKPROG_SERIAL_OPTS} write "$<TARGET_FILE:${PROJECT_NAME}>"
        DEPENDS ${PROJECT_NAME}
        COMMENT "Writing $<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>$<TARGET_FILE_SUFFIX:${PROJECT_NAME}> to device using easypdkprog ..."
        VERBATIM
        )

# Target for erasing flash devices using easypdkprog
add_custom_target (
        erase
        COMMAND easypdkprog -n "${PDK_DEVICE}" erase
        DEPENDS ${PROJECT_NAME}
        COMMENT "Erasing device flash using easypdkprog ..."
        VERBATIM
        )
