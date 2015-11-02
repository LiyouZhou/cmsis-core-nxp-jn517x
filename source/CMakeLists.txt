# mbed Microcontroller Library
# Copyright (c) 2014 ARM Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# this custom build file pulls in different startup assembly based on the
# toolchain being used

if(${CMAKE_C_COMPILER_ID} STREQUAL CR)
    set(STARTUP_SRC "boot_gcc/startup_jn517x.c")
elseif(${CMAKE_C_COMPILER_ID} STREQUAL GNU)
    set(STARTUP_SRC "boot_gcc/startup_jn517x.c")
else()
    set(STARTUP_ASM "")
    set(STARTUP_SRC "")
    message(ERROR "Unsupported target toolchain (gcc is supported)")
endif()

add_library(cmsis-core-nxp-jn517x
    ${STARTUP_SRC}
    #cmsis_nvic.c
)
set_target_properties(cmsis-core-nxp-jn517x PROPERTIES
    LINKER_LANGUAGE C
)

target_link_libraries(cmsis-core-nxp-jn517x
    cmsis-core
    cmsis-core-nxp
)
