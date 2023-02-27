# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/jaket/esp/esp-idf/components/bootloader/subproject"
  "C:/Users/jaket/OneDrive/Documents/ESP_FlightSoftware/timer_group/build/bootloader"
  "C:/Users/jaket/OneDrive/Documents/ESP_FlightSoftware/timer_group/build/bootloader-prefix"
  "C:/Users/jaket/OneDrive/Documents/ESP_FlightSoftware/timer_group/build/bootloader-prefix/tmp"
  "C:/Users/jaket/OneDrive/Documents/ESP_FlightSoftware/timer_group/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/jaket/OneDrive/Documents/ESP_FlightSoftware/timer_group/build/bootloader-prefix/src"
  "C:/Users/jaket/OneDrive/Documents/ESP_FlightSoftware/timer_group/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/jaket/OneDrive/Documents/ESP_FlightSoftware/timer_group/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
