cmake_minimum_required(VERSION 3.20)
project(practica2)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(music_arduino)

add_executable(practica2
        music_arduino/let_it_be.h
        music_arduino/let_it_be_1bit.h
        music_arduino/music_arduino.ino)
