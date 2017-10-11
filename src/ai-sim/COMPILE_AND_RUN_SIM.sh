#!/bin/bash  
cd build
killall sim
g++ ../gui.cpp -o sim -lGL `sdl2-config --cflags --libs`
if [ $? -eq 0 ]; then
    echo COMPILED SUCCESS
    './sim'
else
    echo COMPILED ERROR
    $SHELL
fi

