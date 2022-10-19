#!/bin/bash
cd bin
rm -rf main 
cd ../build
rm -rf CMakeCache.txt  CMakeFiles  cmake_install.cmake  Makefile
cmake ..
make
cd ..
cd bin
echo "-----------------"
./main
cd ..