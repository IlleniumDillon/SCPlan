#!/bin/bash

echo "Cleaning up..."

if [ -d "build" ]; then
     echo "Removing build directory..."
    rm -rf build
fi

if [ -d "dist" ]; then
    echo "Removing dist directory..."
    rm -rf dist
fi

if [ -d "install" ]; then
     echo "Removing install directory..."
    rm -rf install
fi

if [ -d "log" ]; then
    echo "Removing log directory..."
    rm -rf log
fi

echo "Cleanup complete."
 