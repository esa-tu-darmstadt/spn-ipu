#!/usr/bin/env bash

cd /tmp

ORTOOLS_VERSION=v9.12

# Only the dev branch of OR Tools is not compatible with Gurobi 12.0.1
# right now (see or-tools commit #7fe44a2). Hopefully this will be fixed soon.
ORTOOLS_VERSION=7fe44a2f6bc4832d5a3ce8a7ac68805d9a5c2df7

git clone -b main https://github.com/google/or-tools
cd or-tools
git checkout ${ORTOOLS_VERSION}

# Configure
cmake -G Ninja -B build -DBUILD_DEPS=ON -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build

# Install
sudo cmake --install build

cd ..
rm -rf or-tools