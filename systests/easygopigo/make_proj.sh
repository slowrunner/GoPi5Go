#!/bin/bash

cmake -B build
pushd build
make
popd
echo -e "\nExecutables are in build/"
