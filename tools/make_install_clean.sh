#!/bin/bash
# !Warning! Do not run when you have uncommmited changes.
# TODO: Check if there is an uncommited changes.
# Compile, install the tinyos tools and clean the files.
./Bootstrap && ./configure && make && sudo make install && git clean -df && git checkout -- .
