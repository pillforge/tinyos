#!/bin/bash
# !Warning! Do not run when you have uncommmited changes.
# https://github.com/robbyrussell/oh-my-zsh/blob/e55c715508a2f652fed741f2047c66dda2c6e5b0/lib/git.zsh
# Compile, install the tinyos tools and clean the files.

if [[ $(git status -s) == "" ]]
then
  ./Bootstrap && ./configure && make && sudo make install && git clean -df && git checkout -- .
else
  echo "There are uncommited changes."
fi
