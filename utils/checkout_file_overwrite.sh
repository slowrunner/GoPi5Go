#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "USAGE: ./checkout_file_overwrite.sh <filename>"
  exit
fi
echo -e "git diff ${1}:"
git diff $1
read -p "Checkout ${1} from Git repo? [y|n] " choice
case "$choice" in 
  y|Y ) git checkout -- $1;;
  n|N ) echo "Ok, ${1} unchanged";;
  * ) echo "invalid answer, ${1} unchanged";;
esac
