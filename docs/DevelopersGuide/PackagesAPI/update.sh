#!/bin/bash

#find ../../../../ros/src/computing/perception/detection -name README.md | awk -F/ '{ print $NF }'


#exit

paths=(
  ../../../ros/src/sensing
  ../../../ros/src/computing/perception/detection
  ../../../ros/src/computing/perception/localization
  ../../../ros/src/computing/perception/prediction
  ../../../ros/src/computing/perception/semantics
  ../../../ros/src/computing/planning/motion
  ../../../ros/src/computing/planning/mission
  ../../../ros/src/computing/planning/decision
  ../../../ros/src/computing/planning/state
  ../../../ros/src/actuation
  ../../../ros/src/util
  ../../../ros/src/system
  ../../../ros/src/common
)

for path in ${paths[@]}; do
  dir=`echo $path | awk -F/ '{ print $NF }'`
  rm -rf $dir
  mkdir $dir
  find $path -name README.md | awk -F/ -v dir=$dir '{ print "../"$0, dir"/"$(NF-1)".md" }' | xargs -L1 ln -s
done
