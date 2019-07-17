#!/usr/bin/env bash
# this tool takes a number and a branch name as arguments e.g.: travinator.sh 10 fix/my_branch
# It will trigger 10 travis builds in the current repo by pushing empty commits to this branch
# to use this, you need to install the travis cli. (https://rubygems.org/gems/travis)
#  > sudo apt install ruby ruby-dev gem 
#  > gem install travis
# Note: check before whether the `travis status` command is working. 
# You will have to login first and you may need to set the repo 
# with `--store-repo` (https://www.rubydoc.info/gems/travis/1.8.10#status)

N=$1
echo "Executing $N builds ..."
BRANCH=$2
echo "For branch $BRANCH ..."

function get_current_state {
  output=$(travis show $BRANCH)
  grep "State: " <<< $output
}

for ((i = 1 ; i <= $N ; i++)); do
  echo "Build Nr $i ..."
  go=0
  while [ $go -eq 0 ]; do
    state=$(get_current_state)
    echo $state
    go=1
    if [[ $state == *"started"* ]]; then
      echo "started"
      go=0
    fi
    if [[ $state == *"created"* ]]; then
      echo "created"
      go=0
    fi
    sleep 1s
  done
  echo "build.."
  git commit --allow-empty -m"trigger travis $i"
  git push origin $BRANCH
  sleep 5s
done
