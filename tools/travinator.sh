#!/usr/bin/env bash
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
