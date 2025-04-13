#!/bin/bash

# build ROS package
colcon build

# write build-env.sh
last_commit_checksum=$(git rev-parse --short HEAD)
if [ `git status --porcelain=1 | wc -l` -ne 0 ]; then uncommitted_changes="*"; else uncommitted_changes=""; fi
printf -v time "%(%Y-%m-%d %H:%M)T"
author=$(git config --get user.name)
if [ -f build-env.sh ] ; then rm build-env.sh; fi
echo export CRAWLER_BUILD_METADATA=\"$last_commit_checksum$uncommitted_changes built at $time by $author\" >> build-env.sh
