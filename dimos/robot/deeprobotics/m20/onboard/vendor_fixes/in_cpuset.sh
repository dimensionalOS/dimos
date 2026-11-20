#!/bin/bash
# Runs a command inside a cgroup-v1 cpuset (default cores 6-7), so the launch script's own taskset can only narrow it, never reach rl_deploy's cores 4-5.
# usage: in_cpuset.sh <cpus> <command...>
set -e
cpus=$1; shift
group=/sys/fs/cgroup/cpuset/vendor_cpus_${cpus//[^0-9]/_}
mkdir -p "$group"
cat /sys/fs/cgroup/cpuset/cpuset.mems > "$group/cpuset.mems"
echo "$cpus" > "$group/cpuset.cpus"
echo $$ > "$group/tasks"
exec "$@"
