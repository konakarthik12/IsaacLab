#!/usr/bin/env fish
pushd (dirname (status -f))
cd ..

echo "Raw Run task raw: $argv[1]"
echo "with args: $argv[2..-1]"
set task $argv[1]
if test $task = "ant"
    echo "running ant task"
    python source/standalone/workflows/rsl_rl/train.py --task Isaac-Ant-Direct-v0 --headless $argv[2..-1]
else
    echo "Task not recognized: $task"
    exit 1
end
popd
