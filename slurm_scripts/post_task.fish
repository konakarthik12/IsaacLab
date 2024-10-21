#!/usr/bin/env fish
pushd (dirname (status -f))
cd ..

echo "Ran task with args: $argv[2..-1]"

echo "Copying files back to dir: $argv[1]"
# python source/standalone/workflows/rsl_rl/train.py --task Isaac-Ant-Direct --headless
cp -r outputs/ logs/ $argv[1]
popd