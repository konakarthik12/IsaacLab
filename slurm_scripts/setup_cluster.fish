#!/usr/bin/env fish
pushd (dirname (status -f))
cd ..

. /fs/nexus-projects/KGB-MBRL/scratch_hold/setup_conda.fish
. /fs/nexus-projects/KGB-MBRL/scratch_hold/setup_isaac_sim.fish
ln -s $ISAACSIM_PATH _isaac_sim
set isaac_env "isaac-sim-$SLURM_JOB_ID"

echo "Generating isaac lab conda environment: $isaac_env"
./isaaclab.sh --conda $isaac_env
conda activate $isaac_env

python -m pip install -U pip # pip outdated warnings
python -m pip install "usd-core<24.00" "lxml<5.0.0" "tqdm<5.0.0" # pip resolver warnings

./isaaclab.sh --install rsl-rl
popd
