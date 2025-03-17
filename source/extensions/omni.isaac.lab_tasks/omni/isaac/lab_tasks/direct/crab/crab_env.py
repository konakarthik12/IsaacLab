# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from omni.isaac.lab_assets.crab import CRAB_CFG

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.envs import DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.direct.locomotion.locomotion_env import LocomotionEnv


@configclass
class CrabEnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 15.0
    decimation = 4
    # action_scale = 5e-5 (fast
    # action_scale = 6e-6 (slow, but might still be hard to catch)


    action_scale = 6e-6
    action_space = 18
    observation_space = 66
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 250, render_interval=decimation, device="cpu", use_fabric=False)
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=0.5, replicate_physics=True)

    # robot
    robot: ArticulationCfg = CRAB_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    joint_gears: list = [15] * 18

    heading_weight: float = 0.5
    up_weight: float = 0.1

    energy_cost_scale: float = 0.005
    actions_cost_scale: float = 0.005
    alive_reward_scale: float = 0.5
    dof_vel_scale: float = 0.2

    death_cost: float = -2.0
    termination_height: float = 0.0

    angular_velocity_scale: float = 1.0
    contact_force_scale: float = 0.1


class CrabEnv(LocomotionEnv):
    cfg: CrabEnvCfg

    def __init__(self, cfg: CrabEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
