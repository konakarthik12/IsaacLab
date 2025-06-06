# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from omni.isaac.lab_assets.ant import ANT_CFG

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.envs import DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.direct.locomotion.locomotion_env import LocomotionEnv


@configclass
class AntEnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 15.0
    decimation = 4
    action_scale = 0.1
    action_space = 8
    observation_space = 36
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 250, render_interval=decimation,
                                       device="cpu", use_fabric=False,
                                       # device="cuda:0", use_fabric=True,
                                       )
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
    robot: ArticulationCfg = ANT_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    joint_gears: list = [15, 15, 15, 15, 15, 15, 15, 15]

    heading_weight: float = 0.5
    up_weight: float = 0.1

    energy_cost_scale: float = 0.05
    actions_cost_scale: float = 0.005 * 8
    alive_reward_scale: float = 0.5
    dof_vel_scale: float = 0.2

    death_cost: float = -2.0
    termination_height: float = 0.31 * 0.045

    angular_velocity_scale: float = 1.0
    contact_force_scale: float = 0.1


class AntEnv(LocomotionEnv):
    cfg: AntEnvCfg

    def __init__(self, cfg: AntEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
