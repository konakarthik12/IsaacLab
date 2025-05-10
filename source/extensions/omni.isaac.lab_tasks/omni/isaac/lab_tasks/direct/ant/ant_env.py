# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import numpy
from omni.isaac.lab.sensors import TiledCameraCfg
from omni.isaac.lab.utils.math import quat_from_euler_xyz
from omni.isaac.lab_assets.ant import ANT_CFG

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.envs import DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.direct.locomotion.locomotion_env import LocomotionEnv
from torch import tensor
from omni.isaac.core.utils.rotations import euler_angles_to_quat

DEG_TO_RAD = 3.141592653589793 / 180.0


def to_quat(x, y, z):
    x = tensor(x) * DEG_TO_RAD
    y = tensor(y) * DEG_TO_RAD
    z = tensor(z) * DEG_TO_RAD
    return

quat = euler_angles_to_quat(numpy.asarray([54, 0, 135]), degrees=True).tolist()

@configclass
class AntEnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 15.0
    decimation = 2
    action_scale = 0.5

    tiled_camera: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/envs/env_.*/Robot/torso/Camera",
        offset=TiledCameraCfg.OffsetCfg(pos=(3.5, 3.5, 3.5), rot=quat, convention="opengl"),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
        ),
        width=80,
        height=80,
    )
    action_space = 1
    state_space = 0
    observation_space = [tiled_camera.height, tiled_camera.width, 3]

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)
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
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=128, env_spacing=4.0, replicate_physics=True)

    # robot
    robot: ArticulationCfg = ANT_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    joint_gears: list = [15, 15, 15, 15, 15, 15, 15, 15]

    heading_weight: float = 0.5
    up_weight: float = 0.1

    energy_cost_scale: float = 0.05
    actions_cost_scale: float = 0.005
    alive_reward_scale: float = 0.5
    dof_vel_scale: float = 0.2

    death_cost: float = -2.0
    termination_height: float = 0.31

    angular_velocity_scale: float = 1.0
    contact_force_scale: float = 0.1


class AntEnv(LocomotionEnv):
    cfg: AntEnvCfg

    def __init__(self, cfg: AntEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
