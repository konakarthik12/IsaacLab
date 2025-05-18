# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.envs import ViewerCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.terrains import TerrainImporterCfg, TerrainGeneratorCfg, MeshRepeatedPyramidsTerrainCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR
from omni.isaac.lab_assets.crab import CRAB_CFG
from omni.isaac.lab_tasks.direct.ant.ant_env import AntEnvCfg
from .crab_terrain import ROUGH_TERRAINS_CFG
from omni.isaac.lab_tasks.direct.locomotion.locomotion_env import LocomotionEnv


@configclass
class CrabEnvCfg(AntEnvCfg):
    # env
    episode_length_s = 15.0
    decimation = 4

    # action_scale = 6e-4 # (works but way too fast)
    # action_scale = 2e-4
    action_scale = 3e-3
    action_space = 18
    observation_space = 66
    state_space = 0
    sim_cfg = SimulationCfg(dt=1 / 250, render_interval=4,
                            # device="cpu", use_fabric=False,
                            device="cuda:0", use_fabric=False
                            )
    sim_cfg.physx.gpu_max_rigid_patch_count = 5 * 2 ** 17
    # simulation
    sim: SimulationCfg = sim_cfg
    # terrain = TerrainImporterCfg(
    #     prim_path="/World/ground",
    #     terrain_type="plane",
    #     collision_group=-1,
    #     physics_material=sim_utils.RigidBodyMaterialCfg(
    #         friction_combine_mode="average",
    #         restitution_combine_mode="average",
    #         static_friction=1.0,
    #         dynamic_friction=1.0,
    #         restitution=0.0,
    #     ),
    #     debug_vis=False,
    # )
    # terrain = TerrainImporterCfg(
    #     prim_path="/World/ground",
    #     terrain_type="generator",
    #     terrain_generator=ROUGH_TERRAINS_CFG,
    #     physics_material=sim_utils.RigidBodyMaterialCfg(
    #         friction_combine_mode="multiply",
    #         restitution_combine_mode="multiply",
    #         static_friction=1.0,
    #         dynamic_friction=1.0,
    #     ),
    #     visual_material=sim_utils.MdlFileCfg(
    #         mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
    #         project_uvw=True,
    #         texture_scale=(0.25, 0.25),
    #     ),
    #     debug_vis=False,
    # )

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="usd",
        usd_path="/home/kkona/Documents/research/drone_sim_lab/assets/worlds/water/fluid_dynamics.usd",
        collision_group=-1,
        # physics_material=sim_utils.RigidBodyMaterialCfg(
        #     friction_combine_mode="average",
        #     restitution_combine_mode="average",
        #     static_friction=1.0,
        #     dynamic_friction=1.0,
        #     restitution=0.0,
        # ),
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
    # Kill the crab if it is upside down
    termination_up_proj: float = -0.5

    angular_velocity_scale: float = 1.0
    contact_force_scale: float = 0.1

    viewer = ViewerCfg(
        eye=(4, -2, 0.5),
        lookat=(0.0, 0.0, 0.0),
    )


class CrabEnv(LocomotionEnv):
    cfg: CrabEnvCfg

    def __init__(self, cfg: CrabEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
