# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Crab robot."""

from __future__ import annotations

import os

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg

##
# Configuration
##
from os import path

dirname = os.path.dirname(__file__)

CRAB_USD = path.join(dirname, "crab2.usd")
# CRAB_USD = "/home/kkona/Documents/research/drone_sim_lab/assets/temp/10joints_v3.usd"
SCALE = 3
CRAB_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=CRAB_USD,
        scale=[0.0012 * SCALE] * 3,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=10.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        copy_from_source=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, SCALE * 0.024),
        # rotate 90 degrees around the z-axis
        rot=(0.707107, 0.0, 0.0, 0.707107),
        joint_pos={
# ValueError: The following joints have default positions out of the limits:
#     - 'left_back_001_link_joint': 0.000 not in [0.175, 1.222]
#     - 'left_back_002_link_joint': 0.000 not in [0.175, 1.222]
#     - 'left_back_003_link_joint': 0.000 not in [0.175, 1.222]
#     - 'left_back_004_link_joint': 0.000 not in [0.175, 1.222]
#     - 'left_front_001_link_joint': 0.000 not in [0.175, 1.222]

            # ".*_joint": 0.2,
            # "front_left_foot": 0.785398,  # 45 degrees
            # "front_right_foot": -0.785398,
            # "left_back_foot": -0.785398,
            # "right_back_foot": 0.785398,
            'left_back_001_link_joint': 0.175,
            'left_back_002_link_joint': 0.175,
            'left_back_003_link_joint': 0.175,
            'left_back_004_link_joint': 0.175,
            'left_front_001_link_joint': 0.175,
        },
    ),
    actuators={
        "body": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=0.0,
            damping=0.0,
        ),
    },
)
"""Configuration for the Crab robot."""
