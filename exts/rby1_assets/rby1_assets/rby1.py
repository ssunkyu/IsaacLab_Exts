# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the rby1 robots.

The following configurations are available:

* :obj:`RBY1_CFG`: Rby1 robot with Panda hand

Reference: 
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab_assets import ISAACLAB_ASSETS_DATA_DIR

##
# Configuration
##

RBY1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/RainbowRobotics/RB-Y1/RB-Y1.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "torso_0": 0.0,
            "torso_1": 0.0,
            "torso_2": 0.0,
            "torso_3": 0.0,
            "torso_4": 0.0,
            "torso_5": 0.0,
            "right_arm_0": 0.0,
            "right_arm_1": 0.0,
            "right_arm_2": 0.0,
            "right_arm_3": -1.57,
            "right_arm_4": 0.0,
            "right_arm_5": 0.0,
            "right_arm_6": 0.0,
            "gripper_finger_r.*": 0.04,
            "left_arm_0": 0.0,
            "left_arm_1": 0.0,
            "left_arm_2": 0.0,
            "left_arm_3": -1.57,
            "left_arm_4": 0.0,
            "left_arm_5": 0.0,
            "left_arm_6": 0.0,
            "right_wheel": 0.0,
            "left_wheel": 0.0,
            "gripper_finger_l.*": 0.04,
        },
    ),
    actuators={
        "torso": ImplicitActuatorCfg(
            joint_names_expr=["torso_.*"],
            effort_limit=1000.0,
            velocity_limit=1.0,
            stiffness=1e10,
            damping=100.0,
        ),
        "rby1_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3",
                              "left_arm_0", "left_arm_1", "left_arm_2", "left_arm_3"],
            effort_limit=1000.0,
            velocity_limit=2.175,
            stiffness=800.0,
            damping=200.0,
        ),
        "rby1_forearm": ImplicitActuatorCfg(
            joint_names_expr=["right_arm_4", "right_arm_5", "right_arm_6",
                              "left_arm_4", "left_arm_5", "left_arm_6"],
            effort_limit=1000.0,
            velocity_limit=2.61,
            stiffness=800.0,
            damping=200.0,
        ),
        "rby1_hand": ImplicitActuatorCfg(
            joint_names_expr=["gripper_finger_r1", "gripper_finger_r2",
                              "gripper_finger_l1", "gripper_finger_l2"],
            effort_limit=50.0,
            velocity_limit=1.0,
            stiffness=800,
            damping=400,
        ),
        "rby1_wheel": ImplicitActuatorCfg(
            joint_names_expr=["right_wheel", "left_wheel"],
            effort_limit=1000.0,
            velocity_limit=3.14159265,
            stiffness=2e3,
            damping=1e2
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Rby1 robot."""