# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from omni.isaac.lab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg

from . import joint_pos_env_cfg

##
# Pre-defined configs
##
from rby1_assets.rby1 import RBY1_CFG  # isort: skip

@configclass
class Rby1CubeLiftEnvCfg(joint_pos_env_cfg.Rby1CubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Rby1 as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = RBY1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot",
                                            init_state=ArticulationCfg.InitialStateCfg(pos=[0.0, 0, 0.0], rot=[1.0, 0, 0, 0.0],
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
                                                                                            "gripper_finger_l.*": 0.04
                                                                                            }
                                                                                        ),                         
                                            )

        # Set actions for the specific robot type (Rby1)
        self.actions.right_arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3", "right_arm_4", "right_arm_5", "right_arm_6"],
            body_name="ee_right",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls", ik_params={"lambda_val":0.1}),
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, -0.17]),
            # debug_vis=True,
        )

        self.actions.left_arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["left_arm_0", "left_arm_1", "left_arm_2", "left_arm_3", "left_arm_4", "left_arm_5", "left_arm_6"],
            body_name="ee_left",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls", ik_params={"lambda_val":0.1}),
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, -0.17]),
        )


@configclass
class Rby1CubeLiftEnvCfg_PLAY(Rby1CubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
