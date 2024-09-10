# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from omni.isaac.lab.utils import configclass

import omni.isaac.lab_tasks.manager_based.manipulation.reach.mdp as mdp
from rby1_exts.tasks.manipulation.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from rby1_assets import RBY1_CFG  # isort: skip


##
# Environment configuration
##


@configclass
class Rby1ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to Rby1
        self.scene.robot = RBY1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["ee_right"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["ee_right"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["ee_right"]

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["right_arm_.*"], scale=0.5, use_default_offset=True
        )
        # override command generator body
        # end-effector is along z-direction
        self.commands.ee_pose.body_name = "ee_right"
        self.commands.ee_pose.ranges.pitch = (math.pi, math.pi)


@configclass
class Rby1ReachEnvCfg_PLAY(Rby1ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
