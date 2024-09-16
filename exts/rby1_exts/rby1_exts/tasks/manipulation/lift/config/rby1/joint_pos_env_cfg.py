# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.assets import RigidObjectCfg
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

from rby1_exts.tasks.manipulation.lift import mdp
from rby1_exts.tasks.manipulation.lift.lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG  # isort: skip
from rby1_assets.rby1 import RBY1_CFG  # isort: skip


@configclass
class Rby1CubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Rby1 as robot
        self.scene.robot = RBY1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.actions.torso_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["torso_0", "torso_1", "torso_2",
                                             "torso_3", "torso_4", "torso_5"], scale=0.1, use_default_offset=False
        )

        # Set actions for the specific robot type (Rby1)
        self.actions.right_arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3",
                                             "right_arm_4", "right_arm_5", "right_arm_6"], scale=0.5, use_default_offset=False
        )
        self.actions.right_gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["gripper_finger_r.*"],
            open_command_expr={"gripper_finger_r.*": 0.04},
            close_command_expr={"gripper_finger_r.*": 0.0},
        )
        self.actions.left_arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["left_arm_0", "left_arm_1", "left_arm_2", "left_arm_3",
                                             "left_arm_4", "left_arm_5", "left_arm_6"], scale=0.5, use_default_offset=False
        )
        self.actions.left_gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["gripper_finger_l.*"],
            open_command_expr={"gripper_finger_l.*": 0.04},
            close_command_expr={"gripper_finger_l.*": 0.0},
        )
        self.actions.wheel_action = mdp.JointVelocityActionCfg(
            asset_name="robot",
            joint_names=["right_wheel", "left_wheel"], scale=1.0, use_default_offset=True
        )
        # Set the body name for the end effector
        self.commands.right_object_pose.body_name = "ee_right"
        self.commands.left_object_pose.body_name = "ee_left"

        # Set Cube as object
        self.scene.right_object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/right_Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.50, -0.15, 1.05], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )
        self.scene.left_object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/left_Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.50, 0.15, 1.05], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.right_ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/ee_right",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, -0.17],
                    ),
                ),
            ],
        )
        self.scene.left_ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/ee_left",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, -0.17],
                    ),
                ),
            ],
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
