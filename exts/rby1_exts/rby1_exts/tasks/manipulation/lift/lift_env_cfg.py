# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import CurriculumTermCfg as CurrTerm
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from omni.isaac.lab.sim.spawners.shapes import CuboidCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.sensors import CameraCfg, TiledCameraCfg
from omni.isaac.lab.envs.mdp.observations import grab_images

from . import mdp

##
# Scene definition
##


@configclass
class ObjectTableSceneCfg(InteractiveSceneCfg):
    """Configuration for the lift scene with a robot and a object.
    This is the abstract base implementation, the exact scene is defined in the derived classes
    which need to set the target object, robot and end-effector frames
    """

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING
    # end-effector sensor: will be populated by agent env cfg
    right_ee_frame: FrameTransformerCfg = MISSING
    left_ee_frame: FrameTransformerCfg = MISSING

    right_ee_camera_frame: FrameTransformerCfg = MISSING
    left_ee_camera_frame: FrameTransformerCfg = MISSING

    # right_object_frame: FrameTransformerCfg = MISSING
    # left_object_frame: FrameTransformerCfg = MISSING

    # target object: will be populated by agent env cfg
    right_object: RigidObjectCfg = MISSING
    left_object: RigidObjectCfg = MISSING

    # Table
    # table = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Table",
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0.75, 0, 0.95], rot=[1.0, 0, 0, 0]),
    #     spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    # )
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.6, 0.3, 0.80], rot=[0.707, 0, 0, -0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/ThorlabsTable/table_instanceable.usd"),
    )
    # table = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Table",
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0.65, 0.3, 0.75], rot=[1.0, 0, 0, 0]),
    #     spawn=CuboidCfg(1.0, 1.6, 0.6),
    # )

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0]),
        spawn=GroundPlaneCfg(),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # camera
    right_arm_camera: TiledCameraCfg = MISSING
    left_arm_camera: TiledCameraCfg = MISSING 

##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    right_object_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=MISSING,  # will be set by agent env cfg
        resampling_time_range=(10.0, 10.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.4, 0.6), pos_y=(-0.20, -0.0), pos_z=(0.9, 1.0), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
        ),
    )
    left_object_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=MISSING,  # will be set by agent env cfg
        resampling_time_range=(10.0, 10.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.4, 0.6), pos_y=(0.0, 0.2), pos_z=(0.9, 1.0), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # will be set by agent env cfg
    torso_action: mdp.JointPositionActionCfg = MISSING
    right_arm_action: mdp.JointPositionActionCfg = MISSING
    right_gripper_action: mdp.BinaryJointPositionActionCfg = MISSING
    left_arm_action: mdp.JointPositionActionCfg = MISSING
    left_gripper_action: mdp.BinaryJointPositionActionCfg = MISSING
    wheel_action: mdp.JointVelocityActionCfg = MISSING


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        right_object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame, params={"object_cfg": SceneEntityCfg("right_object")})
        left_object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame, params={"object_cfg": SceneEntityCfg("left_object")})
        right_target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "right_object_pose"})
        left_target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "left_object_pose"})
        # image = ObsTerm(func=mdp.grab_images, params={"sensor_cfg": SceneEntityCfg("camera"), "data_type": "rgb"})
        actions = ObsTerm(func=mdp.last_action)
              
        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_right_object_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.2, -0.05), "y": (-0.15, 0.05), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("right_object", body_names="right_Object"),
        },
    )

    reset_left_object_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.2, -0.05), "y": (-0.05, 0.15), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("left_object", body_names="left_Object"),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    right_reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1, "object_cfg": SceneEntityCfg("right_object"),
                                                                         "ee_frame_cfg": SceneEntityCfg("left_ee_frame")}, weight=1.0)
    left_reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1, "object_cfg": SceneEntityCfg("left_object"),
                                                                        "ee_frame_cfg": SceneEntityCfg("right_ee_frame")}, weight=1.0)

    right_lifting_object = RewTerm(func=mdp.object_is_lifted, params={"minimal_height": 0.84,
                                                                      "object_cfg": SceneEntityCfg("right_object")}, weight=15.0)
    left_lifting_object = RewTerm(func=mdp.object_is_lifted, params={"minimal_height": 0.84,
                                                                     "object_cfg": SceneEntityCfg("left_object")}, weight=15.0)

    right_object_goal_tracking = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.3, "minimal_height": 0.04, "command_name": "right_object_pose", "object_cfg": SceneEntityCfg("right_object")},
        weight=16.0,
    )

    left_object_goal_tracking = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.3, "minimal_height": 0.04, "command_name": "left_object_pose", "object_cfg": SceneEntityCfg("left_object")},
        weight=16.0,
    )

    # right_object_goal_tracking_fine_grained = RewTerm(
    #     func=mdp.object_goal_distance,
    #     params={"std": 0.05, "minimal_height": 0.04, "command_name": "right_object_pose", "object_cfg": SceneEntityCfg("right_object")},
    #     weight=5.0,
    # )

    # left_object_goal_tracking_fine_grained = RewTerm(
    #     func=mdp.object_goal_distance,
    #     params={"std": 0.05, "minimal_height": 0.04, "command_name": "left_object_pose", "object_cfg": SceneEntityCfg("left_object")},
    #     weight=5.0,
    # )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)

    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    right_object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": 0.1, "asset_cfg": SceneEntityCfg("right_object")}
    )
    left_object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": 0.1, "asset_cfg": SceneEntityCfg("left_object")}
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -1e-1, "num_steps": 10000}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000}
    )


##
# Environment configuration
##


@configclass
class LiftEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the lifting environment."""

    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=100, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 10.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
