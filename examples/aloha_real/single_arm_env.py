# Ignore lint errors because this file is mostly copied from ACT (https://github.com/tonyzhaozh/act).
# ruff: noqa
#  CURRENTLY SINGLE ARM ONLY SUPPORTS LEFT ARM
#
import collections
import time
from typing import Optional, List
import dm_env
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
import numpy as np

from examples.aloha_real import constants
from examples.aloha_real import single_arm_robot_utils as robot_utils

# This is the reset position that is used by the standard Aloha runtime.
DEFAULT_RESET_POSITION = [0, -0.96, 1.16, 0, -0.3, 0]


class SingleArmRealEnv:
    """
    Environment for real robot single-arm manipulation
    Action space:      [arm_qpos (6),             # absolute joint position
                        gripper_position (1)]     # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ arm_qpos (6),          # absolute joint position
                                        gripper_position (1)]  # normalized gripper position (0: close, 1: open)
                        "qvel": Concat[ arm_qvel (6),         # absolute joint velocity (rad)
                                        gripper_velocity (1)]  # normalized gripper velocity (pos: opening, neg: closing)
                        "images": {"cam_high": (480x640x3),        # h, w, c, dtype='uint8'
                                   "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
                                   "cam_wrist": (480x640x3)}       # h, w, c, dtype='uint8' (single wrist camera)
    """

    def __init__(self, init_node, *, reset_position: Optional[List[float]] = None, setup_robots: bool = True, arm_side: str = "left"):
        # reset_position = START_ARM_POSE[:6]
        self._reset_position = reset_position[:6] if reset_position else DEFAULT_RESET_POSITION
        self.arm_side = "left" # "left" or "right"
        
        # Initialize only one arm
        if arm_side == "left":
            self.puppet_bot = InterbotixManipulatorXS(
                robot_model="vx300s",
                group_name="arm",
                gripper_name="gripper",
                robot_name="puppet_left",
                init_node=init_node,
            )
            self.recorder = robot_utils.Recorder("left", init_node=False)
        else:
            raise NotImplementedError
            
        if setup_robots:
            self.setup_robots()

        # Single wrist camera instead of left/right
        self.image_recorder = robot_utils.ImageRecorder(init_node=False)
        self.gripper_command = JointSingleCommand(name="gripper")

    def setup_robots(self):
        robot_utils.setup_puppet_bot(self.puppet_bot)

    def get_qpos(self):
        qpos_raw = self.recorder.qpos
        arm_qpos = qpos_raw[:6]
        gripper_qpos = [
            constants.PUPPET_GRIPPER_POSITION_NORMALIZE_FN(qpos_raw[7])
        ]  # this is position not joint
        return np.concatenate([arm_qpos, gripper_qpos])

    def get_qvel(self):
        qvel_raw = self.recorder.qvel
        arm_qvel = qvel_raw[:6]
        gripper_qvel = [constants.PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN(qvel_raw[7])]
        return np.concatenate([arm_qvel, gripper_qvel])

    def get_effort(self):
        effort_raw = self.recorder.effort
        robot_effort = effort_raw[:7]  # 6 arm joints + 1 gripper
        return robot_effort

    def get_images(self):
        images = self.image_recorder.get_images()
        # Return only relevant cameras for single arm
        single_arm_images = {
            "cam_high": images["cam_high"],
            "cam_low": images["cam_low"],
        }
        
        # Add the appropriate wrist camera
        if self.arm_side == "left":
            single_arm_images["cam_wrist"] = images["cam_left_wrist"]
        else:
            raise NotImplementedError
        return single_arm_images

    def set_gripper_pose(self, gripper_desired_pos_normalized):
        gripper_desired_joint = constants.PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN(gripper_desired_pos_normalized)
        self.gripper_command.cmd = gripper_desired_joint
        self.puppet_bot.gripper.core.pub_single.publish(self.gripper_command)

    def _reset_joints(self):
        robot_utils.move_arms(
            [self.puppet_bot], [self._reset_position], move_time=1
        )

    def _reset_gripper(self):
        """Set to position mode and do position resets: first close then open. Then change back to PWM mode

        NOTE: This diverges from the original Aloha code which first opens then closes the gripper. Pi internal aloha data
        was collected with the gripper starting in the open position. Leaving the grippers fully closed was also found to
        increase the frequency of motor faults.
        """
        robot_utils.move_grippers(
            [self.puppet_bot], [constants.PUPPET_GRIPPER_JOINT_CLOSE], move_time=1
        )
        robot_utils.move_grippers(
            [self.puppet_bot], [constants.PUPPET_GRIPPER_JOINT_OPEN], move_time=0.5
        )

    def get_observation(self):
        obs = collections.OrderedDict()
        obs["qpos"] = self.get_qpos()
        obs["qvel"] = self.get_qvel()
        obs["effort"] = self.get_effort()
        obs["images"] = self.get_images()
        return obs

    def get_reward(self):
        return 0

    def reset(self, *, fake=False):
        if not fake:
            # Reboot puppet robot gripper motors
            self.puppet_bot.dxl.robot_reboot_motors("single", "gripper", True)
            self._reset_joints()
            self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST, reward=self.get_reward(), discount=None, observation=self.get_observation()
        )

    def step(self, action):
        # Action is now [arm_qpos(6), gripper_pos(1)] = 7 dimensions
        arm_action = action[:6]
        gripper_action = action[6]
        
        self.puppet_bot.arm.set_joint_positions(arm_action, blocking=False)
        self.set_gripper_pose(gripper_action)
        time.sleep(constants.DT)
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID, reward=self.get_reward(), discount=None, observation=self.get_observation()
        )


def get_action(master_bot, arm_side="left"):
    """Get action from master bot for single arm"""
    action = np.zeros(7)  # 6 joint + 1 gripper
    # Arm actions
    action[:6] = master_bot.dxl.joint_states.position[:6]
    # Gripper actions
    action[6] = constants.MASTER_GRIPPER_JOINT_NORMALIZE_FN(master_bot.dxl.joint_states.position[6])
    return action


def make_single_arm_real_env(init_node, *, reset_position: Optional[List[float]] = None, setup_robots: bool = True, arm_side: str = "left") -> SingleArmRealEnv:
    return SingleArmRealEnv(init_node, reset_position=reset_position, setup_robots=setup_robots, arm_side=arm_side)
