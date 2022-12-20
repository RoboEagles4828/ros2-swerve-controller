# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from swervesim.tasks.base.rl_task import RLTask
from swervesim.robots.articulations.swerve import Swerve
from swervesim.robots.articulations.views.swerve_view import SwerveView
from swervesim.tasks.utils.usd_utils import set_drive
from omni.isaac.core.objects import DynamicSphere


from omni.isaac.core.utils.prims import get_prim_at_path

from omni.isaac.core.utils.torch.rotations import *

import numpy as np
import torch
import math


class SwerveTask(RLTask):
    def __init__(
        self,
        name,
        sim_config,
        env,
        offset=None
    ) -> None:
        self._sim_config = sim_config
        self._cfg = sim_config.config
        self._task_cfg = sim_config.task_config

        # normalization
        self.lin_vel_scale = 1.0
        self.ang_vel_scale = 0.5
        self.dof_pos_scale = 1
        self.dof_vel_scale = 0.05
        self.action_scale = 13.5

        # reward scales
        self.rew_scales = {}
        self.rew_scales["lin_vel_xy"] = self._task_cfg["env"]["learn"]["linearVelocityXYRewardScale"]
        self.rew_scales["ang_vel_z"] = self._task_cfg["env"]["learn"]["angularVelocityZRewardScale"]
        self.rew_scales["joint_acc"] = self._task_cfg["env"]["learn"]["jointAccRewardScale"]
        self.rew_scales["action_rate"] = self._task_cfg["env"]["learn"]["actionRateRewardScale"]
        self.rew_scales["cosmetic"] = self._task_cfg["env"]["learn"]["cosmeticRewardScale"]

        # base init state
        pos = self._task_cfg["env"]["baseInitState"]["pos"]
        # rot = self._task_cfg["env"]["baseInitState"]["rot"]
        v_lin = self._task_cfg["env"]["baseInitState"]["vLinear"]
        v_ang = self._task_cfg["env"]["baseInitState"]["vAngular"]
        state = pos + v_lin + v_ang

        self.base_init_state = state

        # other
        self.dt = 1 / 60
        self.max_episode_length_s = self._task_cfg["env"]["learn"]["episodeLength_s"]
        self.max_episode_length = int(self.max_episode_length_s / self.dt + 0.5)
        self.Kp = self._task_cfg["env"]["control"]["stiffness"]
        self.Kd = self._task_cfg["env"]["control"]["damping"]

        for key in self.rew_scales.keys():
            self.rew_scales[key] *= self.dt

        self._num_envs = self._task_cfg["env"]["numEnvs"]
        self._swerve_translation = torch.tensor([0.0, 0.0, 0.62])
        self._env_spacing = self._task_cfg["env"]["envSpacing"]
        self._num_observations = 13
        self._num_actions = 6
        self.swerve_position = torch.tensor([0,0,0])
        self._ball_position = torch.tensor([1,1,0])
        self.target_positions = torch.zeros((self._num_envs, 3), device=self._device, dtype=torch.float32)
        self.target_positions[:, 1] = 1
        self.force_indices = torch.tensor([0, 2], device=self._device)
        self.spinning_indices = torch.tensor([1, 3], device=self._device)

        RLTask.__init__(self, name, env)
        return

    def set_up_scene(self, scene) -> None:
        self.get_swerve()
        self.get_target()
        super().set_up_scene(scene)
        self._swerve = SwerveView(prim_paths_expr="/World/envs/.*/swerve", name="swerveview")
        self._balls = RigidPrimView(prim_paths_expr="/World/envs/.*/ball", name="targets_view", reset_xform_properties=False)
        scene.add(self._swerve)
        scene.add(self._swerve._axle)
        scene.add(self._swerve._wheel)
        scene.add(self._swerve._base)
        scene.add(self._balls)
        print("scene set up")

        return

    def get_swerve(self):
        swerve = Swerve(self.default_zero_env_path + "/swerve", "swerve", self._swerve_translation)
        print("Line:120")
        self._sim_config.apply_articulation_settings("swerve", get_prim_at_path(swerve.prim_path), self._sim_config.parse_actor_config("swerve"))
        print("Line:122")
        # Configure joint properties
        joint_paths = ["front_left_axle_link",
                           "front_right_axle_link",
                           "rear_left_axle_link",
                           "rear_right_axle_link",
                           "front_left_wheel_link",
                           "front_right_wheel_link",
                           "rear_left_wheel_link",
                           "rear_right_wheel_link",
                        ]
        # for quadrant in ["LF", "LH", "RF", "RH"]:
        #     for component, abbrev in [("HIP", "H"), ("THIGH", "K")]:
        #         joint_paths.append(f"{quadrant}_{component}/{quadrant}_{abbrev}FE")
        #     joint_paths.append(f"base/{quadrant}_HAA")
        
        for joint_path in joint_paths:
            if("wheel" in joint_path):
                print("set_wheel")
                set_drive(f"{swerve.prim_path}/{joint_path}", "angular", "velocity", 0, 400, 40, 1000)
            if("axle" in joint_path):
                print("set_axle")
                set_drive(f"{swerve.prim_path}/{joint_path}", "angular", "position", 0, 400, 40, 1000)

    def get_target(self):
        radius = 0.1
        color = torch.tensor([1, 0, 0])
        ball = DynamicSphere(
            prim_path=self.default_zero_env_path + "/ball", 
            translation=self._ball_position, 
            name="target_0",
            radius=radius,
            color=color,
        )
        self._sim_config.apply_articulation_settings("ball", get_prim_at_path(ball.prim_path), self._sim_config.parse_actor_config("ball"))
        ball.set_collision_enabled(True)
    def get_observations(self) -> dict:
        print("line 156")
        self.root_pos, self.root_rot = self._swerve.get_world_poses(clone=False)
        self.root_velocities = self._copters.get_velocities(clone=False)

        root_positions = self.root_pos - self._env_pos
        root_quats = self.root_rot
        root_linvels = self.root_velocities[:, :3]
        root_angvels = self.root_velocities[:, 3:]

        self.obs_buf[..., 0:3] = (self.target_positions - root_positions) / 3
        self.obs_buf[..., 3:7] = root_quats
        self.obs_buf[..., 7:10] = root_linvels / 2
        self.obs_buf[..., 10:13] = root_angvels / math.pi

        observations = {
            self._copters.name: {
                "obs_buf": self.obs_buf
            }
        }
        print("line 195")
        return observations

    def pre_physics_step(self, actions) -> None:
        print("line 203")

        reset_env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        if len(reset_env_ids) > 0:
            self.reset_idx(reset_env_ids)

        set_target_ids = (self.progress_buf % 500 == 0).nonzero(as_tuple=False).squeeze(-1)
        if len(set_target_ids) > 0:
            self.set_targets(set_target_ids)

        self.actions[:] = actions.clone().to(self._device)
        current_targets = self.current_targets + self.action_scale * self.actions * self.dt 
        self.current_targets[:] = torch.clamp(current_targets, self.swerve_dof_lower_limits, self.swerve_dof_upper_limits)
        self._swerve.set_joint_position_targets(self.current_targets, indices)
        print("line 209")
    def reset_idx(self, env_ids):
        print("line 211")
        num_resets = len(env_ids)
        # randomize DOF velocities
        velocities = torch_rand_float(-0.1, 0.1, (num_resets, self._swerve.num_dof), device=self._device)
        dof_pos = self.default_dof_pos[env_ids]
        dof_vel = velocities

        self.current_targets[env_ids] = dof_pos[:]

        root_vel = torch.zeros((num_resets, 6), device=self._device)

        # apply resets
        indices = env_ids.to(dtype=torch.int32)
        self._swerve.set_joint_positions(dof_pos, indices)
        self._swerve.set_joint_velocities(dof_vel, indices)

        self._swerve.set_world_poses(self.initial_root_pos[env_ids].clone(), self.initial_root_rot[env_ids].clone(), indices)
        self._swerve.set_velocities(root_vel, indices)

        self.commands_x[env_ids] = torch_rand_float(
            self.command_x_range[0], self.command_x_range[1], (num_resets, 1), device=self._device
        ).squeeze()
        self.commands_y[env_ids] = torch_rand_float(
            self.command_y_range[0], self.command_y_range[1], (num_resets, 1), device=self._device
        ).squeeze()
        self.commands_yaw[env_ids] = torch_rand_float(
            self.command_yaw_range[0], self.command_yaw_range[1], (num_resets, 1), device=self._device
        ).squeeze()

        # bookkeeping
        self.reset_buf[env_ids] = 0
        self.progress_buf[env_ids] = 0
        self.last_actions[env_ids] = 0.
        self.last_dof_vel[env_ids] = 0.
        print("line 249")

    def post_reset(self):
        print("line 252")
        self.root_pos, self.root_rot = self._swerve.get_world_poses()
        self.root_velocities = self._copters.get_velocities()

        self.dof_pos = self._copters.get_joint_positions()
        self.dof_vel = self._copters.get_joint_velocities()

        self.initial_ball_pos, self.initial_ball_rot = self._balls.get_world_poses()
        self.initial_root_pos, self.initial_root_rot = self.root_pos.clone(), self.root_rot.clone()


        # initialize some data used later on
        self.extras = {}
        self.gravity_vec = torch.tensor([0.0, 0.0, -1.0], device=self._device).repeat(
            (self._num_envs, 1)
        )
        self.actions = torch.zeros(
            self._num_envs, self.num_actions, dtype=torch.float, device=self._device, requires_grad=False
        )
        self.last_dof_vel = torch.zeros((self._num_envs, 12), dtype=torch.float, device=self._device, requires_grad=False)
        self.last_actions = torch.zeros(self._num_envs, self.num_actions, dtype=torch.float, device=self._device, requires_grad=False)

        self.time_out_buf = torch.zeros_like(self.reset_buf)

        # randomize all envs
        indices = torch.arange(self._swerve.count, dtype=torch.int64, device=self._device)
        self.reset_idx(indices)
        print("line 276")
    def set_targets(self, env_ids):
        num_sets = len(env_ids)
        envs_long = env_ids.long()
        # set target position randomly with x, y in (-1, 1) and z in (1, 2)
        self.target_positions[envs_long, 0:2] = torch.rand((num_sets, 2), device=self._device) * 2 - 1
        self.target_positions[envs_long, 2] = torch.rand(num_sets, device=self._device) + 1

        # shift the target up so it visually aligns better
        ball_pos = self.target_positions[envs_long] + self._env_pos[envs_long]
        ball_pos[:, 2] += 0.4
        self._balls.set_world_poses(ball_pos[:, 0:3], self.initial_ball_rot[envs_long].clone(), indices=env_ids)
    def calculate_metrics(self) -> None:
        torso_position, torso_rotation = self._swerve.get_world_poses(clone=False)
        root_velocities = self._swerve.get_velocities(clone=False)
        dof_pos = self._swerve.get_joint_positions(clone=False)
        dof_vel = self._swerve.get_joint_velocities(clone=False)

        velocity = root_velocities[:, 0:3]
        ang_velocity = root_velocities[:, 3:6]

        base_lin_vel = quat_rotate_inverse(torso_rotation, velocity)
        base_ang_vel = quat_rotate_inverse(torso_rotation, ang_velocity)

        # velocity tracking reward
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - base_lin_vel[:, :2]), dim=1)
        ang_vel_error = torch.square(self.commands[:, 2] - base_ang_vel[:, 2])
        rew_lin_vel_xy = torch.exp(-lin_vel_error / 0.25) * self.rew_scales["lin_vel_xy"]
        rew_ang_vel_z = torch.exp(-ang_vel_error / 0.25) * self.rew_scales["ang_vel_z"]

        rew_lin_vel_z = torch.square(base_lin_vel[:, 2]) * self.rew_scales["lin_vel_z"]
        rew_joint_acc = torch.sum(torch.square(self.last_dof_vel - dof_vel), dim=1) * self.rew_scales["joint_acc"]
        rew_action_rate = torch.sum(torch.square(self.last_actions - self.actions), dim=1) * self.rew_scales["action_rate"]
        rew_cosmetic = torch.sum(torch.abs(dof_pos[:, 0:4] - self.default_dof_pos[:, 0:4]), dim=1) * self.rew_scales["cosmetic"]

        total_reward = rew_lin_vel_xy + rew_ang_vel_z + rew_joint_acc  + rew_action_rate + rew_cosmetic + rew_lin_vel_z
        total_reward = torch.clip(total_reward, 0.0, None)

        self.last_actions[:] = self.actions[:]
        self.last_dof_vel[:] = dof_vel[:]

        self.fallen_over = self._swerve.is_base_below_threshold(threshold=0.51, ground_heights=0.0)
        total_reward[torch.nonzero(self.fallen_over)] = -1
        self.rew_buf[:] = total_reward.detach()
        print("line 309")

    def is_done(self) -> None:
        print("line 312")
        # reset agents
        time_out = self.progress_buf >= self.max_episode_length - 1
        self.reset_buf[:] = time_out | self.fallen_over
        print("line 316")
