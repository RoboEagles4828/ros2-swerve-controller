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

from typing import Optional
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView

class SwerveView(ArticulationView):
    def __init__(
        self,
        prim_paths_expr: str,
        name: Optional[str] = "SwerveView",
    ) -> None:
        """[summary]
        """

        super().__init__(
            prim_paths_expr=prim_paths_expr,
            name=name,
            reset_xform_properties=False
        )
        
        self._base = RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/swerve_chassis_link", name="base_view", reset_xform_properties=False)
        # self._axle = RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/.*_axle_link", name="axle_view", reset_xform_properties=False)
        # self._wheel = RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/.*_wheel_link", name="wheel_view", reset_xform_properties=False)

        self._axle = [
            RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/front_left_axle_link", name="axle_view[0]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/front_right_axle_link", name="axle_view[1]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/rear_left_axle_link", name="axle_view[2]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/rear_right_axle_link", name="axle_view[3]", reset_xform_properties=False)
            ]
        self._wheel = [
            RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/front_left_wheel_link", name="wheel_view[0]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/front_right_wheel_link", name="wheel_view[1]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/rear_left_wheel_link", name="wheel_view[2]", reset_xform_properties=False),
            RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/rear_right_wheel_link", name="wheel_view[3]", reset_xform_properties=False)
            ]
        # self._axle = [
        #     RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/swerve_chassis_link/front_left_axle_joint", name="axle_view[0]", reset_xform_properties=False),
        #     RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/swerve_chassis_link/front_right_axle_joint", name="axle_view[1]", reset_xform_properties=False),
        #     RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/swerve_chassis_link/rear_left_axle_joint", name="axle_view[2]", reset_xform_properties=False),
        #     RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/swerve_chassis_link/rear_right_axle_joint", name="axle_view[3]", reset_xform_properties=False)
        #     ]
        # self._wheel = [
        #     RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/front_left_axle_link/front_left_wheel_joint", name="wheel_view[0]", reset_xform_properties=False),
        #     RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/front_right_axle_link/front_right_wheel_joint", name="wheel_view[1]", reset_xform_properties=False),
        #     RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/rear_left_axle_link/rear_left_wheel_joint", name="wheel_view[2]", reset_xform_properties=False),
        #     RigidPrimView(prim_paths_expr="/World/envs/.*/swerve/rear_right_axle_link/rear_right_wheel_joint", name="wheel_view[3]", reset_xform_properties=False)
        #     ]

    # def get_knee_transforms(self):
    #     return self._knees.get_world_poses()

    # def is_knee_below_threshold(self, threshold, ground_heights=None):
    #     knee_pos, _ = self._knees.get_world_poses()
    #     knee_heights = knee_pos.view((-1, 4, 3))[:, :, 2]
    #     if ground_heights is not None:
    #         knee_heights -= ground_heights
    #     return (knee_heights[:, 0] < threshold) | (knee_heights[:, 1] < threshold) | (knee_heights[:, 2] < threshold) | (knee_heights[:, 3] < threshold)

    # def is_base_below_threshold(self, threshold, ground_heights):
    #     base_pos, _ = self.get_world_poses()
    #     base_heights = base_pos[:, 2]
    #     base_heights -= ground_heights
    #     return (base_heights[:] < threshold)
