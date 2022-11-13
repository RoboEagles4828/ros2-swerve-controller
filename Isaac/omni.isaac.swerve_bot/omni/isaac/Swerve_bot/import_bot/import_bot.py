import omni.graph.core as og
import omni.usd
from omni.isaac.Swerve_bot.base_sample import BaseSample
from omni.isaac.urdf import _urdf
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils import prims
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.kit.viewport_legacy import get_default_viewport_window
from pxr import UsdPhysics
import omni.kit.commands
import os
import numpy as np
import math
import carb

def set_drive_params(drive, stiffness, damping, max_force):
    drive.GetStiffnessAttr().Set(stiffness)
    drive.GetDampingAttr().Set(damping)
    drive.GetMaxForceAttr().Set(max_force)
    return

class ImportBot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        self.setup_perspective_cam()
        self.setup_world_action_graph()
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self.robot_name = "Swerve"
        self.extension_path = os.path.abspath(__file__)
        self.project_root_path = os.path.abspath(os.path.join(self.extension_path, "../../../../../../.."))
        self.path_to_urdf = os.path.join(self.project_root_path, "src/robot_description/Swerve.urdf")
        carb.log_info(self.path_to_urdf)

        self._robot_prim_path = self.import_robot(self.path_to_urdf)
        
        if self._robot_prim_path is None:
            print("Error: failed to import robot")
            return
        
        self._robot_prim = self._world.scene.add(
            Robot(prim_path=self._robot_prim_path, name=self.robot_name, position=np.array([0.0, 0.0, 0.3]))
        )
        self.configure_robot(self._robot_prim_path)
        return
    
    def import_robot(self, urdf_path):
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.fix_base = False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = False
        import_config.import_inertia_tensor = True
        import_config.default_drive_strength = 1047.19751
        import_config.default_position_drive_damping = 52.35988
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
        import_config.distance_scale = 1.0
        import_config.density = 0.0
        result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", 
            urdf_path=urdf_path,
            import_config=import_config)

        if result:
            return prim_path
        return None

    
    def configure_robot(self, robot_prim_path):
        w_sides = ['left', 'right']
        l_sides = ['front', 'rear']
        stage = self._world.stage

        for w_side in w_sides:
            for l_side in l_sides:
                for i in range(9):
                    joint_name = "{}_{}_roller_{}_joint".format(l_side, w_side, i)
                    joint_path = "{}/{}_{}_mecanum_link/{}".format(robot_prim_path, l_side, w_side, joint_name)
                    prim = stage.GetPrimAtPath(joint_path)
                    omni.kit.commands.execute(
                        "UnapplyAPISchemaCommand",
                        api=UsdPhysics.DriveAPI,
                        prim=prim,
                        api_prefix="drive",
                        multiple_api_token="angular")
                    # drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                    # set_drive_params(drive, 0.0, 2.0, 0.0)

        front_left = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/chassis_link/front_left_mecanum_joint"), "angular")
        front_right = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/chassis_link/front_right_mecanum_joint"), "angular")
        rear_left = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/chassis_link/rear_left_mecanum_joint"), "angular")
        rear_right = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_prim_path}/chassis_link/rear_right_mecanum_joint"), "angular")

        set_drive_params(front_left, 0, math.radians(1e5), 98.0)
        set_drive_params(front_right, 0, math.radians(1e5), 98.0)
        set_drive_params(rear_left, 0, math.radians(1e5), 98.0)
        set_drive_params(rear_right, 0, math.radians(1e5), 98.0)
        self.create_lidar(robot_prim_path)
        self.create_depth_camera()
        self.setup_robot_action_graph(robot_prim_path)
        return

    def create_lidar(self, robot_prim_path):
        lidar_parent = "{}/lidar_link".format(robot_prim_path)
        lidar_path = "/lidar"
        self.lidar_prim_path = lidar_parent + lidar_path
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=lidar_path,
            parent=lidar_parent,
            min_range=0.4,
            max_range=25.0,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False
        )
        return

    
    def create_depth_camera(self):
        self.depth_left_camera_path = f"{self._robot_prim_path}/zed_left_camera_frame/left_cam"
        self.depth_right_camera_path = f"{self._robot_prim_path}/zed_right_camera_frame/right_cam"
        self.left_camera = prims.create_prim(
            prim_path=self.depth_left_camera_path,
            prim_type="Camera",
            attributes={
                "focusDistance": 1,
                "focalLength": 24,
                "horizontalAperture": 20.955,
                "verticalAperture": 15.2908,
                "clippingRange": (0.1, 1000000),
                "clippingPlanes": np.array([1.0, 0.0, 1.0, 1.0]),
            },
        )
        self.right_camera = prims.create_prim(
            prim_path=self.depth_right_camera_path,
            prim_type="Camera",
            attributes={
                "focusDistance": 1,
                "focalLength": 24,
                "horizontalAperture": 20.955,
                "verticalAperture": 15.2908,
                "clippingRange": (0.1, 1000000),
                "clippingPlanes": np.array([1.0, 0.0, 1.0, 1.0]),
            },
        )
        return

    def setup_world_action_graph(self):
        og.Controller.edit(
            {"graph_path": "/globalclock", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
            }
        )
        return
    
    def setup_perspective_cam(self):
        # Get the Viewport and the Default Camera
        viewport_window = get_default_viewport_window()
        camera = self.get_world().stage.GetPrimAtPath(viewport_window.get_active_camera())

        # Get Default Cam Values
        camAttributes = {}
        camOrientation = None
        camTranslation = None
        for att in camera.GetAttributes():
            name = att.GetName()
            if not (name.startswith('omni') or name.startswith('xform')):
                camAttributes[att.GetName()] = att.Get()
            elif name == 'xformOp:orient':
                convertedQuat = [att.Get().GetReal()] + list(att.Get().GetImaginary())
                camOrientation = np.array(convertedQuat)
            elif name == 'xformOp:translate':
                camTranslation = np.array(list(att.Get()))

        # Modify what we want
        camAttributes["clippingRange"] = (0.1, 1000000)
        camAttributes["clippingPlanes"] = np.array([1.0, 0.0, 1.0, 1.0])

        # Create a new camera with desired values
        cam_path = "/World/PerspectiveCam"
        prims.create_prim(
            prim_path=cam_path,
            prim_type="Camera",
            translation=camTranslation,
            orientation=camOrientation,
            attributes=camAttributes,
        )

        # Use the camera for our viewport
        viewport_window.set_active_camera(cam_path)
        return

    def setup_robot_action_graph(self, robot_prim_path):
        robot_controller_path = f"{robot_prim_path}/ros_interface_controller"
        og.Controller.edit(
            {"graph_path": robot_controller_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("isaac_read_lidar_beams_node", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                    ("ros2_publish_laser_scan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                    ("ros2_camera_helper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("ros2_camera_helper_02", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("ros2_camera_helper_03", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("isaac_create_viewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                    ("set_active_camera", "omni.graph.ui.SetActiveViewportCamera"),
                    ("get_prim_path", "omni.graph.nodes.GetPrimPath"),
                    ("constant_token", "omni.graph.nodes.ConstantToken"),
                    ("constant_token_02", "omni.graph.nodes.ConstantToken"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                    ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
                    ("articulation_controller.inputs:usePath", False),
                    ("ros2_publish_laser_scan.inputs:topicName", "laser_scan"),
                    ("ros2_publish_laser_scan.inputs:frameId", "base_link"),
                    ("ros2_camera_helper.inputs:frameId", "base_link"),
                    ("ros2_camera_helper_02.inputs:frameId", "base_link"),
                    ("ros2_camera_helper_02.inputs:topicName", "camera_info"),
                    ("ros2_camera_helper_03.inputs:frameId", "base_link"),
                    ("ros2_camera_helper_03.inputs:topicName", "depth"),
                    ("isaac_create_viewport.inputs:viewportId", 1),
                    ("constant_token.inputs:value", "camera_info"),
                    ("constant_token_02.inputs:value", "depth"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "isaac_read_lidar_beams_node.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "isaac_create_viewport.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "articulation_controller.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "ros2_publish_laser_scan.inputs:timeStamp"),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "ros2_publish_laser_scan.inputs:context"),
                    ("Context.outputs:context", "ros2_camera_helper.inputs:context"),
                    ("Context.outputs:context", "ros2_camera_helper_02.inputs:context"),
                    ("SubscribeJointState.outputs:jointNames", "articulation_controller.inputs:jointNames"),
                    ("SubscribeJointState.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                    ("isaac_read_lidar_beams_node.outputs:execOut", "ros2_publish_laser_scan.inputs:execIn"),
                    ("isaac_read_lidar_beams_node.outputs:azimuthRange", "ros2_publish_laser_scan.inputs:azimuthRange"),
                    ("isaac_read_lidar_beams_node.outputs:depthRange", "ros2_publish_laser_scan.inputs:depthRange"),
                    ("isaac_read_lidar_beams_node.outputs:horizontalFov", "ros2_publish_laser_scan.inputs:horizontalFov"),
                    ("isaac_read_lidar_beams_node.outputs:horizontalResolution", "ros2_publish_laser_scan.inputs:horizontalResolution"),
                    ("isaac_read_lidar_beams_node.outputs:intensitiesData", "ros2_publish_laser_scan.inputs:intensitiesData"),
                    ("isaac_read_lidar_beams_node.outputs:linearDepthData", "ros2_publish_laser_scan.inputs:linearDepthData"),
                    ("isaac_read_lidar_beams_node.outputs:numCols", "ros2_publish_laser_scan.inputs:numCols"),
                    ("isaac_read_lidar_beams_node.outputs:numRows", "ros2_publish_laser_scan.inputs:numRows"),
                    ("isaac_read_lidar_beams_node.outputs:rotationRate", "ros2_publish_laser_scan.inputs:rotationRate"),
                    ("isaac_create_viewport.outputs:viewport", "ros2_camera_helper.inputs:viewport"),
                    ("isaac_create_viewport.outputs:viewport", "ros2_camera_helper_02.inputs:viewport"),
                    ("isaac_create_viewport.outputs:viewport", "set_active_camera.inputs:viewport"),
                    ("isaac_create_viewport.outputs:execOut", "set_active_camera.inputs:execIn"),
                    ("set_active_camera.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
                    ("set_active_camera.outputs:execOut", "ros2_camera_helper_02.inputs:execIn"),
                    ("get_prim_path.outputs:primPath", "set_active_camera.inputs:primPath"),
                    ("constant_token.inputs:value", "ros2_camera_helper_02.inputs:type"),
                ],
            }
        )

        set_target_prims(primPath=f"{robot_controller_path}/articulation_controller", targetPrimPaths=[robot_prim_path])
        set_target_prims(primPath=f"{robot_controller_path}/PublishJointState", targetPrimPaths=[robot_prim_path])
        set_target_prims(primPath=f"{robot_controller_path}/isaac_read_lidar_beams_node", targetPrimPaths=[self.lidar_prim_path], inputName="inputs:lidarPrim")
        set_target_prims(primPath=f"{robot_controller_path}/get_prim_path", targetPrimPaths=[self.depth_left_camera_path], inputName="inputs:prim")
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return
    
    async def setup_post_clear(self):
        return
    
    def world_cleanup(self):
        self._world.scene.remove_object(self.robot_name)
        return
