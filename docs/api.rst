.. _dimos-api-reference:

=============
API Reference
=============

This reference is generated automatically from the docstrings in the source.

Blueprints and coordination
============================

.. autofunction:: dimos.core.coordination.blueprints.autoconnect

.. autoclass:: dimos.core.coordination.blueprints.Blueprint

.. autoclass:: dimos.core.coordination.blueprints.BlueprintAtom

.. autoclass:: dimos.core.coordination.blueprints.StreamRef

.. autoclass:: dimos.core.coordination.blueprints.ModuleRef

.. autoclass:: dimos.core.coordination.module_coordinator.ModuleCoordinator

.. autoclass:: dimos.core.coordination.module_coordinator.ModuleDescriptor

.. autoclass:: dimos.core.coordination.python_worker.Actor

.. autoclass:: dimos.core.coordination.python_worker.ActorFuture

.. autoclass:: dimos.core.global_config.GlobalConfig

.. autodata:: dimos.core.global_config.global_config

.. autoclass:: dimos.core.native_module.NativeModule
   :no-members:

.. autoclass:: dimos.core.native_module.NativeModuleConfig
   :no-members:

.. autoclass:: dimos.core.native_module.LogFormat
   :no-members:

.. autoclass:: dimos.spec.utils.Spec

.. autoclass:: dimos.protocol.service.spec.BaseConfig
   :no-members:

.. autoclass:: dimos.protocol.service.spec.Configurable
   :no-members:

.. autoclass:: dimos.protocol.service.system_configurator.base.SystemConfigurator

.. autoclass:: dimos.protocol.tf.tf.TFSpec

.. autoclass:: dimos.protocol.tf.tf.TFConfig

Modules and streams
===================

.. autofunction:: dimos.core.core.rpc

.. autoclass:: dimos.core.module.ModuleBase

.. autoclass:: dimos.core.module.Module

   .. autoattribute:: blueprint

.. autoclass:: dimos.core.module.ModuleConfig

.. autoclass:: dimos.core.module.SkillInfo

.. automodule:: dimos.core.introspection.module.info
   :members:

.. py:class:: dimos.core.module.ModuleSpec

   Type alias for a ``(module type, global config, kwargs)`` triple describing a
   module to build.

.. py:class:: dimos.visualization.rerun.bridge.RerunMulti

   Type alias for ``list[tuple[str, Archetype]]`` — rerun entity paths paired with
   their archetypes.

.. autoclass:: dimos.core.rpc_client.RPCClient

.. autoclass:: dimos.core.rpc_client.ModuleProxyProtocol

.. py:class:: dimos.core.rpc_client.ModuleProxy

   A ``TYPE_CHECKING``-only stand-in exposing a remote module's :func:`@rpc
   <dimos.core.core.rpc>` methods over the coordinator; instances are built
   dynamically at runtime, so there is no concrete class to introspect.

.. autoclass:: dimos.protocol.rpc.spec.RPCSpec

.. autoclass:: dimos.protocol.rpc.spec.RPCServer

.. autoclass:: dimos.protocol.rpc.spec.RPCInspectable

.. autoclass:: dimos.core.stream.Stream

.. autoclass:: dimos.core.stream.State

.. autoclass:: dimos.core.stream.In

.. autoclass:: dimos.core.stream.Out

.. autoclass:: dimos.core.stream.RemoteIn

.. autoclass:: dimos.core.stream.RemoteOut

Transports
==========

.. autoclass:: dimos.core.stream.Transport

.. autoclass:: dimos.core.transport.LCMTransport

.. autoclass:: dimos.core.transport.ROSTransport
   :no-members:

.. autoclass:: dimos.core.transport.ZenohTransport
   :no-members:

.. autoclass:: dimos.core.transport.pLCMTransport
   :no-members:

.. autoclass:: dimos.core.transport.pSHMTransport
   :no-members:

.. autoclass:: dimos.core.transport.pZenohTransport
   :no-members:

.. autoclass:: dimos.core.coordination.blueprints.TransportSpec

.. autoclass:: dimos.protocol.pubsub.spec.PubSub
   :no-members:

.. autoclass:: dimos.protocol.pubsub.encoders.LCMEncoderMixin
   :no-members:

.. autoclass:: dimos.protocol.pubsub.encoders.PickleEncoderMixin
   :no-members:

.. autoclass:: dimos.protocol.pubsub.encoders.PubSubEncoderMixin
   :no-members:

.. autoclass:: dimos.protocol.pubsub.impl.memory.Memory
   :no-members:

.. autoclass:: dimos.protocol.pubsub.impl.shmpubsub.PickleSharedMemory
   :no-members:

.. autoclass:: dimos.protocol.pubsub.impl.zenohpubsub.Zenoh
   :no-members:

.. autoclass:: dimos.protocol.pubsub.impl.jpeg_lcm.JpegEncoderMixin
   :no-members:

.. py:currentmodule:: dimos.protocol.pubsub.impl.redispubsub

.. py:class:: Redis

.. py:currentmodule:: dimos.protocol.pubsub.impl.ddspubsub

.. py:class:: DDS

Messages
========

.. autoclass:: dimos.msgs.geometry_msgs.Twist.Twist

.. autoclass:: dimos.msgs.geometry_msgs.Vector3.Vector3

.. autoclass:: dimos.msgs.geometry_msgs.Quaternion.Quaternion

.. autoclass:: dimos.msgs.geometry_msgs.Transform.Transform

.. autoclass:: dimos.msgs.geometry_msgs.PoseStamped.PoseStamped

.. autoclass:: dimos.msgs.geometry_msgs.Pose.Pose

.. autoclass:: dimos.msgs.geometry_msgs.PointStamped.PointStamped
   :no-members:

.. autoclass:: dimos.msgs.geometry_msgs.TwistStamped.TwistStamped
   :no-members:

.. autoclass:: dimos.msgs.tf2_msgs.TFMessage.TFMessage

.. autoclass:: dimos.msgs.sensor_msgs.Image.Image

.. autoclass:: dimos.msgs.sensor_msgs.Image.ImageFormat

.. autoclass:: dimos.msgs.sensor_msgs.Image.AgentImageMessage

.. autoclass:: dimos.msgs.sensor_msgs.CameraInfo.CameraInfo
   :no-members:

.. autoclass:: dimos.msgs.sensor_msgs.Imu.Imu
   :no-members:

.. autoclass:: dimos.msgs.sensor_msgs.PointCloud2.PointCloud2
   :no-members:

.. autoclass:: dimos.msgs.std_msgs.Int32.Int32
   :no-members:

.. autoclass:: dimos.msgs.nav_msgs.OccupancyGrid.OccupancyGrid
   :no-members:

.. autoclass:: dimos.msgs.nav_msgs.Path.Path
   :no-members:

.. autoclass:: dimos.msgs.trajectory_msgs.JointTrajectory.JointTrajectory
   :no-members:

Agents and MCP
==============

.. autofunction:: dimos.agents.annotation.skill

.. autoclass:: dimos.agents.mcp.mcp_client.McpClient

.. autoclass:: dimos.agents.mcp.mcp_client.McpClientConfig

.. autoclass:: dimos.agents.mcp.mcp_server.McpServer

.. py:currentmodule:: dimos.agents.skills.google_maps_skill_container

.. py:class:: GoogleMapsSkillContainer

.. py:currentmodule:: dimos.agents.skills.navigation

.. py:class:: NavigationSkillContainer

.. py:currentmodule:: dimos.agents.skills.osm

.. py:class:: OsmSkill

.. autoclass:: dimos.agents.skills.speak_skill.SpeakSkill
   :no-members:

.. autoclass:: dimos.agents.web_human_input.WebInput
   :no-members:

.. py:currentmodule:: dimos.robot.unitree.unitree_skill_container

.. py:class:: UnitreeSkillContainer

.. py:currentmodule:: dimos.robot.unitree.go2.connection

.. py:class:: GO2Connection

.. py:currentmodule:: dimos.robot.unitree.keyboard_teleop

.. py:class:: KeyboardTeleop

.. py:currentmodule:: dimos.hardware.sensors.camera.module

.. py:class:: CameraModule

.. autoclass:: dimos.hardware.sensors.camera.webcam.Webcam
   :no-members:

.. autoclass:: dimos.teleop.keyboard.keyboard_teleop_module.KeyboardTeleopModule
   :no-members:

.. py:currentmodule:: dimos.teleop.hosted.go2_command

.. py:class:: Go2CommandModule

.. autoclass:: dimos.teleop.hosted.camera_mux.CameraMuxModule
   :no-members:

.. autoclass:: dimos.teleop.hosted.map_compress.MapCompressModule
   :no-members:

.. py:currentmodule:: dimos.teleop.hosted.hosted_stats

.. py:class:: HostedStatsModule

.. autoclass:: dimos.protocol.pubsub.impl.webrtc.providers.broker.BrokerProvider
   :no-members:

.. autoclass:: dimos.protocol.pubsub.impl.webrtc.providers.broker.BrokerConfig
   :no-members:

Data and reactive utilities
===========================

.. autoclass:: dimos.types.timestamped.Timestamped
   :no-members:

.. autofunction:: dimos.types.timestamped.align_timestamped

.. autofunction:: dimos.utils.data.get_data

.. autofunction:: dimos.utils.reactive.backpressure

.. autofunction:: dimos.utils.reactive.getter_cold

.. autofunction:: dimos.utils.reactive.getter_hot

.. autofunction:: dimos.utils.reactive.quality_barrier

.. autofunction:: dimos.utils.reactive.to_observable

.. autofunction:: dimos.msgs.sensor_msgs.Image.sharpness_barrier

.. py:currentmodule:: dimos.utils.reactive

.. py:function:: callback_to_observable

.. py:currentmodule:: dimos.utils.data

.. py:class:: LfsPath

.. autofunction:: dimos.core.introspection.svg.to_svg

.. py:currentmodule:: dimos.perception.common.utils

.. py:function:: load_camera_info

Memory
======

.. autoclass:: dimos.memory2.embed.EmbedImages
   :no-members:

.. autoclass:: dimos.memory2.store.base.Store
   :no-members:

.. autoclass:: dimos.memory2.store.sqlite.SqliteStore
   :no-members:

.. autoclass:: dimos.memory2.transform.QualityWindow
   :no-members:

.. autofunction:: dimos.memory2.transform.downsample

.. autofunction:: dimos.memory2.transform.normalize

.. autofunction:: dimos.memory2.transform.peaks

.. autofunction:: dimos.memory2.transform.significant

.. autofunction:: dimos.memory2.transform.smooth

.. autofunction:: dimos.memory2.transform.smooth_time

.. autofunction:: dimos.memory2.transform.speed

.. autofunction:: dimos.memory2.transform.throttle

.. autoclass:: dimos.memory2.vis.color.Color
   :no-members:

.. autoclass:: dimos.memory2.vis.color.ColorRange
   :no-members:

.. autoclass:: dimos.memory2.vis.plot.elements.HLine
   :no-members:

.. autoclass:: dimos.memory2.vis.plot.elements.Series
   :no-members:

.. autoclass:: dimos.memory2.vis.plot.elements.Style
   :no-members:

.. autoclass:: dimos.memory2.vis.plot.elements.VLine
   :no-members:

.. autoclass:: dimos.memory2.vis.plot.plot.Plot
   :no-members:

.. autoclass:: dimos.memory2.vis.space.elements.Box3D
   :no-members:

.. autoclass:: dimos.memory2.vis.space.elements.Point
   :no-members:

.. autoclass:: dimos.memory2.vis.space.space.Space
   :no-members:

.. py:currentmodule:: dimos.memory2.vis.utils

.. py:function:: mosaic

Mapping and navigation
======================

.. autoclass:: dimos.mapping.costmapper.CostMapper
   :no-members:

.. py:currentmodule:: dimos.mapping.relocalization.module

.. py:class:: RelocalizationModule

.. py:currentmodule:: dimos.mapping.voxels

.. py:class:: VoxelGridMapper

.. py:class:: VoxelMapTransformer

.. autoclass:: dimos.navigation.movement_manager.movement_manager.MovementManager
   :no-members:

.. autoclass:: dimos.navigation.patrolling.module.PatrollingModule
   :no-members:

.. autoclass:: dimos.navigation.replanning_a_star.module.ReplanningAStarPlanner
   :no-members:

.. autofunction:: dimos.mapping.occupancy.inflation.simple_inflate

.. autofunction:: dimos.mapping.pointclouds.occupancy.general_occupancy

.. autofunction:: dimos.mapping.pointclouds.occupancy.simple_occupancy

.. autofunction:: dimos.mapping.pointclouds.occupancy.height_cost_occupancy

.. autofunction:: dimos.mapping.pointclouds.util.read_pointcloud

Manipulation
============

.. autoclass:: dimos.control.components.HardwareComponent
   :no-members:

.. autoclass:: dimos.control.components.HardwareType
   :no-members:

.. autofunction:: dimos.control.components.make_joints

.. autoclass:: dimos.control.coordinator.ControlCoordinator
   :no-members:

.. autoclass:: dimos.control.coordinator.TaskConfig
   :no-members:

.. autoclass:: dimos.control.tasks.eef_twist_task.eef_twist_task.EEFTwistTask
   :no-members:

.. autoclass:: dimos.control.tasks.eef_twist_task.eef_twist_task.EEFTwistTaskConfig
   :no-members:

.. autoclass:: dimos.hardware.manipulators.registry.AdapterRegistry
   :no-members:

.. autoclass:: dimos.hardware.manipulators.spec.ManipulatorAdapter
   :no-members:

.. autoclass:: dimos.manipulation.manipulation_module.ManipulationModule
   :no-members:

.. autoclass:: dimos.manipulation.manipulation_module.ManipulationModuleConfig
   :no-members:

.. py:currentmodule:: dimos.manipulation.pick_and_place_module

.. py:class:: PickAndPlaceModule

.. autoclass:: dimos.manipulation.planning.spec.config.RobotModelConfig
   :no-members:

.. autoclass:: dimos.manipulation.planning.spec.models.GeneratedPlan
   :no-members:

.. autoclass:: dimos.manipulation.planning.spec.models.PlanningSceneInfo
   :no-members:

.. autoclass:: dimos.manipulation.planning.spec.models.VisualizationSession
   :no-members:

.. autoclass:: dimos.manipulation.planning.spec.models.VisualizationStateFrame
   :no-members:

.. autoclass:: dimos.manipulation.planning.spec.protocols.VisualizationSpec
   :no-members:

.. autoclass:: dimos.manipulation.planning.spec.protocols.WorldSpec
   :no-members:

.. autoclass:: dimos.manipulation.planning.monitor.world_monitor.WorldMonitor
   :no-members:

.. autoclass:: dimos.manipulation.visualization.operator.ManipulationOperator
   :no-members:

Visualization
=============

.. autoclass:: dimos.visualization.rerun.bridge.RerunBridgeModule
   :no-members:

.. autofunction:: dimos.visualization.rerun.init.rerun_init

.. py:currentmodule:: dimos.visualization.vis_module

.. py:function:: vis_module
