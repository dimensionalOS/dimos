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

.. autoclass:: dimos.spec.utils.Spec

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

.. autoclass:: dimos.core.coordination.blueprints.TransportSpec

Messages
========

.. autoclass:: dimos.msgs.geometry_msgs.Twist.Twist

.. autoclass:: dimos.msgs.geometry_msgs.Vector3.Vector3

.. autoclass:: dimos.msgs.geometry_msgs.Quaternion.Quaternion

.. autoclass:: dimos.msgs.geometry_msgs.Transform.Transform

.. autoclass:: dimos.msgs.geometry_msgs.PoseStamped.PoseStamped

.. autoclass:: dimos.msgs.geometry_msgs.Pose.Pose

.. autoclass:: dimos.msgs.tf2_msgs.TFMessage.TFMessage

.. autoclass:: dimos.msgs.sensor_msgs.Image.Image

.. autoclass:: dimos.msgs.sensor_msgs.Image.ImageFormat

.. autoclass:: dimos.msgs.sensor_msgs.Image.AgentImageMessage

Agents and MCP
==============

.. autoclass:: dimos.agents.mcp.mcp_client.McpClient

.. autoclass:: dimos.agents.mcp.mcp_client.McpClientConfig

.. autoclass:: dimos.agents.mcp.mcp_server.McpServer
