================================================
The Agentive Operating System for Physical Space
================================================

Dimensional is the modern operating system for generalist robotics. We are setting the
next-generation SDK standard, integrating with the majority of robot manufacturers.

With a simple install and no ROS required, build physical applications entirely in Python
that run on any humanoid, quadruped, or drone. Dimensional is agent native — "vibecode" your
robots in natural language and build (local and hosted) multi-agent systems that work
seamlessly with your hardware. Agents run as native modules, subscribing to any embedded
stream, from perception (lidar, camera) and spatial memory down to control loops and motor
drivers.

Current version is |release|.

.. important::

   This is a pre-release beta. Direct your favorite agent (OpenClaw, Claude Code, etc.) to
   `AGENTS.md <https://github.com/dimensionalOS/dimos/blob/main/AGENTS.md>`_ and the
   `Agent CLI and MCP`_ interfaces to start building Dimensional applications.

Capabilities
============

- **Navigation and mapping** — SLAM, dynamic obstacle avoidance, route planning, and
  autonomous exploration, via both DimOS native and ROS
  (`watch video <https://x.com/stash_pomichter/status/2010471593806545367>`__).
- **Perception** — detectors, 3D projections, VLMs, and audio processing.
- **Agentive control, MCP** — *"hey Robot, go find the kitchen"*
  (`watch video <https://x.com/stash_pomichter/status/2015912688854200322>`__).
- **Spatial memory** — spatial and temporal RAG, dynamic memory, object localization and
  permanence (`watch video <https://x.com/stash_pomichter/status/1980741077205414328>`__).

Hardware
========

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Category
     - Supported platforms
   * - Quadruped
     - :brand:`Unitree` Go2 pro/air (stable), :brand:`Unitree` B1 (experimental)
   * - Humanoid
     - :brand:`Unitree` G1 (beta)
   * - Arm
     - :brand:`xArm` (beta), :brand:`AgileX` Piper (beta)
   * - Drone
     - MAVLink (alpha), DJI :brand:`Mavic` (alpha)
   * - Misc
     - Force Torque Sensor (experimental)

Installation
============

Interactive install
-------------------

.. code-block:: bash

   curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash

See ``scripts/install.sh --help`` for non-interactive and advanced options.

Manual system install
---------------------

To set up your system dependencies, follow one of these guides:

- `Ubuntu 22.04 / 24.04 <https://github.com/dimensionalOS/dimos/blob/main/docs-old/installation/ubuntu.md>`_ (stable)
- `NixOS / General Linux <https://github.com/dimensionalOS/dimos/blob/main/docs-old/installation/nix.md>`_ (stable)
- `macOS <https://github.com/dimensionalOS/dimos/blob/main/docs-old/installation/osx.md>`_ (alpha)

Full system requirements, tested configs, and dependency tiers are listed in
`docs-old/requirements.md <https://github.com/dimensionalOS/dimos/blob/main/docs-old/requirements.md>`_.

Python install
--------------

Quick start
~~~~~~~~~~~

.. code-block:: bash

   uv venv --python "3.12"
   source .venv/bin/activate
   uv pip install 'dimos[base,unitree]'

   # Replay a recorded quadruped session (no hardware needed).
   # NOTE: the first run shows a black rerun window while ~75 MB downloads from LFS.
   dimos --replay run unitree-go2

.. code-block:: bash

   # Install with simulation support.
   uv pip install 'dimos[base,unitree,sim]'

   # Run a quadruped in MuJoCo simulation.
   dimos --simulation run unitree-go2

   # Run a humanoid in simulation.
   dimos --simulation run unitree-g1-sim

.. code-block:: bash

   # Control a real robot (Unitree quadruped over WebRTC).
   export ROBOT_IP=<YOUR_ROBOT_IP>
   dimos run unitree-go2

Featured runfiles
=================

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Run command
     - What it does
   * - :bash:`dimos --replay run unitree-go2`
     - Quadruped navigation replay — SLAM, costmap, A* planning
   * - :bash:`dimos --replay --replay-db go2_bigoffice run unitree-go2-memory`
     - Quadruped temporal memory replay
   * - :bash:`dimos --simulation run unitree-go2-agentic`
     - Quadruped agentic + MCP server in simulation
   * - :bash:`dimos --simulation run unitree-g1-sim`
     - Humanoid in MuJoCo simulation
   * - :bash:`dimos --replay run drone-basic`
     - Drone video + telemetry replay
   * - :bash:`dimos run demo-camera`
     - Webcam demo — no hardware needed

Full blueprint docs are in
`docs-old/usage/blueprints.md <https://github.com/dimensionalOS/dimos/blob/main/docs-old/usage/blueprints.md>`_.

Agent CLI and MCP
=================

The :program:`dimos` CLI manages the full lifecycle — run blueprints, inspect state, interact
with agents, and call skills via :abbr:`MCP (Model Context Protocol)`.

.. code-block:: bash

   dimos run unitree-go2-agentic --daemon   # Start in background
   dimos status                             # Check what's running
   dimos log -f                             # Follow logs
   dimos agent-send "explore the room"      # Send agent a command
   dimos mcp list-tools                     # List available MCP skills
   dimos mcp call relative_move --arg forward=0.5  # Call a skill directly
   dimos stop                               # Shut down

Full CLI reference:
`docs-old/usage/cli.md <https://github.com/dimensionalOS/dimos/blob/main/docs-old/usage/cli.md>`_.

Using DimOS as a Library
========================

The example below is a simple robot-connection module that publishes a stream of
:class:`~dimos.msgs.sensor_msgs.Image.Image` frames, and a listener that subscribes to them.
DimOS modules are subsystems that communicate using standardized messages over typed
:class:`~dimos.core.stream.In` / :class:`~dimos.core.stream.Out` streams, with remotely
callable methods marked by the :func:`~dimos.core.core.rpc` decorator.

.. literalinclude:: code/index.py
   :pyobject: RobotConnection

.. literalinclude:: code/index.py
   :pyobject: Listener

Compose the modules with :func:`~dimos.core.coordination.blueprints.autoconnect` — which
connects streams by ``(name, type)`` — and run them by handing the resulting blueprint to
:meth:`~dimos.core.coordination.module_coordinator.ModuleCoordinator.build`, then
:meth:`~dimos.core.coordination.module_coordinator.ModuleCoordinator.loop`:

.. literalinclude:: code/index.py
   :pyobject: run_connection
   :lines: 2-
   :dedent:

Blueprints
----------

Blueprints are instructions for how to construct and wire modules. Each
:class:`~dimos.core.module.Module` exposes a :attr:`~dimos.core.module.Module.blueprint` factory, and
:func:`~dimos.core.coordination.blueprints.autoconnect` composes several into a single
:class:`~dimos.core.coordination.blueprints.Blueprint`. Blueprints can be composed, remapped,
or have transports overridden with
:meth:`~dimos.core.coordination.blueprints.Blueprint.transports` when ``autoconnect()`` cannot
resolve conflicting names or message types on its own.

The example below connects the image stream from a :brand:`Unitree` Go2 to an
:abbr:`MCP (Model Context Protocol)`-backed agent for
reasoning and action execution, pinning the ``color_image`` stream onto an explicit
:class:`~dimos.core.transport.LCMTransport`:

.. literalinclude:: code/index.py
   :pyobject: run_agentic_blueprint
   :lines: 2-
   :dedent:

API reference
-------------

See :doc:`api` for the full API reference.

Development
===========

.. code-block:: bash

   export GIT_LFS_SKIP_SMUDGE=1
   git clone https://github.com/dimensionalOS/dimos.git
   cd dimos

   # Run the default test suite (uv run syncs deps on demand).
   uv run pytest --numprocesses=auto dimos

Multi-language support
----------------------

Python is the glue and prototyping language, but many languages are supported via LCM
interop. See the language interop examples for
`C++ <https://github.com/dimensionalOS/dimos/blob/main/examples/language-interop/cpp/>`_,
`Lua <https://github.com/dimensionalOS/dimos/blob/main/examples/language-interop/lua/>`_, and
`TypeScript <https://github.com/dimensionalOS/dimos/blob/main/examples/language-interop/ts/>`_.

Table of Contents
=================

.. toctree::
   :name: mastertoc
   :maxdepth: 2

   api
