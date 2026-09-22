# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Agentic recorded-memory world blueprints."""

from pathlib import Path

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.hyperspace.module import Hyperspace
from dimos.teleop.memory_world.module import MemoryWorldModule

MEMORY_WORLD_SYSTEM_PROMPT = """You answer questions about a recorded robot memory,
and the person asking is standing inside it in a headset.

Use the `find_in_memory` tool for every question about what the robot saw. It is a
vector-database lookup over the recording's own CLIP/SigLIP image embeddings: it lights
the places it found in the world and hangs the photograph behind each one where the
camera stood. Pass it the thing to look for, not a sentence: "a fire extinguisher", not
"where did I see a fire extinguisher".

For a question about PART of the recording, pass `from_fraction` and `to_fraction`.
They are fractions of the recording's length, 0.0 at the beginning and 1.0 at the end,
so "in the first half" is 0.0 to 0.5 and "near the end" is about 0.8 to 1.0. Leave them
alone for a question about the whole recording.

Each place the tool reports carries `seconds_into_recording`. That, not the order they
come back in, is what answers "the FIRST one" -- the list is ranked by how well each
matched, not by time.

A place is somewhere the thing was SEEN FROM: the pose of the camera that photographed
it. It is not the object's own position, so do not tell the person the thing is at those
coordinates.

To show someone the way, call `navigate_to_place`. `place` is 1 for the first place the
last answer listed. `start` is where to walk from: "recording start" for where the robot
was when the recording began -- which is what "from the starting point" means -- or
"viewer" for where the person is standing now.

Being ASKED TO GO somewhere is two steps, not one. "navigate to the first basket",
"take me to the whiteboard", "walk me to where you saw a person" all mean: call
`find_in_memory` for the thing ("a basket"), then call `navigate_to_place` for the one
they meant. Finding it and describing it is only half of what was asked -- if the person
said to go, the turn is not finished until a route is drawn or you have said why none
could be.

Which place they meant is in the words. "the first" means earliest in the recording and
"the last" means latest, so compare `seconds_into_recording` and pass that place's number
-- NOT `place=1`, which is merely the best match. With nothing to distinguish them, the
best match is a fair choice.

Where to walk FROM is also in the words, and the two are not the same question. "take me
to it", "bring me there", "walk me over" mean from where the person is standing, so
`start` is "viewer" -- they are asking to be led from where they are. Only an explicit
"from the start", "from the beginning" or "from where the robot began" means "recording
start". When they say neither, "viewer" is the better guess: someone standing in the world
asking to be shown something means from here.

The tool's reply tells you how many places it found and how close each was. Report what
it actually found. It can fail three ways that mean different things: NOT_FOUND means
nothing in that stretch of the recording resembles it closely enough, INDEX_NOT_READY
means the recording has not been embedded yet, and NO_ROUTE means no walkable path
joined the two ends -- say which, rather than reporting nothing was there.

Answering "no" is a real answer. If `find_in_memory` comes back NOT_FOUND for "a person"
over the first half, then there were no people in the first half; say so.

Do not claim anything was shown in the world unless the tool call succeeded.
"""

memory_world_agent = autoconnect(
    # The page's ask box goes to the agent in THIS blueprint, because this is the one that
    # has an agent. `memory-world-module` keeps the direct skill call.
    MemoryWorldModule.blueprint(ask_via_agent=True),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=MEMORY_WORLD_SYSTEM_PROMPT),
).global_config(n_workers=4)


# ---- the hyperspace variant ------------------------------------------------------

# The recording both modules read: MemoryWorld for the map, the timeline and the
# photographs, Hyperspace for the index already built inside it. One file, so the world
# you walk and the world you question cannot drift apart.
HYPERSPACE_RECORDING = str(Path.home() / "datasets" / "lite_recorder" / "grocery.db")

# TWO of the recording's four members, not all four. The full index is ~31.6 GB at fp32
# (7.9 billion numbers) and a 30 GB machine does not fail cleanly on that -- it swaps,
# and every timing taken afterwards is meaningless without looking wrong. These two are
# ~1.5 GB and still give the agreement step the two members it needs. A machine with room
# can pass all four back; the ceiling is RAM, not correctness.
HYPERSPACE_MEMBERS = ["base_patch16_224", "base_patch16_256"]

MEMORY_WORLD_HYPERSPACE_PROMPT = """You answer questions about a recorded robot memory,
and the person asking is standing inside it in a headset.

You have three ways to look, and they differ in what an ANSWER IS rather than in how they
search. Every one of them lights the world up where it found something.

`start_item_query` finds THINGS, with a detector that draws a box around each one and can
say no. Use it for anything countable: "a fire extinguisher", "the red chair". It is the
one to reach for by default. Because a real detector looked, it is allowed to refuse, and
a refusal is a real answer -- it means it looked and declined, which is not the same as
finding nothing.

`start_heatmap_query` finds WHERE SOMETHING IS TRUE, with no detector and nothing to
refuse. Use it when the thing has no edges to box: "somewhere damp", "where the cables
run". `within_m` keeps only what is near the robot.

`start_area_query` finds a PLACE rather than a thing: "the kitchen", "a corridor".

`query_results` walks through what a query found; it does not replay it.

Pass the THING to look for, not the sentence. "a fire extinguisher", never "where did I
see a fire extinguisher".

An answer is WHERE THE THING IS. This is the important difference from the older memory
world, which could only say where a thing had been seen FROM. Here a box is a measurement
of the object itself, so you may tell the person the thing is at those coordinates. Two
limits: a heatmap or area answer has no size, because nothing measured one, so do not
describe its extent; and the distance an answer carries is how far the thing was FROM THE
CAMERA THAT PHOTOGRAPHED IT, which is not how far it is from the person asking. Never say
a thing is "a metre away" on the strength of it -- they are somewhere else entirely, and
the route length is the only number that says how far they must walk. Say "seen from about
a metre" if you say it at all; it is worth saying when it is large, because a box eight
metres out is worth less than the same box at one metre.

How sure a thing is means different things across the three. An item answer carries the
detector's own score for a box it drew. A heatmap or area answer carries the score of a
CELL, which is a different quantity on a different scale -- compare those only against
each other, never against an item's score, and do not call a cell score a detection.

To show someone the way, call `navigate_to_place`. `place` is 1 for the first place the
last answer listed. `start` is where to walk from.

Being ASKED TO GO somewhere is two steps, not one. "take me to the fire extinguisher"
means: find it, then navigate to it. Finding it and describing it is only half of what
was asked -- if the person said to go, the turn is not finished until a route is drawn or
you have said why none could be.

Where to walk FROM is in the words. "take me to it", "bring me there", "walk me over"
mean from where the person is standing, so `start` is "viewer". Only an explicit "from
the start" or "from the beginning" means "recording start". Said neither, prefer
"viewer": someone standing in the world asking to be shown something means from here.

Report what the tools actually found, and answering "no" is a real answer. Do not claim
anything was shown in the world unless the call succeeded.
"""

# One Hyperspace instance, not one per process: the index is the largest thing either
# module touches, and a second copy of it is the difference between fitting in memory and
# not. Everything that wants to ask goes through this one.
memory_world_hyperspace = (
    autoconnect(
        MemoryWorldModule.blueprint(
            ask_via_agent=True,
            store_path=HYPERSPACE_RECORDING,
        ),
        Hyperspace.blueprint(
            db_path=HYPERSPACE_RECORDING,
            detect_models=HYPERSPACE_MEMBERS,
            # Rank frames with the cheapest member over the whole index, then score the rest
            # only on the frames it liked. Measured by hyperspace's author on bike.db: first
            # answer 5.98 s -> 4.33 s with every place the full search finds still there.
            rank_with="auto",
            # NOT the default 400. Cutting to the ranking member's best N frames is faster
            # again -- 2.25 s at 200 -- and it loses REAL answers, not duplicates: on "a
            # traffic light" the 12 baseline places are 8 distinct lights and top200 keeps 5
            # of the 8. The place COUNT hides it, 12 -> 11 reading as one duplicate leaving
            # when it was a whole light leaving and a different duplicate arriving. 0 is the
            # whole index, and this demo is about what it can find, not how fast.
            rank_frames=0,
            # METAL IS OFF FOR THIS BLUEPRINT, MEASURED 2026-09-22. `allow_mps` became a
            # config field defaulting to Metal, on the strength of OWLv2 answering in a real
            # worker at 10.7 s against ~140 s on cpu. IT DOES NOT SURVIVE HERE: with the
            # default, `Hyperspace/start` dies in worker 1 with "Failed to create metal
            # library ... Unable to reach MTLCompilerService", and the demo never comes up.
            # Reproduced identically under `dtk run` and plain `dimos run`, so the supervisor
            # is not the cause -- this blueprint runs four workers and one of the others
            # reaches Metal before Hyperspace's does. The 10.7 s measurement was taken on a
            # smaller stack; it is not wrong, it just does not transfer to this one.
            #
            # So the field stays default-on for stacks where it works, and this demo opts
            # out. Turning it back on here means reproducing the failure above first.
            allow_mps=False,
            #
            # The numbers below are all CPU numbers and the cut below is argued from a CPU
            # tail. On Metal that arithmetic changes and 12 x 3 may be affordable again --
            # which would buy recall back. NOT CHANGED HERE, because that is a measurement
            # nobody has made yet, and the cut is what makes today's demo survive its worst
            # case. The detector's cost is
            # max_episodes x detect_attempts forward passes. At the defaults (12 x 3) a
            # grocery.db item query measured 138-274 s on this Mac, which no amount of
            # timeout makes demoable. At 6 x 1 it is 24.7-26.6 s.
            #
            # The speedup is not the main argument; the TAIL is. At 12 x 3 the same
            # query measured 162 s and then 217 s, and across queries 138-274 s, because
            # the cost is episodes x attempts and a REFUSAL spends all of its attempts --
            # so the queries the detector mostly turns down are the expensive ones, which
            # is backwards from what anyone would guess. At 6 x 1 every query here landed
            # in 24.7-26.6 s. A demo dies on the worst case, and the long config's worst
            # case is the part that is unbounded.
            #
            # MEASURED FREE on three queries, comparing the PLACES and not just the
            # count -- the count is exactly what hid hyperspace's own traffic-light
            # regression:
            #   a shopping basket   3 places -> the SAME 3, same scores, same radii
            #   a fire extinguisher 1 place  -> the SAME 1, centre identical to 2 dp
            #   a person            0 places -> 0 either way (the detector refuses)
            # Three queries on one recording is not a proof. An object whose only good
            # look is in the 7th-ranked episode WOULD be lost, and nothing here says so.
            detect_attempts=1,
            max_episodes=6,
            # The MCP server reaches this module over RPC and
            # `ModuleConfig.default_rpc_timeout` caps it -- the 120 s default killed every
            # item query before the cut, and would again on a slower box or a harder word.
            # Not a fix for slowness; a margin so a failure is the query's, not the
            # transport's.
            default_rpc_timeout=600.0,
        ),
        McpServer.blueprint(),
        McpClient.blueprint(system_prompt=MEMORY_WORLD_HYPERSPACE_PROMPT),
    )
    .remappings(
        # Streams join by EFFECTIVE NAME, so `hyperspace_found` and Hyperspace's `found` do
        # not meet on their own -- they would both sit there connected to nothing and the
        # world would simply never light up, with no error to say why. The descriptive name
        # stays on the module, where `found` alone would say nothing about whose.
        [(MemoryWorldModule, "hyperspace_found", "found")]
    )
    .global_config(n_workers=6)
)
