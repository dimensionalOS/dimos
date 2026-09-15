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

"""Recorded Go2 memory query; all analysis and exports use existing DimOS APIs."""

import json
import os
from pathlib import Path
import sys

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.transform import peaks, significant
from dimos.memory.vis.plot.elements import Series, VLine
from dimos.memory.vis.plot.plot import Plot
from dimos.memory.vis.space.space import Space
from dimos.memory.vis.utils import mosaic
from dimos.models.embedding.clip import CLIPModel


def main() -> None:
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    store = SqliteStore(path=os.environ["DIMCODE_TEST_GO2_DB"], must_exist=True)
    try:
        vector = CLIPModel(device="cpu").embed_text("plant")
        query = store.streams.color_image_embedded.search(vector).order_by("ts").materialize()
        rows = list(query)
        selected = query.transform(peaks(key=lambda obs: obs.similarity, distance=1.0))
        selected = selected.transform(significant(method="mad")).materialize()
        matches = list(selected)
        assert len(rows) > 10 and matches, "Recording must contain scored images and peaks"
        plot = Plot().add(
            Series(
                ts=[row.ts for row in rows],
                values=[row.similarity for row in rows],
                color="#43bda7",
                label="plant similarity",
                connect=7.5,
            )
        )
        for row in matches:
            plot.add(VLine(row.ts, color="#e87c59"))
        plot.to_svg(str(out / "timeline.svg"))
        # The context cloud and selected image poses both come from this recording.
        context = [store.streams.lidar.at(row.ts).last() for row in matches]
        assert all(row is not None for row in context)
        Space().add([row.data for row in context]).add(selected).to_svg(str(out / "space.svg"))
        mosaic(selected, cols=min(3, len(matches)), cell_height=300).data.save(
            str(out / "frames.png")
        )
        report = {
            "query": "plant",
            "source": os.environ["DIMCODE_TEST_GO2_DB"],
            "operation": "search(CLIP plant).order_by(ts).materialize() → peaks(distance=1) → significant(mad)",
            "observations": len(rows),
            "context_cloud_timestamps": [row.ts for row in context],
            "selected": [
                {"id": row.id, "timestamp": row.ts, "similarity": row.similarity} for row in matches
            ],
        }
        (out / "query.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report))
    finally:
        store.stop()


if __name__ == "__main__":
    main()
