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

"""List the licence of every Python package in a site-packages folder.

licences.py <site-packages> <out.tsv>

Writes one row per package and prints the rows a person has to look at: no
licence in the metadata, or a licence with conditions on redistribution.
"""

import csv
from importlib.metadata import Distribution, distributions
import re
import sys

REVIEW = re.compile(r"GPL|proprietary|commercial|nvidia|other|unknown", re.IGNORECASE)


def licence(dist: Distribution) -> str:
    meta = dist.metadata
    if meta.get("License-Expression"):
        return str(meta["License-Expression"])
    classifiers = [
        value.split(" :: ")[-1]
        for value in meta.get_all("Classifier", [])
        if value.startswith("License ::")
    ]
    if classifiers:
        return ", ".join(classifiers)
    # Some packages put the whole licence text in this field.
    lines = (meta.get("License") or "").strip().splitlines()
    return lines[0][:80] if lines else ""


def main() -> None:
    site_packages, out = sys.argv[1:]
    packages = sorted(
        (dist.name.lower(), dist.version, licence(dist))
        for dist in distributions(path=[site_packages])
    )
    with open(out, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, delimiter="\t", lineterminator="\n")
        writer.writerow(("package", "version", "licence"))
        writer.writerows(packages)
    review = [row for row in packages if not row[2] or REVIEW.search(row[2])]
    print(f"{len(packages)} Python packages, {len(review)} to review:")
    for name, version, text in review:
        print(f"  {name} {version}: {text or 'no licence in the metadata'}")


if __name__ == "__main__":
    main()
