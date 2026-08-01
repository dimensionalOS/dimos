# Static Spatial Benchmark Data Terms

## Distribution decision

Structured3D requires users to accept its Terms of Use before downloading the dataset. The terms limit use to non-commercial research and education and prohibit redistribution of downloaded data, in whole or in part.

Until the Structured3D rights holder grants explicit written permission, keep every scene-derived benchmark artifact behind the same access controls as the source dataset. Do not publish:

- `PointCloud2` maps or other map representations derived from a Structured3D scene;
- normalized or extracted geometry, room regions, openings, or topology;
- scene-level questions, answers, labels, instances, or metadata; or
- manifests and provenance records that expose source-derived scene facts.

Code, schemas, empty examples, predicate definitions, configuration, and documentation that contain no Structured3D data may be distributed under this repository's license. Non-reconstructive aggregate statistics require separate review because the terms do not expressly permit them.

This is a conservative engineering policy, not legal advice. Reclassify an artifact as publicly redistributable only after recording explicit permission from the rights holder or an applicable legal review.

## Sources

- [Structured3D project site](https://structured3d-dataset.org/): dataset downloads require acceptance of the Structured3D Terms of Use.
- [Structured3D repository README](https://github.com/bertjiazheng/Structured3D/blob/master/README.md): the dataset uses separate Structured3D Terms of Use; the MIT license applies to code.
- [Structured3D Terms of Use](https://drive.google.com/open?id=13ZwWpU_557ZQccwOUJ8H5lvXD7MeZFMa): use is limited to non-commercial research and education, and downloaded data may not be redistributed in whole or in part.

## Release requirement

Release validation and generation documentation must label all scene-derived public and oracle bundles as gated. A release manifest must not claim public redistribution permission unless it cites the written authorization or legal decision that supersedes this policy.
