# AppImage data

`data.tsv` lists the LFS archive stems proposed for the production image. Each
row identifies its consumer, the robot or subsystem owner who should review it,
the licence evidence, and any open question. An owner records their name or
review reference in `reviewed_by` after checking the need for the archive and
the terms for its contents. Ticket 04 records project approval for seven rows with
components marked `licence unspecified`. Their upstream terms are recorded
where known. Keep the applicable licence notices and attributions in the image.

Run the inventory check from the DimOS checkout:

```sh
python3 packaging/appimage/check_data.py
```

The total counts compressed payload bytes even in a checkout with Git LFS
pointers. The build runs the following command after fetching the listed
payloads. On success stdout contains one archive path per line, ready for
extraction into the image data directory. On failure the build must stop.

```sh
python3 packaging/appimage/check_data.py --build-inputs
```

MuJoCo provides the hardware-free installation check. Replay recordings,
including the default Go2 and G1 datasets, are supplied separately by the
operator. The image also omits the test assets and LCM fixture, office video,
optional scene packages, the non-commercial person mesh and Ultralytics weights.

The R1 Pro navigation blueprint resolves its robot description from
`userguide-galaxea/URDF` at `2e5d31e1784481a34d178006c0d0e18e0a84a82a`.
The image build stages these Git robot descriptions and external model weights
separately from the LFS archives.
