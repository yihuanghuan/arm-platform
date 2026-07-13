# AprilTag 36h11 ID 0 Gazebo Model

This model is a static visual AprilTag target for Gazebo Classic.

- Family: `tag36h11`
- ID: `0`
- Detection tag size: `0.20 m`
- Size definition: the `0.20 m` value is the black effective detection edge between AprilTag detection corners, not the full texture plane.
- Texture plane size: `0.30 m x 0.30 m`
- Texture layout: 12 cells across; the 8-cell black effective tag edge is 0.20 m, and the outer white quiet-zone/background occupies the remaining cells.
- Plane normal in model coordinates: `+Z`; the world include pose rotates the model so the tag faces the camera.

## Source

The texture was generated from the upstream AprilRobotics AprilTag `tag36h11.c` code table for ID 0.

- Repository: https://github.com/AprilRobotics/apriltag
- Commit: `0e16a12dd380fd607e4afd54712ee9b1ffb9ec8f`
- Source files consulted: `tag36h11.c`, `apriltag.c`, `README.md`, `LICENSE.md`
- License: BSD 2-Clause, copied in `LICENSE.apriltag.md`
- Generated on: 2026-07-13

Generation used the upstream `apriltag_to_image()` layout semantics:
`width_at_border=8`, `total_width=10`, `reversed_border=false`, plus one additional white quiet-zone cell around the rendered 10-cell tag image.
