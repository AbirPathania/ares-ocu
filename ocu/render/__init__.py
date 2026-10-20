"""A real-time 3D renderer for the cameras.

The numpy camera in `core/camera.py` draws smoothed height fields. This draws the
objects that are actually in the world, as geometry, on the GPU: terrain with
lighting and a shadow map, individual rocks, trees and bushes, a sky, and
atmosphere. Nothing here decides what is in the world; it draws
`TerrainWorld.boulders`, `.trees` and `.bushes`, the same lists whose
footprints the LiDAR map and the route planner are made from, so what a camera
shows and what the rover avoids are one thing.

No Qt import belongs in this package.
"""
