SnakeLib Getting Started 
========================

To view this documentation, first install grip:

`pip install grip`

Then start the grip server and navigate to the server address with a browser: 

`cd docs`

`grip` 

Tutorials for specific snakes. 
- [EELS](./eels.md)

# Adding terrains to PyBullet 

A collection of terrains are available in `snakelib_description/terrain`. The recommended way to add custom terrains is using Blender, then exporting the mesh as an .obj file, then creating a URDF that loads that .obj file. Note that the mesh may need to be denoted as concave in the URDF (see the examples under `terrain`). You may then provide the path to the terrain in `snakelib_bullet/param/sim_params.yaml`.

We have tried to use OpenSCAD in the past to create STL meshes, but had errors when loading OpenSCAD's STLs into PyBullet. 
