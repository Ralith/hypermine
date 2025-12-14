# World generation
World generation in Hypermine is constrained the following principles:
* Consistency: The contents of a chunk when it is first visited should be determined only by the chunk's location and the world generation parameters, which never change within a world.
* Isotropy: World generation should be as isotropic as reasonably possible, so that it is not obvious which direction leads to the origin.

These constraints help inspire the details of the world generation algorithm, which are explained further in the sections below.

## Noise
Hypermine relies on a random noise function for multiple purposes, such as the deciding the shape of the terrain and what material the ground should be. Because of this, to understand Hypermine's world generation, it's important to understand the noise function it uses.

Hyperbolic space has a unique challenge compared to Euclidean space: Common approaches to generating good-looking terrain, such as fractal noise, rely on using grids of multiple different scales, while in hyperbolic space, grids cannot be arbitrarily scaled. This would cause terrain features to have a limited size if generated this way. To avoid this limitation, Hypermine uses a different approach specifically designed for a hyperbolic tiling, inspired by Hyperrogue, which is described in the following section.

### Coarse noise function output
The first step is to form a coarse approximation of the noise function, deciding on one value for each node. To do this, we take advantage of the tiling itself. For a 2D analogy, the pentagonal tiling of the hyperbolic plane can be thought of as a set of lines dividing the hyperbolic plane instead of individual pentagons.

TODO: Picture of pentagonal tiling with lines colored to distinguish them from each other

Similarly, in 3D, the dodecahedral tiling can be thought of as a set of planes dividing hyperbolic space. This interpretation of the dodecahedral tiling is important for understanding how the noise function works between nodes.

To decide on a noise value for each node, we break the dodecahedral tiling up into these planes. We associate each plane with a randomly chosen noise delta, such that crossing a specific plane in one direction increases or decreases the noise value by a specific amount, and crossing the same plane from the other side has the opposite effect. Once we decide on a noise value for the root node, this definition fully determines the noise value of every other node.

The following diagram shows an example of the 2D equivalent of this algorithm.

TODO: Picture of pentagonal tiling with each line labeled with the noise delta, using arrows or something similar to show how this offset applies. The center of each pentagon is also labeled with a number with its current noise value. Integers are used everywhere to allow the reader to verify the math easily in their head.

In this diagram, the randomly-chosen noise delta of each line, along with the derived noise value of each node, is shown. Note how the difference in noise values between any two adjacent nodes matches the noise delta of the line dividing them.

This algorithm allows for random variation while keeping nearby nodes similar to each other, which is what we need from a noise function. One notable quirk worth mentioning is that the noise value is unbounded, which currently means that hills and valleys in Hypermine can become arbitrarily high and deep, respectively.

### Fine noise function output
The next step is to use this coarse approximation of the noise function to produce the actual noise function. To begin, set the noise value at the center of each node to the coarse output we computed earlier.

TODO: Picture of pentagonal tiling with a color at the center of each node to represent the noise value.

Then, trilinearly interpolate these values based on voxel coordinates to create a continuous function across the world. Note that in 2D, this would be bilinear interpolation.

TODO: Picture of the same pentagonal tiling with gradients added. Decorate the center of each node with a dot to highlight the control points of the interpolation.

Finally, for each voxel, add a random offset to its noise value, drawn independently from some distribution.

TODO: Picture of same pentagonal tiling with the final noise value of each voxel.

## Terrain shape
Hypermine uses the 3D noise function to determine the shape of the terrain. This may seem surprising, as it is arguably simpler to use a 2D noise function instead to form a heightmap of the world. However, using 3D noise instead is a useful way of generating more interesting terrain with overhangs, and more importantly, a 2D heightmap works less well in hyperbolic space because of the way that space expands as you move away from the ground plane. The naive approach would cause hills and valleys to have significantly less detail than terrain near the ground plane.

Instead, the basic algorithm is as follows: Using a 3D noise function, we determine the hypothetical elevation of the terrain at each voxel. We then subtract this elevation by the actual height of the voxel above the ground plane to determine a value (denoted `dist` in code) that can be roughly translated to "the voxel's depth relative to the terrain's surface". If this value is above zero, we are inside the terrain, and the voxel should be solid, and otherwise, it should be void. If the value is above zero but close to zero, one of the surface materials (like dirt) will be used, while if the value is far above zero, a material like stone will be used instead.

Note that the above is a simplification of the actual algorithm. It is recommended to read the implementation of `worldgen::ChunkParams::generate_terrain` to understand all the details. For instance, terracing is used to add flatter terrain layers with steeper hills between them, and the strength of this terracing effect is controlled by another parameter affected by noise called `blockiness`. In addition, some measures are taken to make the terrain surface smoother than the interface between the different terrain layers.

## Terrain material
The terrain material depends entirely on the following four factors:
* Elevation above the ground plane
* Estimated distance below the terrain surface
* Temperature
* Rainfall

Note that temperature and rainfall are noise functions set up for the purpose of allowing varying terrain materials. How these are used is described in detail in `terraingen.rs`, so the details are omitted here.

## The road
Currently, the only megastructure in Hypermine is a single infinite straight road. This megastructure works by using the state machine `worldgen::NodeStateRoad`, which is much like `worldgen::NodeStateKind`, but instead of defining the ground plane, it defines the plane that divides the road into its "east" and "west" sides (with this terminology assuming that the road runs "north" and "south"). Based on this state, the `worldgen::ChunkParams::generate_road` function will be called for chunks right above the ground plane, generating the road's surface and carving out any terrain that is in the way. The `worldgen::ChunkParams::generate_road_support` function will be called for chunks below the ground plane, generating the wooden truss if the road is above the terrain, acting as a bridge.

## Trees
For decoration, tiny two-block trees are scattered throughout the terrain, with their density depending on the amount of rainfall. Each tree is generated by placing a wood block next to a dirt or grass block, followed by placing a "leaves" block on that wood block, away from the dirt or grass block. While this algorithm is unaware of gravity, it often generates trees upright because the wood blocks are placed on relatively flat ground. See `worldgen::ChunkParams::generate_trees` for more details.

Note that generating larger trees requires a more complicated algorithm that has not yet been planned out or implemented.

## Random number generation
The above sections mention random choices being made in several areas, but Hypermine requires any given chunk to always be generated with the same contents no matter when it is generated and no matter which computer is used. To accomplish this, Hypermine uses a deterministic and portable random number generator (RNG), seeding it for each node and chunk.

To elaborate, each node is given a "spice" value, which is set to the last 64 bits of the node's unique `NodeId`. This node spice is used directly as the seed for an RNG, which is used for non-chunk-specific decisions, such as the noise deltas between nodes. The node spice is then hashed together with the chunk's `Vertex` to produce the seed for another RNG, which is used for chunk-specific-decisions, such as how trees should be scattered within each chunk.

## Additional information
For additional information related to world generation, it is recommended to read the code and its documentation in `worldgen.rs`, as it has many details not covered here.
