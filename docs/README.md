# Current outline
This is subject to change.
* Introduction
* How to play Hypermine
    * Controls
    * Config file
    * The save file (with a warning that compatibility between versions of Hypermine are not guaranteed and another warning to back up save files)
    * Setting up multiplayer
* Background math
    * Linear algebra
        * (This can link to external resources, but readers should be guided on what parts of linear algebra are worth learning, and making these docs self-contained would be a good long-term goal. If we do use external links, we should include a date so that readers know when health of each link was last checked.)
        * Vectors
        * Matrices, matrix-vector multiplication, and its meaning
        * Matrix-matrix multiplication and its meaning
            * (We should likely explain both the "transformation" and "change of basis" interpretations)
        * 3D examples (assuming previous sections have used 2D examples)
        * Dot products
        * Projections, reflections, rotations
        * Homogeneous coordinates and translations
    * Spherical geometry
        * (We want such a section because it's a good segue to problem solving techniques for hyperbolic geometry problems)
        * Representing points as unit vectors
        * Projections, reflections, rotations, translations (which are rotations expressed differently)
    * Hyperbolic geometry
        * Minkowski space (with the "inner product" as the main difference)
        * Representing points as "normalized" vectors
        * Projections, reflections, rotations, translations, horo-rotations
        * A note about floating point precision
        * (Do we put advanced shape-casting math here? Probably not.)
* Tiling the world
    * Nodes and their coordinate systems (describing the order-4 dodecahedral honeycomb and how Hypermine uses it)
    * Chunks and their coordinate systems (explaining node_to_dual and dual_to_node)
    * Voxels within chunks (Margins should be mentioned here.)
    * The graph (how dodecahedra are organized into the tiling in code. Also mention how NodeId works.)
* World generation (See `world_generation.md`)
* How world generation is driven (wait until this section to describe anything async or anything related to margins)
* Character physics (May want to mention the word "player" for searchability)
    * The movement algorithm (generally describing character_controller/mod.rs)
    * Constraining movement vectors (describing vector_bounds.rs)
    * Sphere casting (how collisions are actually detected)
* Block updates (placing and breaking blocks)
* Entity/Component/System
    * (Note that we use hecs, and point to documentation. Provide some tips on how to discover entities/components in use)
    * (Do not list out all compoenents, as that kind of documentation belongs in code)
* Netcode
    * Introduction (note that we use quinn, and point to documentation)
    * Syncing character movement
    * Syncing chunk data and block updates
    * Syncing entities
    * Logging on and off
* Rendering
    * Introduction (note that we use Vulkan with the ash library, and point to documentation)
    * Chunk rendering (Enable reader to trace through all the code involved in rendering a chunk. Include fog.)
    * Surface extraction (including ambient occlusion)
    * Character rendering (with mesh.vert and mesh.frag)
    * Asset loading
    * GUI (note that we use the yakui library, and point to documentation)
* Miscellaneous
    * (Note important entry points, such as core.rs for starting up Vulkan and window.rs for the event loop)
* FAQ (placeholder to put questions if they do appear often)

# Design guidelines
* Docs should live in the repository itself rather than a separate wiki.
    * This helps keep docs in sync with the code and allows the quality of docs to be enforced with pull requests. The wiki has a disadvantage of forcing users to make unilateral edits to it.
* Docs should have a suggested linear order to read them.
* We should provide documenting things that are impossible to learn elsewhere.
* There should be a way for readers to know if it's safe to skip a section.
    * This can for instance be done with "After reading this section, readers should be able to (...). This could be questions they can answer.
* Since math is heavily involved, exercises can be useful as knowledge checks.
* While there is a linear order, readers should also be able to tell what parts they can skip if they are reading the docs for a particular purpose.
* We should be prepared to keep placeholders in the docs, potentially with links to external sources, as this would allow us to separate the tasks of writing and organizing documentation.
* Since geometry is heavily involved, the docs should contain pictures.
    * Interactive elements would also be helpful.
* As 3D geometry can be difficult to visualize, we should use a 2D analogy for anything that can be reduced to 2D without loss of generality. For instance, many diagrams that explain concepts can be 2D.
    * It should still be made clear how the analogy extends to 3D.
* To avoid running up against GitHub limits or making repositories take longer to clone, larger images and videos will need to be generated on the reader's machine.
    * One option would be to add functionality to Hypermine itself for these visualizations. It is not decided what the preferred approach is.
    * Images that don't take up much storage are fine to store with Git LFS. Try to keep the total size of all assets used in the documentation under approximately 1 MiB (subject to change based on feasibility/importance of larger images). It should be possible to use the SVG file format to keep most images very small. Producing such images with code is still recommended.
* We should try to keep the reader interested/motivated, making the documentation enjoyable to read.
    * Animations and interactive visualizations can help with this a lot (in addition to helping the learning process).
* If some information in the docs can be made concrete by pointing to source code, we should do that.
    * There could be a desync, but if such a mistake is made, readers can git blame the documentation to see what the code was like when the documentation was written.
    * We can also try to avoid repeated work by referencing code comments, asking readers to check them for further detail. However, we cannot put diagrams in code comments.
    * We should keep this documentation relatively short, especially when diagrams are not needed, as code comments are better. The documentation here should generally just give people less familiar with the codebase somewhere to look.
