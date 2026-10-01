# How to Play
## Initial Setup
### Precompiled version
Hypermine has precompiled builds for Linux and Windows. The simplest way to obtain Hypermine is to download a precompiled release at <https://github.com/Ralith/hypermine/releases>. There will be a zip file in the "Assets" section for your operating system. Once you download the zip, unzip it and run "client" (or "client.exe" on Windows), and the game should launch.

<!-- TODO: Include this documentation in compiled releases and suggest that the user read that documentation to ensure that documentation changes discussing unreleased functionality don't cause confusion. Note that we likely don't need to include most documentation in compiled releases, since the target audience of most documentation will be contributors, not players. -->

### Building from source
To build Hypermine, you need the following dependencies:
- The Rust compiler (<https://rust-lang.org/tools/install/>). Note that on Windows, this requires Build Tools for Visual Studio (<https://rust-lang.github.io/rustup/installation/windows-msvc.html>).
- Git (<https://git-scm.com/>)
- Git Large File Storage (<https://git-lfs.com/>)
- shaderc (See <https://github.com/google/shaderc-rs?tab=readme-ov-file#setup> for how to set it up)

To build Hypermine, first clone the repo with `git clone https://github.com/Ralith/hypermine.git`. Then, go into the `hypermine` directory and run `git lfs pull` to ensure that you have all the assets Hypermine depends on.

#### Troubleshooting
Before spending too much time troubleshooting Hypermine when building it from source, it is recommended to try running a precompiled binary to make sure that your machine meets the system requirements.

A common issue is that a dependency is missing, or a tool cannot find it because it is missing in PATH. For example, Git needs to be installed and in the PATH. To check whether Git is configured correctly, open a shell and run `git --version`.

If you receive an error about being unable to find a command like "cmake", "python", or "ninja", this is likely because the build process failed to find shaderc and tried to build it from source as a fallback. The recommended fix is to install the Vulkan SDK, which has shaderc bundled with it. Some Linux distributions also allow you to install shaderc from their package manager. Please refer to <https://github.com/google/shaderc-rs?tab=readme-ov-file#setup> for details.

## Controls and Gameplay
Hypermine currently allows you to fly or walk around, as well as placing and breaking blocks.

The game saves automatically, although the save file format will likely change between versions in incompatible ways. Since this game is still a work in progress, backing up save files is highly recommended. On Linux, save files are by default located in `~/.local/share/hypermine/`, and on Windows, they are located at `%LOCALAPPDATA%\hypermine\data\`.

The controls are as follows:
- Move mouse - look (may need to left-click in the game window to grab cursor first)
- `w`, `a`, `s`, `d` - move
- Spacebar - jump
- `v` - toggle no-clip mode (gravity and collision are turned off when no-clip mode is on)
- `q`/`e` - rotate (in no-clip mode)
- `r`/`f` - fly up/down (in no-clip mode)
- Left mouse button - delete block
- Right mouse button - place block
- Number row - select material to place
- Escape - release cursor

## Configuration
Although Hypermine does not currently provide any menus for changing settings, it does read from a config file to allow you to change ways in which the game works. To change the configuration, you will need to create a file called "client.toml". On Linux, this file should be located at `~/.config/hypermine/`, and on Windows, it should be located at `%APPDATA%\hypermine\config\`.

Copy the following contents into the config file, and uncomment and edit any lines that you want to change. The comments within the file should help explain what each relevant option does:
```toml
# The `name` option affects some log entries and identifies you if you connect to a server. Defaults to "player".
#name = "player"

# Provides a way to manually specify a directory to search for assets for the game, such as the "materials" folder. Setting this is generally unnecessary.
#data_dir = "..."

# The name of your save file. Specify a full file path to put it in a non-default location. Defaults to "default.save".
#save = "default.save"

# How many chunks to generate terrain for at the same time. Reducing this can improve performance at the cost of making terrain slower to load. Changing this is generally unnecessary.
#chunk_load_parallelism = 256

# If you set the `server` option to an IP address, Hypermine will connect to that server instead of running in single player.
#server = "[::1]:1234"

# Options under `[local_simulation]` affect gameplay and are likely the most interesting to experiment with.
[local_simulation]

# Number of simulation steps per second. Defaults to 30.
#rate = 30

# Maximum distance at which nodes will be rendered in meters. Setting this too high relative to chunk_generation_distance will cause lag. Defaults to 75.
#view_distance = 75

# Maximum distance at which new chunks will be generated in meters. Setting this too high will cause lag and/or errors. Defaults to 60.
#chunk_generation_distance = 60

# Distance at which fog becomes completely opaque in meters. Defaults to 90.
#fog_distance = 90

# Advanced option related to netcode. It represents the amount of time after the first input in a given uninterrupted sequence of inputs the server must wait before beginning to consume inputs. Changing this is generally unnecessary.
#input_queue_size_ms = 50

# The width of a single chunk in voxels. If you decrease this, you will also need to proportionally decrease chunk_generation_distance and view_distance, as well as ensuring that block_reach is less than chunk_generation_distance, to avoid lag and/or errors. If you have an existing save file, changing this will have no effect. Defaults to 12.
#chunk_size = 12

# An approximate average size of each voxel in meters. Defaults to 1.
#voxel_size = 1

[local_simulation.character]
# Movement speed in m/s during no-clip. Defaults to 12.
#no_clip_movement_speed = 12

# Maximum movement speed while on the ground in m/s. Defaults to 4.
#max_ground_speed = 4

# Artificial speed cap in m/s to avoid overloading the server. Defaults to 30.
#speed_cap = 30

# Maximum ground slope (0=horizontal, 1=45 degrees). Defaults to 1.73 (60 degrees).
#max_ground_slope = 1.73

# Acceleration while on the ground in m/s^2. Defaults to 20.
#ground_acceleration = 20

# Acceleration while in the air in m/s^2. Defaults to 2.
#air_acceleration = 2

# Acceleration of gravity in m/s^2. Defaults to 20.
#gravity_acceleration = 20

# Air resistance in (m/s^2) per (m/s); scales linearly with respect to speed. Defaults to 0.2.
#air_resistance = 0.2

# How fast the player jumps off the ground in m/s. Defaults to 8.
#jump_speed = 8

# How far away the player needs to be from the ground in meters to be considered in the air. Defaults to 0.2.
#ground_distance_tolerance = 0.2

# Radius of the player in meters. Defaults to 0.4.
#character_radius = 0.4

# How far a player can reach in meters when placing blocks. Defaults to 10.
#block_reach = 10
```

## Multiplayer
TODO: Explain
