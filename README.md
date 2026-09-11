# collision-cluster

A 3D particle simulation in C++. Particles have varying radii and spin, collide
with each other, and fall toward the origin. The sim runs
offline and writes one CSV per frame; Blender reads those CSVs back through a
Geometry Nodes setup and does all the rendering.

There is no realtime renderer here — the C++ side only produces
numbers and Blender is used for rendering. You can see it in action [on my website](https://youssefbiaz.com/workshop/collision-cluster/).

## Build and run

Requires CMake 3.28+, a C++20 compiler, and [Eigen3](https://eigen.tuxfamily.org)
(`brew install eigen` / `apt install libeigen3-dev`).

```sh
cmake -S . -B build
cmake --build build
./build/bin/CollisionCluster        # reads ./config.cfg, writes ./output/
```

Run it from the repo root — the binary looks for `config.cfg` in the current
directory unless you pass a path (`./build/bin/CollisionCluster my.cfg`).
`output/` is wiped and recreated on every run.

That's everything the sim needs. The Blender side additionally wants two CC0
HDRIs that aren't in the repo — see [assets/README.md](assets/README.md).

The checked-in `config.cfg` is the one behind the video in the writeup: 100
particles, 15 seconds at 60fps, 900 frames. It takes a fraction of a second to
simulate; writing the CSVs is the slow part (on my machine, ~0.08s to run the
simulation, and ~0.4s to write the CSVs).

## How it works

**Verlet integration, structure-of-arrays.** State lives in Eigen matrices:
`pos_`, `old_pos_`, `accel_` are all `3 x N`; radius and inverse mass are
length-`N` vectors. This replaces my 2D version, which had an array of per-particle objects. Everything
except collision handling is written as whole-matrix operations so Eigen can
vectorize it. See [AoS and SoA](https://en.wikipedia.org/wiki/AoS_and_SoA).

### Each substep

Each frame runs `substeps` sub-steps. A sub-step zeroes acceleration, calls the
sim's `ApplyForces()`, integrates orientation, then does the Verlet position
update.

**Forces:** `CollisionClusterSim::ApplyForces()` applies a constant-magnitude
acceleration toward the origin (not Newtonian — magnitude does not fall off with
distance; it is there to keep the cluster together) and then resolves
collisions.

**Collisions and spin:** `ApplyPairwiseCollisionsWithSpin()` is the one place
that has to loop, since it is inherently pairwise (O(N^2), no spatial hash).
Overlapping particles are shoved apart along the contact normal, mass-weighted
by inverse mass so heavier particles move less; mass is `r^3`, so equal-radius
particles reduce to an even 0.5/0.5 split.

Spin is deliberately not a real rigid-body impulse response. At each contact,
only the components of angular velocity _perpendicular_ to the contact normal
are blended between the two particles, by a per-substep fraction
`collision_spin_transfer`. Parallel components and linear motion are untouched.
It's a cheap proxy that gives us the illusion of interacting spins without tracking velocities or impulses.

Orientation is a quaternion per particle, integrated as
`dq/dt = 0.5 * (0, omega) * q` and renormalized each sub-step, vectorized across
all particles at once.

**Output.** One CSV per frame in `output/`, named `sim_0000.csv` upward, one row
per particle:

```
x, y, z, r, qw, qx, qy, qz
```

## Configuration

`config.cfg` is `key = value` lines, `#` for comments. Unknown keys are ignored.
`min_radius` and `max_radius` are required.

| key                           | what it does                                                                |
| ----------------------------- | --------------------------------------------------------------------------- |
| `dims`                        | 2 or 3. In 2D, z is pinned to zero.                                         |
| `universe_size`               | Half-extent of the initial spawn box.                                       |
| `init_strategy`               | `random` (uniform in the box) or `spaced` (a regular grid).                 |
| `num_particles`               | Particle count. Ignored by `spaced`, which derives it from `universe_size`. |
| `friction`                    | Linear damping coefficient. `0` is undamped.                                |
| `framerate`, `seconds`        | Total frames written = `framerate * seconds`.                               |
| `substeps`                    | Physics sub-steps per frame. More = stiffer, more stable collisions.        |
| `gravity_to_origin`           | Acceleration magnitude toward the origin. `0` disables it.                  |
| `min_radius`, `max_radius`    | Radii are sampled uniformly in this range.                                  |
| `max_angular_speed`           | Bound on each component of the initial angular velocity (rad/s).            |
| `collision_spin_transfer`     | Per-substep blend fraction for the contact-tangent spin blend.              |
| `angular_friction`            | Per-substep multiplicative damping on angular velocity.                     |
| `write_to_file`, `output_dir` | Whether to write CSVs, and where.                                           |

## Layout

```
CMakeLists.txt
config.cfg
src/
  main.cc                       load config, run, write
  config_loader.h               config.cfg parser
  particle_config.h             base config + derived fields
  particle_engine.h             CRTP base: state, init, sub-step loop, capture
  spinning_particle_config.h    spin-specific config
  spinning_particle_engine.h    CRTP layer adding orientation + angular velocity
  particle_collisions.h         pairwise collision + spin blend
  collision_cluster_config.h    this sim's config: gravity_to_origin
  collision_cluster_sim.h/.cc   this sim: forces and collision call
  file_io.h                     per-frame CSV writer
```

The engine is a CRTP template so a sim only has to supply `ApplyForces()`;
spin is a separate layer on top of the base engine rather than baked into it.

## Blender side

Feel free to open the file and play around! If you want to create your own
version, feel free to make the PointCSVImporterNode a reusable asset.

**Before you open the .blend, you need to download two HDRIs** — they're CC0 but
too big to commit, so `assets/` ships with just a
[README](assets/README.md) pointing at them. Grab both, drop them in `assets/`
under the exact filenames listed there, or the world lighting comes up pink.

Geometry Nodes reads `sim_####.csv` for the current frame and
instances a mesh at each row's position, scaled by `r` and rotated by the
quaternion. Because Blender re-reads the CSVs every frame, re-running the sim
with new parameters shows up immediately in the viewport with no re-import.

## Possible future work

- Spatial hashing / uniform grid to get collisions off O(N^2).
- Parallelizing the collision pass (it is the only serial part left).
- Audio-reactive parameters driven from outside Blender.

## Credits

Inspired by [Pezzza's Work](https://www.youtube.com/@PezzzasWork) and his Verlet
collision videos. Built on [Eigen](https://eigen.tuxfamily.org) (MPL2).

The two HDRIs the .blend lights with are both CC0 (public domain — no
attribution required, credited here anyway). Neither is committed;
[assets/README.md](assets/README.md) has the download instructions:

- **Night Sky HDRI 007** — [ambientCG](https://ambientcg.com/view?id=NightSkyHDRI007), CC0.
- **White Home Studio** by Grzegorz Wronkowski —
  [Poly Haven](https://polyhaven.com/a/white_home_studio), CC0.
