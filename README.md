# Bandura Physics Engine

Bandura is a small C physics engine for rigid-body simulation. It provides a
minimal API for dynamic and static bodies, simple collision handling, impulses,
and ray casts, with demo scenes built on top of raylib.

It ships as a C library with a compact interface in `include/bandura.h`
and a Zig build that also assembles interactive demos in `demos/scenarios`.

The engine is built on top of [Cyclone Engine](https://github.com/idmillington/cyclone-physics) and with the help of "Game Physics Engine Development" book by Ian Millington. 

## Install and Build

Requirements:

- Zig toolchain (to build the project)
- C99 compiler (for API consumers)

From the repo root:

```bash
zig build
```

That builds and installs artifacts into `zig-out/`:

- `zig-out/lib/libbandura.so` (shared library)
- `zig-out/include/bandura.h` (installed header)

## Use the Library (C API)

Include the header and link against `libbandura`:

```c
#include "bandura.h"

int main(void) {
  physics_config config = physics_default_config();
  physics_world *world = physics_init(&config);

  // Ground plane and a falling sphere
  (void)physics_add_plane(world, (v3){0, 0, 0}, (v3){0, 1, 0});
  body ball = physics_add_sphere_dynamic(world, 3.0f, 0.5f);
  *ball.position = (v3){0, 5, 0};

  // Give the ball an upward impulse, then step the simulation
  physics_apply_impulse(world, ball.handle, (v3){0, 12, 0});

  for (int i = 0; i < 120; ++i) {
    physics_step(world, 1.0f / 120.0f);
  }

  v3 pos = physics_get_position(world, ball.handle);
  // use pos.* as needed

  physics_teardown(world);
  return 0;
}
```

## API Snapshot

- `physics_default_config` / `physics_init` / `physics_reset` / `physics_teardown`
- `physics_add_box_*`, `physics_add_sphere_*`, `physics_add_cylinder_*`,
  `physics_add_plane`, `physics_add_compound_body_*`
- `physics_apply_force`, `physics_apply_impulse`, `physics_awaken_body`
- `physics_step`, `physics_get_position`, `physics_get_rotation`,
  `physics_raycast`

## Demo Scenarios

The `demos/scenarios` folder contains ready-to-run examples and is a good starting
point for writing your own scenes.

## License

This project is licensed under the Zlib License. See [`LICENSE`](LICENSE).
