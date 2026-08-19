# Bandura physics engine

Bandura is a traditional Ukrainian [musical instrument](https://en.wikipedia.org/wiki/Bandura) and also a simple and fast 3D physics engine.

## Table of contents

* [Features](#features)
* [Installation and build](#installation-and-build)
* [Limitations](#limitations)
* [Using the library](#using-the-library)
  * [Quickstart](#quickstart)
  * [Spawning bodies](#spawning-bodies)
  * [Referencing bodies](#referencing-bodies)
  * [Bounding the bodies together](#bounding-the-bodies-together)
  * [Using meshes as body shapes](#using-meshes-as-body-shapes)
  * [Querying the physics world](#querying-the-physics-world)
  * [Reacting to collisions](#reacting-to-collisions)
* [Configuring the engine](#configuring-the-engine)
* [Handling errors](#handling-errors)
* [Handling simulation time steps](#handling-simulation-time-steps)
* [Bring your own...](#bring-your-own)
  * [Math](#math)
  * [Memory allocator](#memory-allocator)
* [License](#license)
* [LLM Usage](#llm-usage)

## Features 

* **Portability**. Written in C99, so it can be used in any project that supports the C ABI.
* **No dependencies**. The library itself is self-contained, so no third-party code is included.
* **Data-oriented**. The data layout inside the engine allows for efficient CPU cache utilization, enabling high performance.
* **Cross-platform**. Works on Linux, MacOS and Windows.
* **Easy to use**. Include `bandura.h` for the core API and compile `bandura.c` with your other source files. No need to build or link the library.
* **Rigidbody physics simulation**. Allows for dynamic simulations with different shapes, including primitives, compounds and triangular meshes. Supports forces, impulses, rotations and joints.
* **Impulse-based collision resolution**. Objects respond to the collisions based on their mass, shape and collision velocity.

## Limitations

* **No concave meshes support yet**. Currently the library can only use convex meshes as collision shapes. Support for concave ones will be added in the future.
* **Only discrete collision detection**. Object that move very fast can pass through other objects. This will be fixed in the future versions.
* **Not thread-safe**. All library APIs can only be called from a single thread. This limitation will also be lifted in the future.

## Installation and build

To integrate Bandura into your project, take these two files from the repository:
- `bandura.c`
- `include/bandura.h`
Compile `bandura.c` along with your other source files with a compiler and settings of your choice. You don't need to build the library yourself and link it, it's just two files. 

Optionally you may want to also grab `include/bnd-math.h`. This file contains some math helper functions if you need them.

## Using the library

The examples below use `bnd-math.h` for helpers such as `bnd_v3_zero`, `bnd_v3_up`, `bnd_v3_add`, and `bnd_quat_identity`.

### Quickstart

```c
#include "bandura.h"
#include "bnd-math.h"

int main() {
  // Create default config object.
  bnd_config config = bnd_default_config();

  // Initialize the physics world. This will be the root object for physics operations.
  bnd_world *world = bnd_init(config);

  // Add a ground plane that passes through zero and has a (0, 1, 0) normal vector.
  bnd_add_plane(world, bnd_v3_zero(), bnd_v3_up());

  // Add a sphere with mass 5 and radius 1.
  bnd_body_handle sphere = bnd_add_sphere_dynamic(world, 5, 1).value;

  // Lift it up above the ground.
  bnd_set_position(world, sphere, (bnd_v3) {0, 5, 0});

  // Simulate for 1 second.
  for (int i = 0; i < 120; ++i) {
    bnd_simulate(world, 1.0 / 120);
  }

  // Free the internal memory in the end.
  bnd_teardown(world);
  return 0;
}
```
### Spawning bodies

Rigidbodies in Bandura can be of two types: **dynamic** and **static**. As the naming suggests, the former can move and respond to collisions, while the latter serve as static environment. Primitive shapes expose both variants, for example:

```c
bnd_result_handle bnd_add_box_dynamic(bnd_world *world, float mass, bnd_v3 size);
bnd_result_handle bnd_add_box_static(bnd_world *world, bnd_v3 size);
bnd_result_handle bnd_add_sphere_dynamic(bnd_world *world, float mass, float radius);
bnd_result_handle bnd_add_sphere_static(bnd_world *world, float radius);
```

Note that the static version doesn't require to specify mass, since static objects are treated as having _infinite_ mass.

There is also a more generic method that allows you to specify the type and shape of the body as parameters:

```c
bnd_result_handle bnd_add_primitive_body(bnd_world *world, bnd_body_type type, bnd_shape_type shape_type, bnd_shape shape, float mass);
```

The returned value `bnd_result_handle` is wrapper around the body handle with an error attached. The actual body handle is stored in its `value` field. The handle can then be used to refer to that body.

```c
#include "bandura.h"
#include "bnd-math.h"

int main() {
  // ...

  // Add a static box and set it's position. The rotation will be initialized to identity.
  bnd_body_handle static_box = bnd_add_box_static(world, (bnd_v3){10, 2, 3}).value;
  bnd_set_position(world, static_box, (bnd_v3){0, 1, 0});

  bnd_body_handle dynamic_sphere = bnd_add_sphere_dynamic(world, 5, 1).value;
  bnd_set_position(world, dynamic_sphere, (bnd_v3){5, 10, 5});
  bnd_set_velocity(world, dynamic_sphere, (bnd_v3){0, 5, 0});
}
```

Bandura also supports combining multiple primitive shapes into a single body. This allows for creation of complex shapes without using more expensive meshes.

```c
  // Make a dumbell shape with a rotated capsule and two spheres.
  bnd_body_shape shapes[] = {
    (bnd_body_shape) { 
      .type = BND_CAPSULE,
      .value = { .capsule = { .radius = 0.3, .height = 3 } },
      .offset = bnd_v3_zero(),
      .rotation = (bnd_quat) { sinf(PI / 4), 0, 0, cosf(PI / 4) } 
    },
    (bnd_body_shape) {
      .type = BND_SPHERE,
      .value = { .sphere = { .radius = 0.7 } },
      .offset = (bnd_v3){0, 0, 1.5},
      .rotation = bnd_quat_identity()
    },
    (bnd_body_shape) {
      .type = BND_SPHERE,
      .value = { .sphere = { .radius = 0.7 } },
      .offset = (bnd_v3){0, 0, -1.5},
      .rotation = bnd_quat_identity() 
    }
  };

  // Masses correspond to the shapes: `3` for the capsule, `5's` for the spheres.
  float masses[] = { 3, 5, 5 };

  // As a last parameter the function takes the number of shapes in the arrays.
  bnd_body_handle dumbell = bnd_add_compound_body_dynamic(world, shapes, masses, 3).value;
  bnd_set_position(world, dumbell, (bnd_v3){15, 10, 0});
```

### Referencing bodies

Using `bnd_body_handle` from spawning functions you can:

1) **Apply forces, torques and impulses to the body**

```c
#include "bandura.h"

bnd_body_handle sphere_handle;

int main() {
  // ...
 
  sphere_handle = bnd_add_sphere_dynamic(world, 5, 1).value;

  // ... 

  // Apply an upward impulse to the sphere using the previously stored handle.
  bnd_apply_impulse(world, sphere_handle, (bnd_v3){0, 10, 0});
}
```
2) **Querying and updating body's properties**

```c
bnd_v3 sphere_position = bnd_get_position(world, sphere_handle).value;
bnd_v3 sphere_velocity = bnd_get_velocity(world, sphere_handle).value;

bnd_v3 position_delta = bnd_v3_scale(sphere_velocity, delta_time);
bnd_v3 new_position = bnd_v3_add(sphere_position, position_delta);

// This is only to showcase the engine's capability.
// Setting position and velocity of the bodies directly may lead to unrealistic behaviour. Use with caution.
bnd_set_position(world, sphere_handle, new_position);
```

3) **Removing the body**

```c
bnd_remove_body(world, sphere_handle);

// From here on, `sphere_handle` is no longer valid and shouldn't be used.
```

After the body is removed, its corresponding handle is invalidated. You can always check if a handle is valid using `bnd_error bnd_handle_valid(const bnd_world *world, bnd_body_handle handle)`. A valid handle returns `BND_OK`.

### Bounding the bodies together

Bandura supports imposing constraints on the bodies, so that they cannot move further apart than some specified distance. This can be used, for example, to create ragdolls:

```c
  bnd_body_handle head = bnd_add_sphere_dynamic(world, 3, 0.4).value;
  bnd_set_position(world, head, (bnd_v3){0, 5, 0});

  bnd_body_handle torso = bnd_add_capsule_dynamic(world, 25, 0.3, 0.4).value;
  bnd_set_position(world, torso, (bnd_v3){0, 4.1, 0});

  bnd_body_handle pelvis = bnd_add_capsule_dynamic(world, 20, 0.25, 0.5).value;
  bnd_set_position(world, pelvis, (bnd_v3){0, 3.1, 0});

  bnd_body_handle left_upper_leg = bnd_add_capsule_dynamic(world, 10, 0.2, 0.8).value;
  bnd_set_position(world, left_upper_leg, (bnd_v3){0.23, 2, -0.2});
  bnd_set_rotation(world, left_upper_leg, (bnd_quat) { sinf(PI / 12), 0, 0, cosf(PI / 12) });

  bnd_body_handle left_lower_leg = bnd_add_capsule_dynamic(world, 10, 0.2, 0.8).value;
  bnd_set_position(world, left_lower_leg, (bnd_v3){0.23, 0.9, -0.2});
  bnd_set_rotation(world, left_lower_leg, (bnd_quat) { sinf(PI / 12), 0, 0, cosf(PI / 12) });

  bnd_body_handle right_upper_leg = bnd_add_capsule_dynamic(world, 10, 0.2, 0.8).value;
  bnd_set_position(world, right_upper_leg, (bnd_v3){-0.23, 2, 0});

  bnd_body_handle right_lower_leg = bnd_add_capsule_dynamic(world, 10, 0.2, 0.8).value;
  bnd_set_position(world, right_lower_leg, (bnd_v3){-0.23, 0.7, 0});

  bnd_body_handle left_upper_arm = bnd_add_capsule_dynamic(world, 10, 0.1, 1).value;
  bnd_set_position(world, left_upper_arm, (bnd_v3){0.4, 3.9, -0.4});
  bnd_set_rotation(world, left_upper_arm, (bnd_quat) { sinf(PI / 10), 0, 0, cosf(PI / 10) });

  bnd_body_handle left_lower_arm = bnd_add_capsule_dynamic(world, 10, 0.1, 1).value;
  bnd_set_position(world, left_lower_arm, (bnd_v3){0.43, 3.37, -1.45});
  bnd_set_rotation(world, left_lower_arm, (bnd_quat) { sinf(PI / 4), 0, 0, cosf(PI / 12) });

  bnd_body_handle right_upper_arm = bnd_add_capsule_dynamic(world, 10, 0.1, 1).value;
  bnd_set_position(world, right_upper_arm, (bnd_v3){-0.43, 3.8, 0});

  bnd_body_handle right_lower_arm = bnd_add_capsule_dynamic(world, 10, 0.1, 1).value;
  bnd_set_position(world, right_lower_arm, (bnd_v3){-0.43, 2.63, -0.3});
  bnd_set_rotation(world, right_lower_arm, (bnd_quat) { sinf(PI / 12), 0, 0, cosf(PI / 12) });

  const float joint_margin = 0.1;

  bnd_add_joint(world, head, torso, (bnd_v3){0, -0.4, 0}, (bnd_v3){0, 0.5, 0}, joint_margin);
  bnd_add_joint(world, torso, pelvis, (bnd_v3){0, -0.5, 0}, (bnd_v3){0, 0.5, 0}, joint_margin);

  bnd_add_joint(world, torso, left_upper_arm, (bnd_v3){0.3, 0.45, 0}, (bnd_v3){-0.1, 0.6, 0}, joint_margin);
  bnd_add_joint(world, left_upper_arm, left_lower_arm, (bnd_v3){0, -0.6, 0}, (bnd_v3){0, 0.6, 0}, joint_margin);

  bnd_add_joint(world, torso, right_upper_arm, (bnd_v3){-0.3, 0.45, 0}, (bnd_v3){0.1, 0.6, 0}, joint_margin);
  bnd_add_joint(world, right_upper_arm, right_lower_arm, (bnd_v3){0, -0.6, 0}, (bnd_v3){0, 0.6, 0}, joint_margin);

  bnd_add_joint(world, pelvis, left_upper_leg, (bnd_v3){0.23, -0.5, 0}, (bnd_v3){0, 0.6, 0}, joint_margin);
  bnd_add_joint(world, left_upper_leg, left_lower_leg, (bnd_v3){0, -0.6, 0}, (bnd_v3){0, 0.6, 0}, joint_margin);

  bnd_add_joint(world, pelvis, right_upper_leg, (bnd_v3){-0.23, -0.5, 0}, (bnd_v3){0, 0.6, 0}, joint_margin);
  bnd_add_joint(world, right_upper_leg, right_lower_leg, (bnd_v3){0, -0.6, 0}, (bnd_v3){0, 0.6, 0}, joint_margin);

  bnd_body_handle ragdoll[11];
  ragdoll[0] = head;
  ragdoll[1] = torso;
  ragdoll[2] = pelvis;
  ragdoll[3] = left_upper_arm;
  ragdoll[4] = left_lower_arm;
  ragdoll[5] = right_upper_arm;
  ragdoll[6] = right_lower_arm;
  ragdoll[7] = left_upper_leg;
  ragdoll[8] = left_lower_leg;
  ragdoll[9] = right_upper_leg;
  ragdoll[10] = right_lower_leg;
```

### Using meshes as body shapes

When primitive shapes or their combinations are not sufficient to represent a collision shape, you can use its mesh directly. In Bandura, this is a two-step process:

1) **Import the mesh data into Bandura**

Meshes have different representations depending on their format, development environment and many other factors. To use them inside the physics engine, they have to be converted into the appropriate format. Here is the function that is responsible for that:

```c
bnd_error bnd_import_mesh(bnd_world *world, const bnd_mesh_data *data, bnd_mesh_handle *handle, bnd_v3 *center_of_mass);
```

  * `bnd_mesh_data` parameter carries information about the mesh's verticies and indicies. It's the user's responsibility to populate it from whatever external mesh format they use. Here is an example of converting the [Raylib's](https://www.raylib.com) mesh:
  ```c
  bnd_mesh_data raylib_mesh_to_bnd(Mesh m) {
    bnd_mesh_data data = {
      .vertex_buffer = { 
        .buffer = m.vertices,
        .element_size = 3 * sizeof(float),
        .elements_count = m.vertexCount,
        .stride = 0 },
    };
  
    if (m.indices != NULL) {
      data.index_buffer.buffer = m.indices;
      data.index_buffer.element_size = sizeof(unsigned short);
      data.index_buffer.elements_count = 3 * m.triangleCount;
      data.index_buffer.stride = 0;
    } else {
      uint32_t *buffer = malloc(3 * m.triangleCount * sizeof(uint32_t));
      data.index_buffer.buffer = buffer;
      data.index_buffer.element_size = sizeof(uint32_t);
      data.index_buffer.elements_count = 3 * m.triangleCount;
      data.index_buffer.stride = 0;
  
      for (int i = 0; i < m.triangleCount; ++i) {
        buffer[3 * i + 0] = 3 * i + 0;
        buffer[3 * i + 1] = 3 * i + 1;
        buffer[3 * i + 2] = 3 * i + 2;
      }
    }
  
    return data;
  } 
  ```
  * `bnd_mesh_handle` will be populated with the handle to the imported mesh upon success. This handle will be needed later to create bodies with this mesh.
  * `bnd_v3 center_of_mass` will be populated with the position of the mesh's center of mass upon success. This position is relative to the origin of the mesh.
  * The function will validate the mesh data inside `bnd_mesh_data` and return a `bnd_error`. Success is reported as `err.type == BND_OK`; otherwise the returned error contains the failure type and message.
  * This functions should be called _once per mesh_. After that, its `bnd_mesh_handle` can be used to create as many bodies as you need from it.

2) **Create a body with the imported mesh**

```c
// `mesh_handle` is received from `bnd_import_mesh`
bnd_body_handle mesh_body = bnd_add_mesh_dynamic(world, 5, mesh_handle).value;
// At this point it's just a regular body. You can change it's position or rotation, store its handle, apply forces, etc.
```

### Querying the physics world

Bandura currently supports the following queries:
- Raycasts (via `bnd_raycast_closest` and `bnd_raycast_multiple`)
- Overlaps (via `bnd_overlap`)

Here is an example of simulating an "explosion" using an overlap:

```c
  bnd_body_handle overlaps[5];
  uint32_t overlap_count = bnd_overlap(world, pos, explosion_radius, overlaps, 5); // Last parameter specifies the maximum overlap count.

  for (uint32_t i = 0; i < overlap_count; ++i) {
    bnd_v3 body_pos = bnd_get_position(world, overlaps[i]).value;
    bnd_apply_impulse(world, overlaps[i], bnd_v3_scale(bnd_v3_normalize(bnd_v3_sub(body_pos, pos)), explosion_impulse));
  }
```

**NOTE:** `bnd_raycast_multiple` and `bnd_overlap` will only return at most as many results as you specify in the parameter. Do not make any assumptions about their proximity
to the origin. They are not guaranteed to be the closest.

### Reacting to collisions

It's possible to subscribe to collision events for the particular body and react to them in your code:

```c
bnd_body_handle b = bnd_add_sphere_dynamic(world, 5, 0.5).value;
bnd_set_position(world, b, (bnd_v3){0, 5, 0});

// Makes the engine report collision events for this body during the simulation.
bnd_event_subscribe(world, b, BND_EVENT_COLLISION);

bnd_simulate(world, dt);

bnd_event_enumerator enumerator;
bnd_event_enumerate(world, b, &enumerator);

while (bnd_event_next(world, &enumerator)) {
  bnd_event e = enumerator.e;
  bnd_contact contact = e.collision;

  // Process the collision
}

// To stop receiving collision events:
bnd_event_unsubscribe(world, b, BND_EVENT_COLLISION);
```

## Configuring the engine

Configuration is done via [`bnd_config`](include/bandura.h) which is passed to `bnd_init`. It's recommended that you create a default config with `bnd_default_config` and then tweak the values if needed before calling `bnd_init`.

Here is the overview of the available configuration options:

### Memory

This section determines how much memory the engine will allocate upon initialization. If the capacity of some buffer is exceeded later on, Bandura will re-allocate it, doubling its capacity. Otherwise, no heap allocations will be done.

If you know  the upper limit for your body count in advance, you can adjust this section so that no allocations will be necessary after the engine is initialized.

* `dynamics_capacity` - number of dynamic bodies.
* `statics_capacity` - number of static bodies.
* `contacts_capacity` - maximum number of contacts per simulation frame.
* `joints_capacity` - number of joints.
* `meshes_capacity` - number of meshes imported with `bnd_import_mesh`.
* `events_capacity` - maximum number of events produced per simulation frame.

### Simulation

These parameters affect the physics simulation. You may want to adjust them for your game's needs

* `gravity` - constant gravitational _acceleration_ applied to all dynamic bodies every simulation frame. Default is `(0, -9.81, 0)`.
* `linear_drag` - how much linear velocity is lost every simulation frame. You may think of this as drag due to the air friction. Should be a value from 0 to 1. Default is `0.95`, meaning the bodies are being slowed down by 5% every frame.
* `angular_drag` - same as the previous one, but for the angular velocity (rotation speed). Default is `0.8`. 
* `bounciness` - Specifies how much the bodies bounce off each other upon collision. Should be the value from 0 to 1. Default is `0.2` which means that the bounce velocity will be 20% of the collision velocity.
* `friction` - dynamic friction between two bodies. Should be the value between 0 and 1, where the bigger value means stronger friction. Default is `0.9`.
* `sleep_base_bias` and `sleep_threshold` control how quickly the bodies go to sleep. Sleeping bodies are dynamic bodies which currently don't move. While in this state they consume almost no CPU resources for simulation, so this is benefitial for the game performance. To determine which body has to fall asleep, the engine uses the biased average of its velocity values across multiple frames. `sleep_base_bias` specifies which velocities are more important: values closer to 0 favour previous velocities, while values closer to 1 favour the current velocity. Once this biased average crosses `sleep_threshold`, the body falls asleep. 
* `min_bounce_velocity` defines the minimum velocity at which the body will bounce. Adjusting this may be useful for getting the more stable resting bodies. Default value is `0.25`.

### Advanced

These options affect the internal systems of the engine, such as collision detection algorithms, contact solver and supporting data structures. Adjust them if you know what you are doing.

## Handling errors

Bandura is transparent about the errors happening inside. The idea is that when the user calls a function, they should be able to know that something went wrong the moment it returns. To achieve that, there are several types which represent possible error states:

* `bnd_error` - the most basic one. Contains the type and a message. Generally the error handling pattern looks like this:

```c
bnd_error e = bnd_set_position(world, body, bnd_v3_zero());

// BND_OK is a value for "no error occured, all good"
if (e.type != BND_OK) {
  // When e.type != BND_OK, e.message will contain a user-friendly explanation of what went wrong
  printf("%s\n", e.message);
} else {
  // No errors, proceed normally
}
```
* `bnd_result_*` - this is a family of types which also wrap some value on top of the error. The idea is that is there is no error (`e.type == BND_OK`), you get the value. If there is one, the value is irrelevant. Example:

```c
bnd_result_v3 result = bnd_get_position(world, body);
if (result.error.type != BND_OK) {
  printf("%s\n", result.error.message);
} else {
  // Use bnd_result.value to get the actual data when there is no error.
  bnd_v3 position = result.value;
}
```

There are few classes of functions in Bandura that return these error-like objects. They include, but not limited to:

1) Functions for adding bodies:
  * Return `bnd_result_handle` - wrapper around `bnd_body_handle` + `bnd_error`.
  * In most cases these functions shouldn't fail, so you can safely use `value` without checking the error. The only way for them to fail is when the internal buffer exceeds its capacity and the engine tries to expand it, but fails. This may happen when the program runs out of memory or when using [custom memory allocators](#memory-allocator). So if you initialize Bandura with `bnd_init` and have some RAM, you shouldn't worry about this.
2) Functions for getting body data
  * Return different `bnd_result_*` types based on the function.
  * These functions can fail when provided a stale `bnd_body_handle`. A handle becomes stale when its body gets removed.
3) Functions for setting body data or applying forces.
  * Return `bnd_error`
  * Fail when provided stale `bnd_body_handle`. Some of them, for example `bnd_set_velocity`, will also fail when called on static bodies.

## Handling simulation time steps

For physics simulations, it's very important to correctly handle the time steps. In particular I'm referring to the `dt` parameter in the `bnd_simulate` function. For the simulation
to be stable and accurate, this should not only be small enough, but also consistent across frames. That's why it's generally not a good idea to use a regular delta time for that: while that 
might be small and consistent _most of the time_, sometimes the game may stutter. In this case the delta time goes up and the physics simulation explodes.

Because of that, physics engines are usually run on a _fixed time step_ - independent of the regular rendering and gameplay one. This way you not only get stable and smooth simulation, but also get to control the duration of this time step. In general, it's a tradeoff between the simulation quality and performance. If you pick a very small step, you get high quality of simulation, but you will have to run it more often, putting more load on the CPU. Increase the step - and the load will ease out, but the simulation might become less stable. As always, it's about finding the right balance for your particular scenario.

Here is an example of the game loop with physics simulation on a fixed time step:

```c
// 60 times per second is a good starting point.
static float physics_time_step = 1.0 / 60;

int main() {
  float delta_time = 0;
  float accumulator = 0;

  bnd_world *world = bnd_init(bnd_default_config());

  while(game_running()) {
    // Your normal game logic running on raw delta time.
    process_inputs(delta_time);
    update_gameplay(delta_time);

    accumulator += delta_time;
    int sim_count = (int) (accumulator / physics_time_step);

    // It's a good idea to bound the number of simulations per frame.
    // Otherwise it might go into a death spiral after large stutters.
    if (sim_count > 10) {
      sim_count = 10;
    }

    // Physics simulation might run multiple times per frame or once per a couple of frames.
    for (int i = 0; i < sim_count; ++i) {
      bnd_simulate(world, physics_time_step);
    }

    accumulator -= sim_count * physics_time_step;
    delta_time = get_delta_time();
  }

  bnd_teardown(world);

  return 0;
}
```

## Bring your own...

### Math

If your project already uses its own math types and functions, you can configure Bandure to use those instead of its native ones. The only requirement is the binary compatibility:

* For 3D vectors: 3 floats `x y z`
* For quaternions: 4 floats `x y z w`
* For 3x3 matrices: 9 floats arranged in rows `row0.x row0.y row0.z row1.x row1.y row1.z row2.x row2.y row2.z`

Here is an example for [Raylib's](https://www.raylib.com) `Vector3` and `Quaternion` types:

```c
#include "raylib.h"

// These define-typedefs must come before bandura.h is included
#define BND_CUSTOM_VEC3
typedef Vector3 bnd_v3;

#define BND_CUSTOM_QUAT
typedef Quaternion bnd_quat;

#include "bandura.h"

int main() {
  bnd_world *world = bnd_init(bnd_default_config());

  Vector3 plane_point = (Vector3) { 0, 5, 0 };
  Vector3 plane_normal = (Vector3) { 0, 1, 0 };

  // Use Raylib's Vector3 directly with Bandura
  bnd_add_plane(world, plane_point, plane_normal);

  // ...
}

```

### Memory allocator

If you want to control how Bandura allocates memory, you can provide your own memory management functions wrapped into [`bnd_allocator`](include/bandura.h). It accepts 3 function pointers: `malloc`, `realloc` and `free`.

* `void *(*bnd_malloc_fn)(uint64_t alignment, uint64_t size)` (mandatory)

  Used to initialize the physics world during `bnd_init_with_allocator`. Memory will be allocated according to the settings in `bnd_config.memory`. You can get the amount of memory needed for this using `bnd_required_memory(const bnd_config *config)`.

* `void *(*bnd_realloc_fn)(void *ptr, uint64_t alignment, uint64_t old_size, uint64_t new_size)` (optional)

  Bandura may request to expand the memory buffers when they become full. For example, when adding a new body, new joint or generating contacts during `bnd_simulate`. It will use this function if provided, otherwise the addition will fail and raise an error.

* `void (*bnd_free_fn)(void *ptr, uint64_t size)` (optional)

  This will be called during `bnd_teardown` to clean up the memory. Depending on your memory management strategy, you may or may not need to implement this.

Here is an example of an arena-type custom allocator. It contains all the memory in a single buffer and doesn't allow to expand it.

```c
#include "bandura.h"
#include <stdlib.h>
#include <stdint.h>

uint8_t *memory;
uint64_t offset, size;

static void *custom_malloc(uint64_t alignment, uint64_t bytes) {
  offset = (offset + alignment - 1) & ~(alignment - 1);
    
  if (offset + bytes > size) {
    // When malloc function returns NULL, Bandura will raise an error.
    return NULL;
  }

  void *ptr = memory + offset;
  offset += bytes;
  return ptr;
}

int main() {
  bnd_config config = bnd_default_config();
  bnd_allocator allocator = { custom_malloc, NULL, NULL };

  size = bnd_required_memory(&config);
  memory = malloc(size);
  offset = 0;
  
  bnd_result_world world = bnd_init_with_allocator(config, allocator);
  if (world.error.type != BND_OK) {
    printf("Failed to initialize Bandura: %s\n", world.error.message);
    return 1;
  }

  // bnd_world pointer is stored in world.value
  bnd_result_handle sphere = bnd_add_sphere_dynamic(world.value, 4, 2);
  if (sphere.error.type == BND_OK) {
    // With custom allocator, body addition may fail if you provide insufficient memory capacity in the config
    // and don't specify the realloc function.
    // 
    // Here you can safely use the sphere's handle: sphere.value
  }

  // ...

  bnd_teardown(world.value);

  return 0;
}
```

## License
The project is licensed under the Zlib License. See [`LICENSE`](LICENSE).

## LLM Usage

On this project, LLMs are used for:

* Writing tests and benchmarks
* Code reviews
* Occasional bugfix support
* Making demo scenes

Core library functionality is written manually without relying on AI neither for design nor implementation. I'm responsible for every line of code written there.
