# Bandura physics engine

Bandura is a traditional Ukrainian [musical instrument](https://en.wikipedia.org/wiki/Bandura) and also a simple and fast 3D physics engine.

## Table of contents

* [Features](#features)
* [Installation and build](#installation-and-build)
  * [Requirements](#requirements)
  * [Building the library itself](#building-the-library-itself)
  * [Building the demos](#building-the-demos)
* [Using the library](#using-the-library)
  * [Quickstart](#quickstart)
  * [Spawning bodies](#spawning-bodies)
  * [Referencing bodies](#referencing-bodies)
  * [Bounding the bodies together](#bounding-the-bodies-together)
  * [Using meshes as body shapes](#using-meshes-as-body-shapes)
  * [Querying the physics world](#querying-the-physics-world)
  * [Reacting to collisions](#reacting-to-collisions)
* [Bring your own...](#bring-your-own)
  * [Math](#math)
  * [Memory allocator](#memory-allocator)
* [License](#license)
* [Third-party code](#third-party-code)

## Features 

* **Portability**. Written in C99, so it can be used in any project that supports the C ABI.
* **No dependencies**. The library itself is self-contained, so no third-party code is included.
* **Data-oriented**. The data layout inside the engine allows for efficient CPU cache utilization, enabling high performance.
* **Cross-platform**. Works on Linux and MacOS. Should work on Windows too, but is not actively tested there.
* **Easy to use**. Include `bandura.h` for the core API and link against the static library. You may also include `bnd-math.h` for convenient math helpers if you need them.
* **Rigidbody physics simulation**. Allows for dynamic simulations with different shapes, including primitives, compounds and triangular meshes. Supports forces, impulses, rotations and joints.
* **Impulse-based collision resolution**. Objects respond to the collisions based on their mass, shape and collision velocity.

## Installation and build

### Requirements

To build Bandura directly, you only need [CMake](https://cmake.org) and a C compiler.

If you only want to use Bandura inside your own CMake game project, you usually do not need to install it separately. You can vendor the repository and add it with `add_subdirectory(...)`.

#### Building the library itself

```bash
cmake -S . -B build
cmake --build build
```

By default, this builds Bandura in `Release` mode and produces a static library.

If you want to install the library and public headers somewhere on your system:

```bash
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/your/install/path
cmake --build build
cmake --install build
```

That installs the library together with `bandura.h` and `bnd-math.h`.

#### Building the demos

Demos require dependencies for window management, rendering and UI. To download them, run the following command once:

```bash
git submodule update --init
```

Then build the library and all demos at once:

```bash
cmake -S . -B build -DBANDURA_BUILD_DEMOS=ON
cmake --build build
```

## Using the library

The examples below use `bnd-math.h` for helpers such as `vec3`, `bnd_v3_zero`, `bnd_v3_up`, and `bnd_qidentity`.

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
  bnd_body sphere = bnd_add_sphere_dynamic(world, 5, 1);

  // Lift it up above the ground.
  *sphere.position = vec3(0, 5, 0);

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
bnd_body bnd_add_box_dynamic(bnd_world *world, float mass, bnd_v3 size);
bnd_body bnd_add_box_static(bnd_world *world, bnd_v3 size);
bnd_body bnd_add_sphere_dynamic(bnd_world *world, float mass, float radius);
bnd_body bnd_add_sphere_static(bnd_world *world, float radius);
```

Note that the static version doesn't require to specify mass, since static objects are treated as having _infinite_ mass.

The returned value `bnd_body` can be used to set initial state of the object: position and rotation, as well as velocity and angular momentum in case of dynamic bodies.

> NOTE: `bnd_body` is meant to be used _immediately_ after body creation and not be stored anywhere. The pointers it provides may be invalidated during the `bnd_simulate` call.

```c
#include "bandura.h"
#include "bnd-math.h"

int main() {
  // ...

  // Add a static box and set it's position. The rotation will be initialized to identity.
  bnd_body static_box = bnd_add_box_static(world, vec3(10, 2, 3));
  *static_box.position = vec3(0, 1, 0);

  bnd_body dynamic_sphere = bnd_add_sphere_dynamic(world, 5, 1);
  *dynamic_sphere.position = vec3(5, 10, 5);

  // Add an upward velocity to the dynamic sphere. For the static box this pointer will be NULL.
  *dynamic_sphere.velocity = vec3(0, 5, 0);
}
```

Bandura also supports combining multiple primitive shapes into a single body. This allows for creation of complex shapes without using more expensive meshes.

```c
  // Make a dumbell shape with a rotated cylinder and two spheres.
  bnd_body_shape shapes[] = {
    (bnd_body_shape) { 
      .type = BND_CYLINDER,
      .cylinder = { .radius = 0.3, .height = 3 },
      .offset = bnd_v3_zero(),
      .rotation = (bnd_quat) { sinf(PI / 4), 0, 0, cosf(PI / 4) } 
    },
    (bnd_body_shape) {
      .type = BND_SPHERE,
      .sphere = { .radius = 0.7 },
      .offset = vec3(0, 0, 1.5),
      .rotation = bnd_qidentity()
    },
    (bnd_body_shape) {
      .type = BND_SPHERE,
      .sphere = { .radius = 0.7 },
      .offset = vec3(0, 0, -1.5),
      .rotation = bnd_qidentity() 
    }
  };

  // Masses correspond to the shapes: 3 for the cylinder, 5's for the spheres.
  float masses[] = { 3, 5, 5 }

  // As a last parameter the function takes the number of shapes in the arrays.
  bnd_body dumbell = bnd_add_compound_body_dynamic(world, shapes, masses, 3);
  *dumbell.position = vec3(15, 10, 0);
```

### Referencing bodies

Apart from the initial body state, `bnd_body` also provides `bnd_body_handle handle` field. This value should be used to access the body across its entire lifetime and can be stored after the body creation. 

`bnd_body_handle` can be used to:

1) **Apply forces, torques and impulses to the body**

```c
#include "bandura.h"

bnd_body_handle sphere_handle;

int main() {
  // ...
 
  bnd_body sphere = bnd_add_sphere_dynamic(world, 5, 1);

  // Store the handle in the global variable.
  sphere_handle = sphere.handle;

  // ... 

  // Apply an upward impulse to the sphere using the previously stored handle.
  bnd_apply_impulse(world, sphere_handle, vec3(0, 10, 0));
}
```
2) **Querying body's properties**

```c
bnd_v3 sphere_position = bnd_get_position(world, sphere_handle);
bnd_v3 sphere_velocity = bnd_get_velocity(world, sphere_handle);
```

3) **Removing the body**

```c
bnd_remove_body(world, sphere_handle);

// From here on, `sphere_handle` is no longer valid and shouldn't be used.
```

After the body is removed, its corresponding handle is invalidated. You can always check if a handle is valid using `bool bnd_handle_valid(const bnd_world *world, bnd_body_handle handle)` function.

### Bounding the bodies together

Bandura supports imposing constraints on the bodies, so that they cannot move further apart than some specified distance. This can be used, for example, to create ragdolls:

```c
  bnd_body head = bnd_add_sphere_dynamic(world, 3, 0.4);
  *head.position = vec3(0, 5, 0);

  bnd_body torso = bnd_add_cylinder_dynamic(world, 25, 0.3, 1.0);
  *torso.position = vec3(0, 4, 0);

  bnd_body pelvis = bnd_add_cylinder_dynamic(world, 20, 0.25, 1.0);
  *pelvis.position = vec3(0, 3, 0);

  bnd_body left_upper_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *left_upper_leg.position = vec3(0.23, 1.8, -0.2);
  *left_upper_leg.rotation = (bnd_quat) { sinf(PI / 12), 0, 0, cosf(PI / 12) };

  bnd_body left_lower_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *left_lower_leg.position = vec3(0.23, 0.6, -0.2);
  *left_lower_leg.rotation = (bnd_quat) { sinf(PI / 12), 0, 0, cosf(PI / 12) };

  bnd_body right_upper_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *right_upper_leg.position = vec3(-0.23, 1.8, 0);

  bnd_body right_lower_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *right_lower_leg.position = vec3(-0.23, 0.6, 0);

  bnd_body left_upper_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *left_upper_arm.position = vec3(0.4, 3.9, -0.4);
  *left_upper_arm.rotation = (bnd_quat) { sinf(PI / 10), 0, 0, cosf(PI / 10) };

  bnd_body left_lower_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *left_lower_arm.position = vec3(0.43, 3.37, -1.45);
  *left_lower_arm.rotation = (bnd_quat) { sinf(PI / 4), 0, 0, cosf(PI / 12) };

  bnd_body right_upper_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *right_upper_arm.position = vec3(-0.43, 3.8, 0);

  bnd_body right_lower_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *right_lower_arm.position = vec3(-0.43, 2.63, -0.3);
  *right_lower_arm.rotation = (bnd_quat) { sinf(PI / 12), 0, 0, cosf(PI / 12) };

  const float joint_margin = 0.1;

  bnd_add_joint(world, head.handle, torso.handle, vec3(0, -0.4, 0), vec3(0, 0.5, 0), joint_margin);
  bnd_add_joint(world, torso.handle, pelvis.handle, vec3(0, -0.5, 0), vec3(0, 0.5, 0), joint_margin);

  bnd_add_joint(world, torso.handle, left_upper_arm.handle, vec3(0.3, 0.45, 0), vec3(-0.1, 0.6, 0), joint_margin);
  bnd_add_joint(world, left_upper_arm.handle, left_lower_arm.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  bnd_add_joint(world, torso.handle, right_upper_arm.handle, vec3(-0.3, 0.45, 0), vec3(0.1, 0.6, 0), joint_margin);
  bnd_add_joint(world, right_upper_arm.handle, right_lower_arm.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  bnd_add_joint(world, pelvis.handle, left_upper_leg.handle, vec3(0.23, -0.5, 0), vec3(0, 0.6, 0), joint_margin);
  bnd_add_joint(world, left_upper_leg.handle, left_lower_leg.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  bnd_add_joint(world, pelvis.handle, right_upper_leg.handle, vec3(-0.23, -0.5, 0), vec3(0, 0.6, 0), joint_margin);
  bnd_add_joint(world, right_upper_leg.handle, right_lower_leg.handle, vec3(0, -0.6, 0), vec3(0, 0.6, 0), joint_margin);

  bnd_body_handle ragdoll[11];
  ragdoll[0] = head.handle;
  ragdoll[1] = torso.handle;
  ragdoll[2] = pelvis.handle;
  ragdoll[3] = left_upper_arm.handle;
  ragdoll[4] = left_lower_arm.handle;
  ragdoll[5] = right_upper_arm.handle;
  ragdoll[6] = right_lower_arm.handle;
  ragdoll[7] = left_upper_leg.handle;
  ragdoll[8] = left_lower_leg.handle;
  ragdoll[9] = right_upper_leg.handle;
  ragdoll[10] = right_lower_leg.handle;
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
bnd_body mesh_body = bnd_add_mesh_dynamic(world, 5, mesh_handle);
// At this point it's just a regular body. You can change it's position or rotation, store its handle, apply forces, etc.
```

### Querying the physics world

Bandura currently supports the following queries:
- Raycasts (via `bnd_raycast_closest` and `bnd_raycast_multiple`)
- Overlaps (via `bnd_overlap`)

Here is an example of simulating an "explosion" using an overlap:

```c
  bnd_body_handle overlaps[5];
  count_t overlap_count = bnd_overlap(world, pos, explosion_radius, overlaps, 5); // Last parameter specifies the maximum overlap count.

  for (count_t i = 0; i < overlap_count; ++i) {
    bnd_v3 body_pos = bnd_get_position(world, overlaps[i]);
    bnd_apply_impulse(world, overlaps[i], bnd_v3_scale(bnd_v3_normalize(bnd_v3_sub(body_pos, pos)), explosion_impulse));
  }
```

**NOTE:** `bnd_raycast_multiple` and `bnd_overlap` will only return at most as many results as you specify in the parameter. Do not make any assumptions about their proximity
to the origin. They are not guaranteed to be the closest.

### Reacting to collisions

It's possible to subscribe to collision events for the particular body and react to them in your code:

```c
bnd_body b = bnd_add_sphere_dynamic(world, 5, 0.5);
*b.position = vec3(0, 5, 0);

// Makes the engine report collision events for this body during the simulation.
bnd_event_subscribe(world, b.handle, BND_EVENT_COLLISION);

bnd_simulate(world, dt);

bnd_event_enumerator enumerator;
bnd_event_enumerate(world, b.handle, &enumerator);

while (bnd_event_next(world, &enumerator)) {
  bnd_event e = enumerator.e;
  bnd_contact contact = e.collision.contact;

  // Process the collision
}

// To stop receiving collision events:
bnd_event_unsubscribe(world, b.handle, BND_EVENT_COLLISION);
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
typedef Vector3 bnd_v3

#define BND_CUSTOM_QUAT
typedef Quaternion bnd_quat

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

  Used to initialize the physics world during `bnd_init`. Memory will be allocated according to the settings in `bnd_config.memory`. You can get the amount of memory needed for this using `bnd_required_memory(const bnd_config *config)`.

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

static void *custom_malloc(uint64_t alignment, uint64_t size) {
  offset = (offset + alignment - 1) & ~(alignment - 1);
    
  if (offset + bytes > size) {
    // When malloc function returns NULL, Bandura will raise an error.
    return NULL;
  }

  void *ptr = memory + offset;
  offset += bytes;
  return ptr;
}

static void on_error(bnd_error_type type, char *message, void *data) {
  switch (type) {
    case BND_ERROR_NO_SPACE_AVAILABLE:
      // Raised when the buffers are full and realloc fuction is not provided.
      break;

    case BND_ERROR_OUT_OF_MEMORY:
      // Raised when malloc or realloc return NULL.
      break;

    case BND_ERROR_INVALID_ALLOCATOR:
      // Raised when malloc function is NULL.
      break;
  }
}

int main() {
  bnd_config config = bnd_default_config();
  bnd_allocator allocator = { custom_malloc, NULL, NULL };
  bnd_error error;
  
  bnd_register_error_callback(on_error);

  size = bnd_required_memory(&config);
  memory = malloc(size);
  offset = 0;
  
  bnd_world *world = bnd_init_with_allocator(config, allocator, &error);
  if (error.type != BND_OK) {
    printf("Failed to initialize Bandura: %s\n", error.message);
    return 1;
  }

  bnd_body sphere = bnd_add_sphere_dynamic(world, 4, 2);
  if (!bnd_handle_valid(world, sphere.handle)) {
    // With custom allocator, body addition may fail if you provide insufficient memory capacity in the config
    // and don't specify the realloc function.
  }

  // ...

  return 0;
}
```

## License
The project is licensed under the Zlib License. See [`LICENSE`](LICENSE).

## Third-party code
See [THIRD_PARTY_NOTICES](./THIRD_PARTY_NOTICES) for details.
