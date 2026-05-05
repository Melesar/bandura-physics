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
  * [Handling errors](#handling-errors)
* [License](#license)
* [Third-party code](#third-party-code)

## Features 

* **Portability**. Written in C99, so it can be used in any project that supports the C ABI.
* **No dependencies**. The library itself is self-contained, so no third-party code is included.
* **Data-oriented**. The data layout inside the engine allows for efficient CPU cache utilization, enabling high performance.
* **Cross-platform**. Works on Linux and MacOS. Should work on Windows too, but is not actively tested there.
* **Easy to use**. Include a single header and link against the shared library and you're good to go.
* **Rigidbody physics simulation**. Allows for dynamic simulations with different shapes, including primitives, compounds and triangular meshes. Supports forces, impulses, rotations and joints.
* **Impulse-based collision resolution**. Objects respond to the collisions based on their mass, shape and collision velocity.

## Installation and build

### Requirements

Bandura uses [Zig](https://ziglang.org) as a build system. Currently supported version is `0.15.2`

#### Building the library itself

```bash
zig build --release=fast -Dinclude-demos=false
```
This will put the shared library file and the header file `bandura.h` into `zig-out` directory within the current directory. If you want the files to be installed into a different location, add `--prefix=<destination>` option to the command above.

#### Building the demos

Demos require dependencies for window management, rendering and UI. To download them, run the following command once:

```bash
git submodule update --init
```

Then build the library and all demos at once:

```bash
zig build
```
Executables will be installed into `zig-out/bin` by default.

## Using the library

### Quickstart

```c
#include "bandura.h"

int main() {
  // Create default config object.
  bnd_config config = bnd_default_config();

  // Initialize the physics world. This will be the root object for physics operations.
  bnd_world *world = bnd_init(&config);

  // Add a ground plane that passes through zero and has a (0, 1, 0) normal vector.
  bnd_add_plane(world, zero(), up());

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

Rigidbodies in Bandura can be of two types: **dynamic** and **static**. As the naming suggests, the former can move and respond to collisions, while the latter serve as static environment. Many functions for spawning bodies have two variants, for example:

```c
bnd_body bnd_add_box_dynamic(bnd_world *world, float mass, v3 size);
bnd_body bnd_add_box_static(bnd_world *world, v3 size);
```

Note that the static version doesn't require to specify mass, since static objects a treated as having _infinite_ mass.

The returned value `bnd_body` can be used to set initial state of the object: position and rotation, as well as velocity and angular momentum in case of dynamic bodies.

> NOTE: `bnd_body` is meant to be used _immediately_ after body creation and not be stored anywhere. The pointers it provides may be invalidated during the `bnd_simulate` call.

```c
#include "bandura.h"

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
  bnd_body_shape shapes[] = {}
    (bnd_body_shape) { 
      .type = BND_CYLINDER,
      .cylinder = { .radius = 0.3, .height = 3 },
      .offset = zero(),
      .rotation = (quat) { sinf(PI / 4), 0, 0, cosf(PI / 4) } 
    },
    (bnd_body_shape) {
      .type = BND_SPHERE,
      .sphere = { .radius = 0.7 },
      .offset = vec3(0, 0, 1.5),
      .rotation = qidentity()
    },
    (bnd_body_shape) {
      .type = BND_SPHERE,
      .sphere = { .radius = 0.7 },
      .offset = vec3(0, 0, -1.5),
      .rotation = qidentity() 
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
v3 sphere_position = bnd_get_position(world, sphere_handle);
v3 sphere_velocity = bnd_get_velocity(world, sphere_handle);
```

3) **Removing the body**

```c
bnd_remove_body(world, sphere_handle);

// From here on, `sphere_handle` is no longer valid and shouldn't be used.
```

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
  *left_upper_leg.rotation = (quat) { sinf(PI / 12), 0, 0, cosf(PI / 12) };

  bnd_body left_lower_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *left_lower_leg.position = vec3(0.23, 0.6, -0.2);
  *left_lower_leg.rotation = (quat) { sinf(PI / 12), 0, 0, cosf(PI / 12) };

  bnd_body right_upper_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *right_upper_leg.position = vec3(-0.23, 1.8, 0);

  bnd_body right_lower_leg = bnd_add_cylinder_dynamic(world, 10, 0.2, 1.2);
  *right_lower_leg.position = vec3(-0.23, 0.6, 0);

  bnd_body left_upper_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *left_upper_arm.position = vec3(0.4, 3.9, -0.4);
  *left_upper_arm.rotation = (quat) { sinf(PI / 10), 0, 0, cosf(PI / 10) };

  bnd_body left_lower_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *left_lower_arm.position = vec3(0.43, 3.37, -1.45);
  *left_lower_arm.rotation = (quat) { sinf(PI / 4), 0, 0, cosf(PI / 12) };

  bnd_body right_upper_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *right_upper_arm.position = vec3(-0.43, 3.8, 0);

  bnd_body right_lower_arm = bnd_add_cylinder_dynamic(world, 10, 0.1, 1.2);
  *right_lower_arm.position = vec3(-0.43, 2.63, -0.3);
  *right_lower_arm.rotation = (quat) { sinf(PI / 12), 0, 0, cosf(PI / 12) };

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
bool bnd_import_mesh(bnd_world *world, const bnd_mesh_data *data, bnd_mesh_handle *handle, v3 *center_of_mass);
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
  * `v3 center_of_mass` will be populated with the position of the mesh's center of mass upon success. This position is relative to the origin of the mesh.
  * The function will validate the mesh data inside `bnd_mesh_data` and return `false` in case it's invalid. It will also raise an error with an explanation of what went wrong (see [Handling errors](#handling-errors))
  * This functions should be called _once per mesh_. After that, its `bnd_mesh_handle` can be used to create as many bodies as you need from it.

2) **Create a body with the imported mesh**

```c
// `mesh_handle` is received from `bnd_import_mesh`
bnd_body mesh_body = bnd_add_mesh_dynamic(world, 5, mesh_handle);
// At this point it's just a regular body. You can change it's position or rotation, store its handle, apply forces, etc.
```

### Handling errors

Bandura can notify the users when something went wrong. To receive this notifications, you should register a callback function at any point of the program's lifetime:

```c
#include "bandura.h"
#include <stdio.h>

static void on_error(bnd_error error, char *message, void *data) {
  switch (error) {
    case BND_ERROR_MESH_INVALID:
      bnd_mesh_data *mesh_data = (bnd_mesh_data *) data;
      printf("Incorrectly populated mesh data: %p\n", data);
      printf("Reason: %s\n", message);
      break;
  }
}

int main () {
  bnd_register_error_callback(on_error);

  bnd_config config = bnd_default_config();

  //...
}
```

## License
The project is licensed under the Zlib License. See [`LICENSE`](LICENSE).

## Third-party code
See [THIRD_PARTY_NOTICES](./THIRD_PARTY_NOTICES) for details.
