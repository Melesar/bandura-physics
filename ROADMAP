# Road to v1.0

The following is a rough roadmap of the intermediate versions before *v1.0*. It outlines only the big features, omitting smaller improvements, bugfixes and performance work.

Patch versions are not listed as well, but they will surely appear at some point.

These are the main pillars of the future *v1.0*:

1) Performance improvements.
2) Richer joint support.
3) Convex meshes support.
4) Continuos collision detection.
5) Determinism.

## v0.2
  * **Better solver**. The original solver in *v0.1* is quite slow on the large number of contacts and doesn't provide stable resting contacts.
  * **Hinge joint**. Allows to constraint the bodies to only rotate around one axis.

## v0.3
  * **Contact islands**. Continuing with the performance improvements for the solver, introduction of islands should make it faster and also prepare grounds for the further work on multithreading.

## v0.4
  * **Determinism**. I plan on making the simulation deterministic across multiple runs, different compilers and *perhaps* across CPU architectures.
  * **Fixed joint**. Allows to bind two bodies together so that they are always at the same distance between each other.

## v0.5
  * **Multithreading**. Here I will work on paralellizing internal calculations of the engine which will allow it to scale well on more powerful machines. Also I might add thread safety to allow running the simulation on a background thread. 

## v0.6
  * **Arbitrary mesh support**. Version *v0.1* only supports convex meshes as collision shapes, which is very limiting. Here I plan to lift that limitation and implement support for arbitrary meshes.
  * **Spring joint**. Allows to simulate a body on a spring.

## v0.7
  * **Broad-phase collision detection**. Here I plan to implement space-partitioning algorithms that should accelerate collision detection phase.

## v0.8
  * **Continous collision detection**. Until now, fast moving objects could penetrate through obstacles. This feature should solve this issue.
  * **Character joint**. Improve ragdoll support.

## v0.9
  * **SIMD**. This is levereging modern CPU features to gain more performance for integration and solver.


