
extern void shapes_tests();
extern void joints_tests();
extern void collisions_tests();
extern void allocator_tests();
extern void events_tests();
extern void joint_api_tests();
extern void queries_tests();
extern void world_tests();
extern void collision_layers_tests();
extern void broad_phase_tests();
extern void manifold_tests();

int main() {
  shapes_tests();
  joints_tests();
  broad_phase_tests();
  manifold_tests();
  collisions_tests();
  allocator_tests();
  events_tests();
  joint_api_tests();
  queries_tests();
  world_tests();
  collision_layers_tests();
}
