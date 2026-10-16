
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>

#include <stdio.h>

extern CU_ErrorCode register_shapes_tests(void);
extern CU_ErrorCode register_joints_tests(void);
extern CU_ErrorCode register_broad_phase_tests(void);
extern CU_ErrorCode register_manifold_tests(void);
extern CU_ErrorCode register_collisions_tests(void);
extern CU_ErrorCode register_allocator_tests(void);
extern CU_ErrorCode register_events_tests(void);
extern CU_ErrorCode register_joint_api_tests(void);
extern CU_ErrorCode register_queries_tests(void);
extern CU_ErrorCode register_world_tests(void);
extern CU_ErrorCode register_collision_layers_tests(void);

int main(void) {
  CU_ErrorCode (*register_suites[])(void) = {
    register_shapes_tests, register_joints_tests, register_broad_phase_tests,
    register_manifold_tests, register_collisions_tests, register_allocator_tests,
    register_events_tests, register_joint_api_tests, register_queries_tests,
    register_world_tests, register_collision_layers_tests,
  };

  CU_ErrorCode error = CU_initialize_registry();
  if (error != CUE_SUCCESS) {
    fprintf(stderr, "CUnit initialization failed: %s\n", CU_get_error_msg());
    return 1;
  }

  for (unsigned int i = 0; i < sizeof(register_suites) / sizeof(register_suites[0]); ++i) {
    error = register_suites[i]();
    if (error != CUE_SUCCESS) {
      fprintf(stderr, "CUnit registration failed (code %d): %s\n", error, CU_get_error_msg());
      CU_cleanup_registry();
      return 1;
    }
  }

  CU_basic_set_mode(CU_BRM_VERBOSE);
  error = CU_basic_run_tests();
  unsigned int failures = CU_get_number_of_failures();
  unsigned int failed_suites = CU_get_number_of_suites_failed();
  CU_cleanup_registry();
  return error == CUE_SUCCESS && failures == 0 && failed_suites == 0 ? 0 : 1;
}
