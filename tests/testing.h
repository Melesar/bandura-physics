#ifndef TESTING_H
#define TESTING_H

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

typedef void (*testing_func)();

struct test {
  char *label;
  testing_func function;
};

#define TESTS_BEGIN(name)                                                                                              \
  printf(name);                                                                                                        \
  printf(":\n\n");                                                                                                     \
  struct test tests[] = {

#define TEST(func) {#func, &func},

#define TESTS_END                                                                                                      \
  { NULL, NULL }                                                                                                       \
  }                                                                                                                    \
  ;                                                                                                                    \
  int test_index = 0;                                                                                                  \
  while (true) {                                                                                                       \
    struct test t = tests[test_index++];                                                                               \
    if (t.label == NULL && t.function == NULL)                                                                         \
      break;                                                                                                           \
    printf("  %s:  ", t.label);                                                                                        \
    t.function();                                                                                                      \
    printf("Success!\n");                                                                                              \
  };                                                                                                                   \
  printf("\n");

#endif
