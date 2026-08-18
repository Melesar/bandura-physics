#ifndef PROFILER_H
#define PROFILER_H

#include <stdint.h>

typedef struct {
  uint16_t body_count;
  uint16_t contacts_count;
} profiler_frame_metadata;

#ifndef BND_PROFILING

#define PROFILE_BLOCK(name)
#define PROFILE_FUNCTION

#define PROFILER_START_FRAME
#define PROFILER_END_FRAME(metadata)
#define PROFILER_INIT
#define PROFILER_TEARDOWN

#else

// TODO

#endif

#endif
