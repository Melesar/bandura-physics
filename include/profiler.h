#ifndef PROFILER_H
#define PROFILER_H

#ifndef BND_PROFILING

#define PROFILER_BLOCK_START(name) (void)(name)
#define PROFILER_BLOCK_END

#define PROFILER_FUNCTION_START
#define PROFILER_FUNCTION_END

#define PROFILER_FRAME_START
#define PROFILER_FRAME_END

#else

#include <tracy/TracyC.h>

#define PROFILER_BLOCK_START(name) TracyCZoneN(profiler_ctx, name, true)
#define PROFILER_BLOCK_END TracyCZoneEnd(profiler_ctx)

#define PROFILER_FRAME_START TracyCFrameMarkStart("bnd_simulate")
#define PROFILER_FRAME_END TracyCFrameMarkEnd("bnd_simulate")

#define PROFILER_FUNCTION_START PROFILER_BLOCK_START(__FUNCTION__)
#define PROFILER_FUNCTION_END PROFILER_BLOCK_END

#endif

#endif
