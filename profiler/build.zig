const std = @import("std");
const common = @import("../scripts/common.zig");

pub const Options = struct {
    target: common.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
};

pub fn createModule(b: *std.Build, options: Options) !*std.Build.Module {
  const module = b.createModule(.{
    .target = options.target,
    .optimize = options.optimize,
    .link_libc = true,
    .sanitize_c = if (options.optimize == .Debug or options.optimize == .ReleaseSafe) .full else .off,
  });

  var flags = try common.CompileFlags.default(b.allocator);
  try flags.addOptimizations(options.optimize);

  module.addCSourceFiles(.{
    .files = try common.collectSources(b, "profiler"),
    .flags = try flags.collect(),
    .language = .c
  });

  common.enableProfiling(module);
  module.addIncludePath(b.path("include"));

  return module;
}

pub fn addLibrary(b: *std.Build, module: *std.Build.Module) *std.Build.Step.Compile {
  const lib = b.addLibrary(.{
    .name = "bnd_profiler",
    .linkage = .static,
    .root_module = module,
  });

  return lib;
}
