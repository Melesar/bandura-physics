const std = @import("std");
const common = @import("../scripts/common.zig");

pub const Options = struct {
    target: common.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    linkage: std.builtin.LinkMode,
};

pub fn createBaseModule(b: *std.Build, options: Options) !*std.Build.Module {
    const module = b.createModule(.{
      .target = options.target,
      .optimize = options.optimize,
      .link_libc = true,
      .sanitize_c = .full,
    });

    module.addIncludePath(b.path("include"));
    module.addIncludePath(b.path("src"));

    var flags = try common.CompileFlags.default(b.allocator);
    try flags.addOptimizations(options.optimize);

    if (options.linkage == .dynamic) {
        module.addCMacro("BND_BUILD_DLL", "");

        try flags.add("-fvisibility=hidden");
    } else {
        module.addCMacro("BND_STATIC", "");
    }

    if (options.optimize == .Debug) {
      module.addCMacro("BND_DEBUG", "");
    }

    module.addCMacro("COLLISION_TEST_SUITE_PATH", "\"tests/collision_test_cases.yaml\"");

    module.addCSourceFiles(.{
      .files = try common.collectSources(b, "src"),
      .flags = try flags.collect(),
      .language = .c
    });

    return module;
}

pub fn addLibrary(b: *std.Build, module: *std.Build.Module, options: Options) *std.Build.Step.Compile {
    const lib = b.addLibrary(.{
      .name = "bandura",
      .linkage = options.linkage,
      .root_module = module
    });

    lib.installHeader(b.path("include/bandura.h"), "bandura.h");
    lib.installHeader(b.path("include/bnd-math.h"), "bnd-math.h");
    lib.installHeader(b.path("src/bnd-core.h"), "bnd-core.h");

    return lib;
}

pub fn buildLibrary(b: *std.Build, options: Options) !*std.Build.Step.Compile {
  const module = try createBaseModule(b, options);

  return addLibrary(b, module, options);
}
