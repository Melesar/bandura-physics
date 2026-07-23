const std = @import("std");
const common = @import("../scripts/build-common.zig");

pub const Options = struct {
    target: common.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    installBinaries: bool
};

pub fn createModule(b: *std.Build, options: Options) !*std.Build.Module {
    const module = b.createModule(.{
      .target = options.target,
      .optimize = options.optimize,
      .link_libc = true,
    });
    module.addIncludePath(b.path("tests"));
    module.addIncludePath(b.path("include"));

    var flags = try common.CompileFlags.default(b.allocator);
    try flags.addOptimizations(options.optimize);

    module.addCSourceFiles(.{
      .files = try common.collectSources(b, "tests"),
      .flags = try flags.collect(),
      .language = .c
    });

    return module;
}

pub fn createExe(b: *std.Build, module: *std.Build.Module) *std.Build.Step.Compile {
    return b.addExecutable(.{
      .name = "tests",
      .root_module = module,
    });
}

pub fn createArtifact(b: *std.Build, options: Options, exe: *std.Build.Step.Compile) *std.Build.Step.Run {
    var artifact = b.addRunArtifact(exe);

    if (options.installBinaries) {
        b.installArtifact(exe);
        artifact.step.dependOn(b.getInstallStep());
    }

    return artifact;
}
