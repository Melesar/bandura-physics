const std = @import("std");
const common = @import("../scripts/common.zig");

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
    module.addCSourceFiles(.{
      .files = try common.collectSources(b, "tests"),
      .flags = try flags.collect(),
      .language = .c
    });

    return module;
}

pub fn createArtifact(b: *std.Build, options: Options, module: *std.Build.Module) *std.Build.Step.Run {
    const exeTests = b.addExecutable(.{
      .name = "tests",
      .root_module = module,
    });

    var artifact = b.addRunArtifact(exeTests);

    if (options.installBinaries) {
        b.installArtifact(exeTests);
        artifact.step.dependOn(b.getInstallStep());
    }

    return artifact;
}
