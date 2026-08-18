const std = @import("std");
const common = @import("../scripts/build-common.zig");

pub fn createStep(b: *std.Build, target: common.ResolvedTarget, banduraSourcePath: std.Build.LazyPath) !*std.Build.Step.Compile {
    const benchmarksDependency = b.dependency("benchmarks", .{});

    var module = b.createModule(.{
        .optimize = .ReleaseFast,
        .target = target,
        .link_libcpp = true,
    });

    module.addCSourceFile(.{
        .file = banduraSourcePath,
        .flags = &.{"-O2"},
        .language = .c,
    });

    module.addCSourceFile(.{
        .file = b.path("benchmarks/benchmarks.cpp"),
        .language = .cpp,
    });

    module.addIncludePath(benchmarksDependency.path("include"));
    module.addIncludePath(b.path("include"));

    module.addLibraryPath(benchmarksDependency.path("build/src"));

    module.linkSystemLibrary("benchmark", .{});
    module.linkSystemLibrary("pthread", .{});

    const exe = b.addExecutable(.{
        .name = "benchmarks",
        .root_module = module,
    });

    const run = b.addRunArtifact(exe);
    if (b.args) |args| {
        run.addArgs(args);
    }
    const step = b.step("bench", "Run benchmarks");

    step.dependOn(&run.step);

    return exe;
}
