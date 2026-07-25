const std = @import("std");
const commons = @import("../scripts/build-common.zig");

pub const Options = struct {
  target: commons.ResolvedTarget,
  optimize: std.builtin.OptimizeMode,
};

pub const Scenario = struct {
  name: []const u8,
  default: *std.Build.Module,
  profiling: *std.Build.Module,

  const Target = struct {
    name: []const u8,
    default: *std.Build.Step.Compile,
    profiling: *std.Build.Step.Compile,

    pub fn createSteps(self: Target, b: *std.Build) void {
        const install = b.addInstallArtifact(self.default, .{});
        const installStep = b.step(b.fmt("{s}", .{self.name}), b.fmt("Build {s} scenario", .{self.name}));
        installStep.dependOn(&install.step);

        const run = b.addRunArtifact(self.default);
        const runStep = b.step(b.fmt("run-{s}", .{self.name}), b.fmt("Run {s} scenario", .{self.name}));
        runStep.dependOn(&run.step);

        const profile = b.addRunArtifact(self.profiling);
        const profileStep = b.step(b.fmt("profile-{s}", .{self.name}), b.fmt("Profile {s} scenario", .{self.name}));
        profileStep.dependOn(&profile.step);
    }
  };

  pub fn compile(self: Scenario, b: *std.Build) Target {
    return Target {
      .name = self.name,
      .default = b.addExecutable(.{ .name = self.name, .root_module = self.default }),
      .profiling = b.addExecutable(.{ .name = self.name, .root_module = self.profiling }),
    };
  }
};


pub fn enumerate(b: *std.Build, options: Options) ![]const Scenario {
    const profilingOptions = Options {
      .target = options.target,
      .optimize = .ReleaseFast,
    };

    const scenarioSources = try commons.collectSources(b, "demos/scenarios");
    defer b.allocator.free(scenarioSources);

    const binarySources = try commons.collectSources(b, "demos");
    defer b.allocator.free(binarySources);

    const scenarios = try b.allocator.alloc(Scenario, scenarioSources.len);
    for(scenarioSources, 0..) |srcFile, i| {
      const scenario = Scenario {
        .name = std.fs.path.stem(srcFile),
        .default = try makeModule(b, options, srcFile, binarySources),
        .profiling = try makeModule(b, profilingOptions, srcFile, binarySources),
      };

      commons.enableProfiling(scenario.profiling);
      scenarios[i] = scenario;
    }

    return scenarios;
}

fn makeModule(b: *std.Build, options: Options, scenarioFile: []const u8, otherFiles: []const []const u8) !*std.Build.Module {
    const scenarioModule = b.createModule(.{
      .target = options.target,
      .optimize = options.optimize,
      .link_libc = true
    });

    var binFlags = try commons.CompileFlags.default(b.allocator);
    try binFlags.addOptimizations(options.optimize);

    const flags = try binFlags.collect();
    scenarioModule.addCSourceFiles(.{
        .files = otherFiles,
        .flags = flags,
    });
    scenarioModule.addCSourceFile(.{
      .file = b.path(scenarioFile),
      .flags = flags
    });

    const raylib = b.dependency("raylib", .{
      .target = options.target,
      .optimize = options.optimize,
      .config = "-DPLATFORM_DESKTOP -DSUPPORT_PRAND_GENERATOR",
      .linkage = .dynamic,
    });
    const clay = b.dependency("clay", .{});
    const raygui = b.dependency("raygui", .{});

    scenarioModule.addIncludePath(raylib.path("src"));
    scenarioModule.addIncludePath(raylib.path("examples"));
    scenarioModule.addIncludePath(clay.path("."));
    scenarioModule.addIncludePath(raygui.path("src"));
    scenarioModule.addIncludePath(raygui.path("styles"));
    scenarioModule.addIncludePath(b.path("demos/include"));
    scenarioModule.addIncludePath(b.path("include"));
    scenarioModule.addIncludePath(b.path("src"));

    scenarioModule.linkLibrary(raylib.artifact("raylib"));

    switch (options.target.result.os.tag) {
        .linux => {
            scenarioModule.linkSystemLibrary("m", .{});
            scenarioModule.linkSystemLibrary("pthread", .{});
            scenarioModule.linkSystemLibrary("GLX", .{});
            scenarioModule.linkSystemLibrary("X11", .{});
            scenarioModule.linkSystemLibrary("Xcursor", .{});
            scenarioModule.linkSystemLibrary("Xext", .{});
            scenarioModule.linkSystemLibrary("Xfixes", .{});
            scenarioModule.linkSystemLibrary("Xi", .{});
            scenarioModule.linkSystemLibrary("Xinerama", .{});
            scenarioModule.linkSystemLibrary("Xrandr", .{});
            scenarioModule.linkSystemLibrary("Xrender", .{});
        },
        .macos => {
            scenarioModule.linkFramework("IOKit", .{});
            scenarioModule.linkFramework("Cocoa", .{});
            scenarioModule.linkFramework("OpenGL", .{});
        },
        else => {},
    }
  return scenarioModule;
}
