const std = @import("std");

const common = @import("scripts/common.zig");
const bandura = @import("src/build.zig");
const tests = @import("tests/build.zig");
const profiler = @import("profiler/build.zig");

const cc = @import("compile_commands");

const Options = struct {
    target: common.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    linkage: std.builtin.LinkMode,
    profiling: bool,
    installTests: bool,
    includeDemos: bool,

    fn getOptions(b: *std.Build) Options {
        return .{
            .target = b.standardTargetOptions(.{}),
            .optimize = b.standardOptimizeOption(.{}),
            .linkage = b.option(std.builtin.LinkMode, "linkage", "Linkage mode") orelse std.builtin.LinkMode.static,
            .profiling = b.option(bool, "profiling", "Enable profiling") orelse false,
            .installTests = b.option(bool, "install-tests", "Install tests binary") orelse false,
            .includeDemos = b.option(bool, "include-demos", "Build demo projects") orelse true,
        };
    }

    fn forBandura(opts: Options) bandura.Options {
      return bandura.Options {
        .target =  opts.target,
        .optimize = opts.optimize,
        .linkage = opts.linkage,
      };
    }

    fn forTests(opts: Options) tests.Options {
      return tests.Options {
        .target = opts.target,
        .optimize = opts.optimize,
        .installBinaries = opts.installTests,
      };
    }

    fn forProfiler(opts: Options) profiler.Options {
      return .{
        .target = opts.target,
        .optimize = opts.optimize
      };
    }

};

pub fn build(b: *std.Build) !void {
    const options = Options.getOptions(b);

    var targets = try b.allocator.alloc(*std.Build.Step.Compile, 2);

    targets[0] = try defaultStep(b, options);
    targets[1] = try testsStep(b, options);

    _ = cc.createStep(b, "cdb", targets[0..]);
}

fn defaultStep(b: *std.Build, options: Options) !*std.Build.Step.Compile {
    const lib = try bandura.buildLibrary(b, options.forBandura());
    b.installArtifact(lib);

    return lib;
}

fn testsStep(b: *std.Build, options: Options) !*std.Build.Step.Compile {
  const banduraOpts = options.forBandura();
  const banduraModule = try bandura.createBaseModule(b, banduraOpts);

  common.enableProfiling(banduraModule);
  common.enableTests(banduraModule);

  banduraModule.addIncludePath(b.path("tests"));

  const profilerModule = try profiler.createModule(b, options.forProfiler());
  common.enableTests(profilerModule);
  profilerModule.addIncludePath(b.path("tests"));

  const banduraLib = bandura.addLibrary(b, banduraModule, banduraOpts);
  const profilerLib = profiler.addLibrary(b, profilerModule);

  const testsOpts = options.forTests();
  const testsModule = try tests.createModule(b, testsOpts);
  testsModule.linkLibrary(banduraLib);
  testsModule.linkLibrary(profilerLib);

  var step = b.step("test", "Run tests");

  const exe = tests.createExe(b, testsModule);
  var artifact = tests.createArtifact(b, testsOpts, exe);
  step.dependOn(&artifact.step);

  return exe;
}
