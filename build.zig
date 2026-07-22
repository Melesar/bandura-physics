const std = @import("std");
const common = @import("scripts/common.zig");
const bandura = @import("src/build.zig");
const tests = @import("tests/build.zig");
const profiler = @import("profiler/build.zig");

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

  try defaultStep(b, options);
  try testsStep(b, options);
}

fn defaultStep(b: *std.Build, options: Options) !void {
    b.installArtifact(try bandura.buildLibrary(b, options.forBandura()));
}

fn testsStep(b: *std.Build, options: Options) !void {
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

  var artifact = tests.createArtifact(b, testsOpts, testsModule);
  step.dependOn(&artifact.step);
}
