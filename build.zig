const std = @import("std");

const common = @import("scripts/build-common.zig");
const amalgam = @import("scripts/amalgam.zig");

const bandura = @import("src/build.zig");
const tests = @import("tests/build.zig");
const profiler = @import("profiler/build.zig");
const scenarios = @import("demos/build.zig");

const cc = @import("compile_commands");

const AmalgamatedSourceName = "bandura.c";

const Options = struct {
    target: common.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    linkage: std.builtin.LinkMode,
    installTests: bool,

    fn getOptions(b: *std.Build) Options {
        return .{
            .target = b.standardTargetOptions(.{}),
            .optimize = b.standardOptimizeOption(.{}),
            .linkage = b.option(std.builtin.LinkMode, "linkage", "Linkage mode") orelse std.builtin.LinkMode.static,
            .installTests = b.option(bool, "install-tests", "Install tests binary") orelse false,
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

    fn forScenarios(opts: Options) scenarios.Options {
      return .{
        .target = opts.target,
        .optimize = opts.optimize,
      };
    }

};

pub fn build(b: *std.Build) !void {
    const options = Options.getOptions(b);

    var targets = try std.ArrayList(*std.Build.Step.Compile).initCapacity(b.allocator, 16);

    var temp = b.addTempFiles();

    const amalgamatedSource = try amalgam.amalgamate(b);
    const tmpPath = temp.add(AmalgamatedSourceName, amalgamatedSource);

    try defaultStep(b, tmpPath);
    try targets.append(b.allocator, try testsStep(b, options));

    const scenarioOptions = options.forScenarios();
    for(try scenarios.enumerate(b, scenarioOptions)) |s| {
      s.default.linkLibrary(try bandura.buildLibrary(b, options.forBandura()));
      s.profiling.linkLibrary(try bandura.buildLibrary(b, options.forBandura().forProfiling()));

      const target = s.compile(b);
      try targets.append(b.allocator, target.default);
      try targets.append(b.allocator, target.profiling);

      target.createSteps(b);
    }

    _ = cc.createStep(b, "cdb", try targets.toOwnedSlice(b.allocator));
}

fn defaultStep(b: *std.Build, tempPath: std.Build.LazyPath) !void {
  const installAmalgam = b.addInstallFile(tempPath, AmalgamatedSourceName);

  b.getInstallStep().dependOn(&installAmalgam.step);
}

fn testsStep(b: *std.Build, options: Options) !*std.Build.Step.Compile {
  const banduraOpts = options.forBandura();
  const banduraModule = try bandura.createBaseModule(b, banduraOpts, .off);

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
