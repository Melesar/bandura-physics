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

    try targets.append(b.allocator, try bandura.buildLibrary(b, options.forBandura()));

    const amalgamatedSource = try amalgam.amalgamate(b);
    const tmpPath = temp.add(AmalgamatedSourceName, amalgamatedSource);

    var flags = try common.CompileFlags.default(b.allocator);
    try flags.addOptimizations(options.optimize);

    const banduraSource = std.Build.Module.CSourceFile {
      .file = tmpPath,
      .flags = try flags.collect(),
      .language = .c
    };

    try defaultStep(b, tmpPath);
    try targets.append(b.allocator, try testsStep(b, banduraSource, options));

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
  var installStep = b.getInstallStep();
  const installHeader = b.addInstallHeaderFile(b.path("include/bandura.h"), "bandura.h");
  const installAmalgam = b.addInstallFile(tempPath, AmalgamatedSourceName);

  installStep.dependOn(&installAmalgam.step);
  installStep.dependOn(&installHeader.step);
}

fn testsStep(b: *std.Build, banduraSource: std.Build.Module.CSourceFile, options: Options) !*std.Build.Step.Compile {
  const testsOpts = options.forTests();
  const testsModule = try tests.createModule(b, testsOpts);
  testsModule.addCSourceFile(banduraSource);
  testsModule.addIncludePath(b.path("src"));

  common.enableProfiling(testsModule);
  common.enableTests(testsModule);

  var step = b.step("test", "Run tests");

  const exe = tests.createExe(b, testsModule);
  var artifact = tests.createArtifact(b, testsOpts, exe);
  step.dependOn(&artifact.step);

  return exe;
}
