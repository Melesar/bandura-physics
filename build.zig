const std = @import("std");

const common = @import("scripts/build-common.zig");
const amalgam = @import("scripts/amalgam.zig");

const bandura = @import("src/build.zig");
const tests = @import("tests/build.zig");
const scenarios = @import("demos/build.zig");
const benchmarks = @import("benchmarks/build.zig");

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
        return bandura.Options{
            .target = opts.target,
            .optimize = opts.optimize,
            .linkage = opts.linkage,
        };
    }

    fn forTests(opts: Options) tests.Options {
        return tests.Options{
            .target = opts.target,
            .optimize = opts.optimize,
            .installBinaries = opts.installTests,
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

    const defaultBanduraSrc = try amalgamPathToSource(tmpPath, b.allocator, options.optimize);
    const profilingBanduraSrc = try amalgamPathToSource(tmpPath, b.allocator, .ReleaseFast);

    try defaultStep(b, tmpPath, options);
    try targets.append(b.allocator, try testsStep(b, defaultBanduraSrc, options));

    const scenarioOptions = options.forScenarios();
    for (try scenarios.enumerate(b, scenarioOptions)) |s| {
        s.default.addCSourceFile(defaultBanduraSrc);
        s.profiling.addCSourceFile(profilingBanduraSrc);

        const target = s.compile(b);
        try targets.append(b.allocator, target.default);
        try targets.append(b.allocator, target.profiling);

        target.createSteps(b);
    }

    try targets.append(b.allocator, try benchmarks.createStep(b, options.target, tmpPath));

    var compileCommandsStep = cc.createStep(b, "cdb", try targets.toOwnedSlice(b.allocator));
    compileCommandsStep.dependOn(&temp.step);
}

fn defaultStep(b: *std.Build, tempPath: std.Build.LazyPath, options: Options) !void {
    var installStep = b.getInstallStep();

    const installBanduraHeader = b.addInstallHeaderFile(b.path("include/bandura.h"), "bandura.h");
    const installMathHeader = b.addInstallHeaderFile(b.path("include/bnd-math.h"), "bnd-math.h");
    const installAmalgam = b.addInstallFile(tempPath, AmalgamatedSourceName);
    const buildBanduraFromOriginalSources = try bandura.buildLibrary(b, options.forBandura());

    installStep.dependOn(&installAmalgam.step);
    installStep.dependOn(&installBanduraHeader.step);
    installStep.dependOn(&installMathHeader.step);
    installStep.dependOn(&buildBanduraFromOriginalSources.step);
}

fn testsStep(b: *std.Build, banduraSource: std.Build.Module.CSourceFile, options: Options) !*std.Build.Step.Compile {
    const testsOpts = options.forTests();
    const testsModule = try tests.createModule(b, testsOpts);
    testsModule.addCSourceFile(banduraSource);
    testsModule.addIncludePath(b.path("src"));
    testsModule.addCMacro("COLLISION_TEST_SUITE_PATH", "\"tests/collision_test_cases.yaml\"");

    common.enableTests(testsModule);

    var step = b.step("test", "Run tests");

    const exe = tests.createExe(b, testsModule);
    var artifact = tests.createArtifact(b, testsOpts, exe);
    step.dependOn(&artifact.step);

    return exe;
}

fn amalgamPathToSource(path: std.Build.LazyPath, allocator: std.mem.Allocator, optimize: std.builtin.OptimizeMode) !std.Build.Module.CSourceFile {
    var flags = try common.CompileFlags.default(allocator);
    try flags.addOptimizations(optimize);

    return std.Build.Module.CSourceFile{ .file = path, .flags = try flags.collect(), .language = .c };
}
