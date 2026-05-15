const std = @import("std");
const zcc = @import("compile_commands");

const ResolvedTarget = std.Build.ResolvedTarget;

const COMMON_FLAGS = &.{ "-std=c99", "-Wall", "-Wextra", "-Werror=format", "-Werror=shadow", "-Werror=incompatible-pointer-types", "-Werror=return-type", "-Wno-unused-parameter" };

const Options = struct {
    profiling: bool,
    installTests: bool,
    includeDemos: bool,

    fn getOptions(b: *std.Build) Options {
        return .{
            .profiling = b.option(bool, "profiling", "Enable profiling") orelse false,
            .installTests = b.option(bool, "install-tests", "Install tests binary") orelse false,
            .includeDemos = b.option(bool, "include-demos", "Build demo projects") orelse true,
        };
    }
};

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const options = Options.getOptions(b);

    var build_targets = try std.ArrayList(*std.Build.Step.Compile).initCapacity(b.allocator, 16);
    defer build_targets.deinit(b.allocator);

    const banduraLib = try build_bandura(b, options, target, optimize);

    const profiler = try build_profiler(b, target, optimize, false);
    try build_targets.append(b.allocator, profiler);
    if (options.profiling) {
        banduraLib.root_module.linkLibrary(profiler);
    }

    b.installArtifact(banduraLib);
    try build_targets.append(b.allocator, banduraLib);

    if (options.includeDemos) {
        const scenarioSources = try collectSources(b, "demos/scenarios");
        defer b.allocator.free(scenarioSources);

        const binarySources = try collectSources(b, "demos");
        defer b.allocator.free(binarySources);

        const raylib = b.dependency("raylib", .{ .target = target, .optimize = optimize, .config = "-DPLATFORM_DESKTOP -DSUPPORT_PRAND_GENERATOR", .linkage = .dynamic });
        const clay = b.dependency("clay", .{});
        const raygui = b.dependency("raygui", .{});
        for (scenarioSources) |scenarioFile| {
            const scenarioModule = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true });
            const binFlags = try scenarioFlags(b, target.result, optimize);
            defer b.allocator.free(binFlags);

            scenarioModule.addCSourceFiles(.{
                .files = binarySources,
                .flags = binFlags,
            });
            scenarioModule.addCSourceFile(.{ .file = b.path(scenarioFile), .flags = binFlags });

            scenarioModule.addIncludePath(raylib.path("src"));
            scenarioModule.addIncludePath(raylib.path("examples"));
            scenarioModule.addIncludePath(clay.path("."));
            scenarioModule.addIncludePath(raygui.path("src"));
            scenarioModule.addIncludePath(raygui.path("styles"));
            scenarioModule.addIncludePath(b.path("demos/include"));

            const scenarioName = std.fs.path.stem(scenarioFile);
            const scenario = b.addExecutable(.{
                .name = scenarioName,
                .root_module = scenarioModule,
            });

            linkLibraries(scenario, target);

            scenario.linkLibrary(raylib.artifact("raylib"));
            scenario.linkLibrary(banduraLib);

            b.installArtifact(scenario);

            const runScenario = b.addRunArtifact(scenario);
            runScenario.step.dependOn(b.getInstallStep());

            const runStep = b.step(b.fmt("run-{s}", .{scenarioName}), b.fmt("Run {s} scenario", .{scenarioName}));
            runStep.dependOn(&runScenario.step);

            try build_targets.append(b.allocator, scenario);
        }
    }

    const tests = try build_tests(b, options, target, optimize);
    try build_targets.append(b.allocator, tests);

    _ = zcc.createStep(b, "cdb", try build_targets.toOwnedSlice(b.allocator));
}

fn build_bandura(b: *std.Build, options: Options, target: std.Build.ResolvedTarget, optimize: std.builtin.OptimizeMode) !*std.Build.Step.Compile {
    const banduraModule = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });

    banduraModule.addIncludePath(b.path("include"));
    banduraModule.addIncludePath(b.path("src"));

    const libFlags = try libraryFlags(b, options, target.result, optimize);
    defer b.allocator.free(libFlags);

    const banduraSources = try collectSources(b, "src");
    defer b.allocator.free(banduraSources);

    banduraModule.addCSourceFiles(.{
        .files = banduraSources,
        .flags = libFlags,
    });

    const banduraLib = b.addLibrary(.{
        .name = "bandura",
        .linkage = .dynamic,
        .root_module = banduraModule,
    });

    banduraLib.installHeader(b.path("include/bandura.h"), "bandura.h");
    banduraLib.installHeader(b.path("include/bnd-math.h"), "bnd-math.h");

    return banduraLib;
}

fn build_profiler(b: *std.Build, target: std.Build.ResolvedTarget, optimize: std.builtin.OptimizeMode, enable_tests: bool) !*std.Build.Step.Compile {
    const module = b.createModule(.{ .link_libc = true, .target = target, .optimize = optimize });
    var flags = try compilerFlags(b, target.result, optimize);
    errdefer flags.deinit(b.allocator);

    try flags.append(b.allocator, "-DBND_PROFILING");
    if (enable_tests) {
        try flags.append(b.allocator, "-DBND_TESTS");
        module.addIncludePath(b.path("tests"));
    }

    module.addCSourceFiles(.{
        .files = try collectSources(b, "profiler"),
        .flags = try flags.toOwnedSlice(b.allocator),
    });
    module.addIncludePath(b.path("include"));

    const lib = b.addLibrary(.{
        .linkage = .static,
        .name = "bnd_profiler",
        .root_module = module,
    });

    return lib;
}

fn build_tests(b: *std.Build, options: Options, target: std.Build.ResolvedTarget, optimize: std.builtin.OptimizeMode) !*std.Build.Step.Compile {
    const testsModule = b.createModule(.{
        .link_libc = true,
        .target = target,
        .optimize = optimize,
    });
    testsModule.addIncludePath(b.path("tests"));
    testsModule.addIncludePath(b.path("include"));

    var testSources = try std.ArrayList([]const u8).initCapacity(b.allocator, 16);
    errdefer testSources.deinit(b.allocator);

    try testSources.appendSlice(b.allocator, try collectSources(b, "tests"));

    var flags = try std.ArrayList([]const u8).initCapacity(b.allocator, 32);
    errdefer flags.deinit(b.allocator);

    try flags.appendSlice(b.allocator, COMMON_FLAGS);
    try flags.appendSlice(b.allocator, &.{ "-DBND_PROFILING", "-DBND_TESTS" });
    try flags.appendSlice(b.allocator, &.{ "-g", "-O0" });

    testsModule.addCSourceFiles(.{
        .files = try testSources.toOwnedSlice(b.allocator),
        .flags = try flags.toOwnedSlice(b.allocator),
    });

    const profiler = try build_profiler(b, target, optimize, true);
    testsModule.linkLibrary(profiler);

    const tests = b.addExecutable(.{
        .name = "bandura_tests",
        .root_module = testsModule,
    });

    const runTests = b.addRunArtifact(tests);

    if (options.installTests) {
        b.installArtifact(tests);
        runTests.step.dependOn(b.getInstallStep());
    }

    const testsStep = b.step("test", "Run tests");
    testsStep.dependOn(&runTests.step);

    return tests;
}

fn linkLibraries(compile: *std.Build.Step.Compile, target: ResolvedTarget) void {
    switch (target.result.os.tag) {
        .linux => {
            compile.linkSystemLibrary("m");
            compile.linkSystemLibrary("pthread");
            compile.linkSystemLibrary("GLX");
            compile.linkSystemLibrary("X11");
            compile.linkSystemLibrary("Xcursor");
            compile.linkSystemLibrary("Xext");
            compile.linkSystemLibrary("Xfixes");
            compile.linkSystemLibrary("Xi");
            compile.linkSystemLibrary("Xinerama");
            compile.linkSystemLibrary("Xrandr");
            compile.linkSystemLibrary("Xrender");
        },
        .macos => {
            compile.linkFramework("IOKit");
            compile.linkFramework("Cocoa");
            compile.linkFramework("OpenGL");
        },
        else => return,
    }
}

fn libraryFlags(b: *std.Build, options: Options, target: std.Target, optimize: std.builtin.OptimizeMode) ![]const []const u8 {
    var flags = try compilerFlags(b, target, optimize);
    if (options.profiling)
        try flags.append(b.allocator, "-DBND_PROFILING");

    try flags.append(b.allocator, "-fvisibility=hidden");

    return flags.toOwnedSlice(b.allocator);
}

fn scenarioFlags(b: *std.Build, target: std.Target, optimize: std.builtin.OptimizeMode) ![]const []const u8 {
    var flags = try compilerFlags(b, target, optimize);
    return flags.toOwnedSlice(b.allocator);
}

fn compilerFlags(b: *std.Build, target: std.Target, optimize: std.builtin.OptimizeMode) !std.ArrayList([]const u8) {
    var flags = try std.ArrayList([]const u8).initCapacity(b.allocator, 32);
    errdefer flags.deinit(b.allocator);

    var sanitizers = try std.ArrayList([]const u8).initCapacity(b.allocator, 2);
    errdefer sanitizers.deinit(b.allocator);

    try flags.appendSlice(b.allocator, COMMON_FLAGS);

    switch (optimize) {
        .Debug => {
            try flags.appendSlice(b.allocator, &.{ "-g", "-O0", "-DBND_DEBUG" });
        },

        .ReleaseSafe => {
            try flags.appendSlice(b.allocator, &.{"-O2"});
        },

        .ReleaseFast => {
            try flags.appendSlice(b.allocator, &.{"-O3"});
        },

        .ReleaseSmall => {
            try flags.appendSlice(b.allocator, &.{"-Os"});
        },
    }

    if (optimize == .Debug or optimize == .ReleaseSafe) {
        try sanitizers.append(b.allocator, "float-divide-by-zero");

        if ((target.os.tag != .macos or target.cpu.arch != .aarch64) and target.os.tag != .windows) {
            try sanitizers.append(b.allocator, "leak");
        }
    }

    if (sanitizers.items.len > 0) {
        try flags.append(b.allocator, b.fmt("-fsanitize={s}", .{try std.mem.join(b.allocator, ",", try sanitizers.toOwnedSlice(b.allocator))}));
    }

    return flags;
}

fn collectSources(b: *std.Build, directory: []const u8) ![]const []const u8 {
    var sources = try std.ArrayList([]const u8).initCapacity(b.allocator, 16);
    errdefer sources.deinit(b.allocator);

    var dir = try std.fs.cwd().openDir(directory, .{ .iterate = true });
    defer dir.close();

    var iter = dir.iterate();
    while (try iter.next()) |entry| {
        if (entry.kind != .file) continue;
        if (std.mem.lastIndexOf(u8, entry.name, ".c") == null) continue;

        try sources.append(b.allocator, b.pathJoin(&.{ directory, entry.name }));
    }

    return sources.toOwnedSlice(b.allocator);
}
