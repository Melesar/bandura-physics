const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const header = b.addConfigHeader(.{
        .style = .{ .autoconf_at = b.path("CUnit/Headers/CUnit.h.in") },
    }, .{
        .VERSION = "2.1",
        .RELEASE = "3",
    });

    const module = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });
    module.addIncludePath(header.getOutputDir());
    module.addIncludePath(b.path("CUnit/Headers"));
    module.addCSourceFiles(.{
        .files = &.{
            "CUnit/Sources/Framework/CUError.c",
            "CUnit/Sources/Framework/MyMem.c",
            "CUnit/Sources/Framework/TestDB.c",
            "CUnit/Sources/Framework/TestRun.c",
            "CUnit/Sources/Framework/Util.c",
            "CUnit/Sources/Basic/Basic.c",
            "CUnit/Sources/Automated/Automated.c",
            "CUnit/Sources/Console/Console.c",
        },
        .flags = &.{"-std=c99"},
        .language = .c,
    });

    const lib = b.addLibrary(.{
        .name = "cunit",
        .linkage = .static,
        .root_module = module,
    });
    b.installArtifact(lib);
}
