const std = @import("std");

pub const ResolvedTarget = std.Build.ResolvedTarget;

const FlagsArray = std.ArrayList([]const u8);

const FLAGS = &.{ "-std=c99", "-Wall", "-Wextra", "-pedantic", "-Werror=format", "-Werror=shadow", "-Werror=incompatible-pointer-types", "-Werror=pointer-type-mismatch", "-Werror=return-type", "-Wno-unused-parameter", "-Wno-braced-scalar-init" };

pub const CompileFlags = struct {
    flags: FlagsArray,
    allocator: std.mem.Allocator,

    pub fn default(allocator: std.mem.Allocator) !CompileFlags {
        var value = CompileFlags{
            .flags = try FlagsArray.initCapacity(allocator, 16),
            .allocator = allocator,
        };

        try value.flags.appendSlice(value.allocator, FLAGS);

        return value;
    }

    pub fn add(self: *CompileFlags, flag: []const u8) !void {
        try self.flags.append(self.allocator, flag);
    }

    pub fn addSlice(self: *CompileFlags, flags: []const []const u8) !void {
        try self.flags.appendSlice(self.allocator, flags);
    }

    pub fn addOptimizations(self: *CompileFlags, optimize: std.builtin.OptimizeMode) !void {
        switch (optimize) {
            .Debug => {
                try self.flags.appendSlice(self.allocator, &.{ "-g", "-O0", "-DBND_DEBUG" });
            },

            .ReleaseSafe, .ReleaseFast => {
                try self.flags.append(self.allocator, "-O2");
            },

            .ReleaseSmall => {
                try self.flags.append(self.allocator, "-Os");
            },
        }
    }

    pub fn collect(self: *CompileFlags) ![]const []const u8 {
        return self.flags.toOwnedSlice(self.allocator);
    }
};

pub fn enableProfiling(module: *std.Build.Module) void {
    module.addCMacro("BND_PROFILING", "");
}

pub fn enableTests(b: *std.Build, module: *std.Build.Module) void {
    module.addCMacro("BND_TESTS", "");
    module.addIncludePath(b.path("tests"));
}

pub fn collectSources(b: *std.Build, directory: []const u8) ![]const []const u8 {
    var sources = try std.ArrayList([]const u8).initCapacity(b.allocator, 16);
    errdefer sources.deinit(b.allocator);

    var threaded = std.Io.Threaded.init_single_threaded;
    const io = threaded.io();

    var dir = try b.build_root.handle.openDir(io, directory, .{ .iterate = true });
    defer dir.close(io);

    var iter = dir.iterate();
    while (try iter.next(io)) |entry| {
        if (entry.kind != .file) continue;
        if (std.mem.lastIndexOf(u8, entry.name, ".c") == null) continue;

        try sources.append(b.allocator, b.pathJoin(&.{ directory, entry.name }));
    }

    return sources.toOwnedSlice(b.allocator);
}
