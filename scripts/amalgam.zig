const std = @import("std");

const Allocator = std.mem.Allocator;
const Writer = std.Io.Writer;

const SourceFile = struct {
  name: []const u8,
  contents: []u8,
};

const MaxFileCount : u32 = 32;

const IncludeStatement = "#include";
const IncludeStatementLen : u32 = IncludeStatement.len;

const IgnoreIncludes : [5][]const u8 = .{
  "#include \"bandura.h\"",
  "#include \"bnd-core.h\"",
  "#include \"bnd-math.h\"",
  "#include \"profiler.h\"",
  "#include \"testing.h\"",
};

const Headers = enum(u32) {
  bandura,
  bnd_core,
  bnd_math,
  profiler,

  count,
};

pub fn createStep(b: *std.Build) !*std.Build.Step {
  const step = try b.allocator.create(std.Build.Step);
  step.* = std.Build.Step.init(.{
    .id = .custom,
    .name = "amalgam",
    .makeFn = amalgamate,
    .owner = b,
  });

  return step;
}

fn amalgamate(step: *std.Build.Step, make_options: std.Build.Step.MakeOptions) anyerror!void {
  _ = make_options;

  const b = step.owner;
  var arenaAllocator = std.heap.ArenaAllocator.init(b.allocator);
  defer _ = arenaAllocator.reset(.free_all);

  var threaded = std.Io.Threaded.init_single_threaded;
  const iop = threaded.io();
  const cwd = b.build_root.handle;

  const arena = arenaAllocator.allocator();
  const sources = try readSourceFiles(arena, cwd, iop);

  const dstFile = try cwd.createFile(iop, "bandura.c", .{ .truncate = true });
  defer dstFile.close(iop);

  try dstFile.setLength(iop, 0);
  var writer = dstFile.writerStreaming(iop, try arena.alloc(u8, 1024));

  try collectStdIncludes(sources, &writer.interface, b.allocator);
}

fn collectStdIncludes(sources: []SourceFile, writer: *Writer, allocator: Allocator) !void {
  defer writer.flush();

  var set = std.BufSet.init(allocator);
  defer set.deinit();

  for(sources) |srcFile| {
    const fileContents = srcFile.contents;

    var lineStartPos : u64 = 0;
    var lineEndPos = lineStartPos + 1;
    while (lineStartPos < fileContents.len) {
      lineEndPos = lineStartPos + 1;
      while(lineEndPos < fileContents.len and fileContents[lineEndPos] != '\n') {
        lineEndPos += 1;
      }

      if (fileContents[lineStartPos] != '#' or lineStartPos > fileContents.len - IncludeStatementLen) {
        lineStartPos = lineEndPos + 1;
        continue;
      }

      if (!std.mem.eql(u8, fileContents[lineStartPos..(lineStartPos + IncludeStatementLen)], IncludeStatement)) {
        lineStartPos = lineEndPos + 1;
        continue;
      }

      var shouldIgnore = false;

      const statement = fileContents[lineStartPos..lineEndPos];
      for(IgnoreIncludes) |ignore| {
        if (std.mem.eql(u8, ignore, statement)) {
          shouldIgnore = true;
          break;
        }
      }

      if (!shouldIgnore and !set.contains(statement)) {
        try set.insert(statement);

        _ = try writer.write(statement);
        _ = try writer.writeByte('\n');
      }

      lineStartPos = lineEndPos + 1;
    }
  }

}

fn readSourceFiles(arena: Allocator, cwd: std.Io.Dir, io: std.Io) ![]SourceFile {
  var files = try arena.alloc(SourceFile, MaxFileCount);

  var fileCount : u32 = @intFromEnum(Headers.count);

  {
    const includeDir = try cwd.openDir(io, "include", .{});
    defer includeDir.close(io);

    files[@intFromEnum(Headers.bandura)] = try readFile(arena, io, includeDir, "bandura.h");
    files[@intFromEnum(Headers.bnd_math)] = try readFile(arena, io, includeDir, "bnd-math.h");
    files[@intFromEnum(Headers.profiler)] = try readFile(arena, io, includeDir, "profiler.h");
  }

  {
    const srcDir = try cwd.openDir(io, "src", .{ .iterate = true });
    defer srcDir.close(io);

    files[@intFromEnum(Headers.bnd_core)] = try readFile(arena, io, srcDir, "bnd-core.h");

    var iterator = srcDir.iterate();
    while (try iterator.next(io)) |entry| {
      if (fileCount >= MaxFileCount) {
        @panic("Source files limit exceeded");
      }

      if (entry.kind != .file) {
        continue;
      }

      const ext = std.fs.path.extension(entry.name);
      if (!std.mem.eql(u8, ".c", ext)) {
        continue;
      }

      files[fileCount] = try readFile(arena, io, srcDir, try arena.dupe(u8, entry.name));
      fileCount += 1;
    }
  }

  return files[0..fileCount];
}

fn readFile(arena: Allocator, io: std.Io, dir: std.Io.Dir, name: []const u8) !SourceFile {
    const file = try dir.openFile(io, name, .{ .mode = .read_only });
    defer file.close(io);

    const fileLength = try file.length(io);

    const fileContents = try arena.alloc(u8, fileLength);

    var reader = file.reader(io, fileContents);
    try reader.interface.fillMore();

    return .{ .name = name, .contents = fileContents };
}
