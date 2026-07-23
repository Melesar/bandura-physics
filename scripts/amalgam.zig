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

const BanduraParsingState = enum {
  none,
  api_define,
  math_types,
  extern_c,
  extern_c_end
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

  var writer = dstFile.writerStreaming(iop, try arena.alloc(u8, 1024));

  try collectStdIncludes(sources, &writer.interface, b.allocator);

  try writeBanduraHeader(sources[@intFromEnum(Headers.bandura)], &writer.interface);
  try writeProfilerHeader(sources[@intFromEnum(Headers.profiler)], &writer.interface);
  try writeHeaderFile(sources[@intFromEnum(Headers.bnd_core)], &writer.interface);
  try writeHeaderFile(sources[@intFromEnum(Headers.bnd_math)], &writer.interface);

  for(@intFromEnum(Headers.count)..sources.len) |i| {
    try writeSourceFile(sources[i], &writer.interface);
  }

  try writer.flush();
}

fn collectStdIncludes(sources: []SourceFile, writer: *Writer, allocator: Allocator) !void {
  var set = std.BufSet.init(allocator);
  defer set.deinit();

  for(sources) |srcFile| {
    const fileContents = srcFile.contents;

    var lineStartPos : u64 = 0;
    while (readLine(&lineStartPos, fileContents)) |line| {
      if (line.len == 0) {
        continue;
      }

      if (line[0] != '#' or line.len < IncludeStatementLen) {
        continue;
      }

      if (!std.mem.eql(u8, line[0..IncludeStatementLen], IncludeStatement)) {
        continue;
      }

      var shouldIgnore = false;

      for(IgnoreIncludes) |ignore| {
        if (std.mem.eql(u8, ignore, line)) {
          shouldIgnore = true;
          break;
        }
      }

      if (!shouldIgnore and !set.contains(line)) {
        try set.insert(line);

        _ = try writer.write(line);
        _ = try writer.writeByte('\n');
      }
    }
  }
}

fn writeBanduraHeader(source: SourceFile, writer: *Writer) !void {
  var lineStartPos : u64 = 0;
  var skipLine = false;
  var parsingState = BanduraParsingState.none;
  var externCCount : u32 = 0;

  try fileHeaderStart(writer, "bandura.h");

  while(readLine(&lineStartPos, source.contents)) |line| {
    if (line.len == 0) {
      _ = try writer.writeByte('\n');
      continue;
    }

    switch (parsingState) {
      .api_define => {
        if (lineEq(line, "#endif")) {
          parsingState = .none;
        }
        continue;
      },
      .math_types => {
        skipLine = lineEq(line, "#endif");
        if (skipLine) {
          parsingState = .none;
        }
      },
      .extern_c => {
        if (lineEq(line, "#endif")) {
          parsingState = .none;
          if (externCCount == 0) {
            _ = try writer.write("#define BNDAPI\n");
          }
          externCCount += 1;
        }
        continue;
      },
      else => skipLine = false,
    }

    if (skipLine) {
      continue;
    }

    if (lineEq(line, "#ifndef BANDURA_H") or lineEq(line, "#define BANDURA_H")) {
      continue;
    }

    if (isIncludeStatement(line)) {
      continue;
    }

    if (lineEq(line, "#if defined(_WIN32)")) {
      parsingState = .api_define;
      skipLine = true;
      continue;
    }

    if (lineEq(line, "#if !defined(BND_CUSTOM_VEC3)") or lineEq(line, "#if !defined(BND_CUSTOM_QUAT)") or lineEq(line, "#if !defined(BND_CUSTOM_MAT3)")) {
      parsingState = .math_types;
      skipLine = true;
      continue;
    }

    if (lineEq(line, "#if defined(__cplusplus)")) {
      parsingState = .extern_c;
      skipLine = true;
      continue;
    }

    // endif closing the #ifndef BANDURA_H
    if (externCCount > 1 and lineEq(line, "#endif"))  {
      continue;
    }

    _ = try writer.write(line);
    _ = try writer.writeByte('\n');
  }
}

fn writeProfilerHeader(source: SourceFile, writer: *Writer) !void {
  var lineStart : u64 = 0;

  _ = readLine(&lineStart, source.contents);
  _ = readLine(&lineStart, source.contents);

  var insideDefine = false;

  while(readLine(&lineStart, source.contents)) |line| {
    if (isIncludeStatement(line)) {
      continue;
    }

    if (insideDefine and lineEq(line, "#else")) {
      break;
    }

    if (lineEq(line, "#ifndef BND_PROFILING")) {
      insideDefine = true;
      continue;
    }

    _ = try writer.write(line);
    _ = try writer.writeByte('\n');
  }
}

fn writeHeaderFile(source: SourceFile, writer: *Writer) !void {
  try fileHeaderStart(writer, source.name);

  var lineStart : u64 = 0;
  _ = readLine(&lineStart, source.contents);
  _ = readLine(&lineStart, source.contents);

  // Find the trailing #endif and truncate the source before it
  var end = lineStart;
  var i = source.contents.len - 1;
  while(i >= lineStart) : (i -= 1) {
    if (source.contents[i] == '#' and lineEq(source.contents[i..(i+6)], "#endif")) {
      end = i;
      break;
    }
  }

  var start : u64 = 0;
  try writeFile(source.contents[lineStart..end], &start, null, writer);
}

fn writeSourceFile(source: SourceFile, writer: *Writer) !void {
  var startPos : u64 = 0;
  try writeFile(source.contents, &startPos, source.name, writer);
}

fn writeFile(contents: []u8, startPos: *u64, name: ?[]const u8, writer: *Writer) !void {
  if (name) |n| {
    try fileHeaderStart(writer, n);
  }

  while(readLine(startPos, contents)) |line| {
    if (line.len == 0) {
      _ = try writer.writeByte('\n');
      continue;
    }

    if (isIncludeStatement(line)) {
      continue;
    }

    _ = try writer.write(line);
    _ = try writer.writeByte('\n');
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

fn readLine(lineStartPos: *u64, fileContents: []u8) ?[]u8 {
    var lineEndPos : u64 = lineStartPos.* + 1;
    if (lineStartPos.* < fileContents.len) {
      lineEndPos = lineStartPos.*;
      while(lineEndPos < fileContents.len and fileContents[lineEndPos] != '\n') {
        lineEndPos += 1;
      }

      const line = fileContents[lineStartPos.*..lineEndPos];
      lineStartPos.* = lineEndPos + 1;
      return line;
    }

    return null;
}

fn lineEq(line: []const u8, str: []const u8) bool {
  return std.mem.eql(u8, line, str);
}

fn isIncludeStatement(line: []const u8) bool {
  return line.len >= IncludeStatementLen and lineEq(line[0..IncludeStatementLen], IncludeStatement);
}

fn fileHeaderStart(writer: *Writer, name: []const u8) !void {
  _ = try writer.writeByte('\n');
  try writeSeparator(writer);
  _ = try writer.write("//   ");
  _ = try writer.write(name);
  _ = try writer.writeByte('\n');
  try writeSeparator(writer);
}

fn fileHeaderEnd(writer: *Writer) !void {
  try writeSeparator(writer);
}

fn writeSeparator(writer: *Writer) !void {
  _ = try writer.write("// ================\n");
}
