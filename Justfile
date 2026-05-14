release prefix="zig-out":
  zig build --release=fast -Dinclude-demos=false --prefix {{prefix}}
