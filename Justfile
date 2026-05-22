lib:
  zig build -Dinclude-demos=false

release prefix="zig-out":
  zig build --release=fast -Dinclude-demos=false --prefix {{prefix}}
  zig build --release=fast -Dlinkage=dynamic -Dinclude-demos=false --prefix {{prefix}}

cmake:
  cmake -S . -B build
  cmake --build build
