# Ghostbusters dev shortcuts. Run `just` to list recipes.

build_dir := "build"
build_type := "RelWithDebInfo"

default:
    @just --list

# Configure (if needed) and build after changes
rebuild:
    cmake -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{build_type}} -DGLFW_BUILD_WAYLAND=OFF -DBOX2D_SAMPLES=ON -DBOX2D_UNIT_TESTS=ON
    cmake --build {{build_dir}} -j

# Build, then open the samples app (run from repo root so settings.ini/imgui.ini are found)
samples: rebuild
    ./{{build_dir}}/bin/samples

# Build, then run unit tests. Optional filter, e.g. `just test GhostbustersTest`
test filter="": rebuild
    ./{{build_dir}}/bin/test {{filter}}

# Wipe the build directory
clean:
    rm -rf {{build_dir}}
