<div align="center">

# Vecture3D

#### A Box3D fork designed and optimized for the Hytale Server.

[![Build Status](https://github.com/blovien/vecture3d/actions/workflows/build.yml/badge.svg)](https://github.com/blovien/vecture3d/actions)
[![CLA assistant](https://cla-assistant.io/readme/badge/blovien/vecture3d)](https://cla-assistant.io/blovien/vecture3d)

<img width="100%" alt="Vecture3D" src="https://github.com/user-attachments/assets/b6f395d9-5218-4329-afea-1d85fa570e33" />

</div>


## Source layout

- `include/vecture3d/`: public engine extensions.
- `src/block_grid/`: BlockGrid implementation, private headers, and collision certification helpers.
- `abi/include/vecture3d/`: public native ABI headers.
- `abi/src/`: ABI implementation and private headers.
- `abi/test/`: native ABI tests.
- `abi/abi_manifest.json`: generated ABI contract, refreshed with `./gradlew updateAbiManifest`.
- `ffi-java/`: Java FFM bindings and consumer tests.

## Features

Vecture3D includes all features synced by the upstream repo: [Box3D's features](https://github.com/erincatto/box3d#features).

## Building all platforms

- Install [CMake](https://cmake.org/)
- Install [git](https://git-scm.com/)
- (TODO: precise java dependencies)
- Ensure these run from the command line

## Building with CMake presets (recommended)

This uses the presets in `CMakePresets.json`.

- Windows: `cmake --preset windows` then `cmake --build --preset windows-release`
- Linux: `cmake --preset linux-release` then `cmake --build --preset linux-release`
- macOS: `cmake --preset macos` then `cmake --build --preset macos-release`
- Windows MinGW: `cmake --preset mingw-release` then `cmake --build --preset mingw-release`

Run the samples app (must be in the Box3D directory).

- Windows: `.\build\bin\Release\samples.exe`
- Linux: `./build/bin/samples`
- macOS: `./build/bin/Release/samples`

## Building for Visual Studio

- Install [Visual Studio](https://visualstudio.microsoft.com/)
- Run `build_vs2026.bat`
- Open and build `build/box3d.slnx`

## Building for Linux

- Run `build.sh` from a bash shell
- Results are in the build sub-folder

## Building for Xcode

- mkdir build
- cd build
- cmake -G Xcode ..
- Open `box3d.xcodeproj`
- Select the samples scheme
- Build and run the samples

## Building for Web

- [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html)
- `emcmake cmake -B build -DBOX3D_SAMPLES=OFF`
- `cmake --build build`

Box3D uses SSE2 with WebAssembly. Define `BOX3D_DISABLE_SIMD` to disable SSE2.

## Building and installing

- mkdir build
- cd build
- cmake ..
- cmake --build . --config Release
- cmake --install . (might need sudo)

## Using Vecture3D in your Hytale plugin

Vecture3D will provide an higher level Hytale API that is coming soon.

## Compatibility

The Vecture3D library and samples build and run on Windows, Linux, and Mac.

You will need a compiler that supports C17 to build the Vecture3D library.

You will need a compiler that supports C++20 to build the samples.

Vecture3D uses SSE2 and Neon SIMD math to improve performance. SIMD can be disabled by defining `BOX3D_DISABLE_SIMD`.

## Documentation

The user manual lives in [`docs/`](docs/) and is built with Doxygen. Enable the `BOX3D_DOCS` CMake option and build the `doc` target.

## Community

- [Box3D's discord](https://discord.gg/NKYgCBP)
- [HytaleModding's discord](https://discord.gg/tanu3KxN85)

## Contributing

Pull requests are welcomed if they follow a created issue that describes precisely the problem or feature request.

## Giving feedback

Before submitting an issue here please understand if the issue is related to Box3D or Vecture3D modifications. 
- For the former: please file an issue or start a chat on discord. You can also use [GitHub Discussions](https://github.com/erincatto/box3d/discussions).
- For the latter: file an issue in this repo
## License

Vecture3D is developed by Blovien and it's a forked work of Box3D developed by Erin Catto. Both uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License).

## LLM Usage

Vecture3D follows the same LLM rules as its upstream:

LLMs are used in the following areas:

- unit tests
- samples app
- migrating code between Box2D and Box3D
- build configuration
- code reviews
- benchmarking
- (vecture3d) abi commands in java ffi

Elsewhere all code is developed and written by me. I take responsibility for every line of code in Vecture3D.
