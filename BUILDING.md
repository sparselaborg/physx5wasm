# Building PhysX for WebAssembly

The build uses Emscripten SDK 6.0.3 and produces static WebAssembly libraries for
the `debug`, `checked`, `profile`, and `release` PhysX configurations.

## Docker build

```sh
docker build -t physx5wasm .
./docker-build.ps1
```

The generated libraries are copied to `dist/<configuration>/`.

Pass one or more configurations to build only those variants, for example
`./docker-build.ps1 -Configuration release`. By default the build uses every
processor available to the container; use `-Jobs` to set an explicit limit.

## Local build

With Emscripten SDK 6.0.3 activated:

```sh
bash generate.sh
bash make.sh
```

Pass one or more configuration names to either script to build only those
configurations, for example `bash generate.sh release && bash make.sh release`.

## Visual Studio Code

Press `Ctrl+Shift+B` to configure and build all four configurations. On Windows,
the task uses Git Bash from its default installation location under
`C:\Program Files\Git`.
