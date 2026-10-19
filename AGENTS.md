# Repository Guidelines

## Project Structure & Module Organization

ROCOS-App is a Linux C++17 robot controller library with an HTTP/JSON API and Lua scripting. `src/` contains implementations and most module headers: hardware, robot state, motion planners, controllers, and HTTP services. `include/` holds shared headers. `test/` contains doctest suites and integration programs; `examples/` provides clients. Runtime URDF/YAML files, meshes, and Lua scripts live in `config/`. `web/` contains built UI assets; `docs/` holds API and design documentation. Dependencies are vendored in `3rdparty/`. Generated binaries and libraries belong in `build/bin/` and `build/lib/`.

## Build, Test, and Development Commands

Use CMake 3.15+ and a C++17 compiler. System dependencies include Boost, yaml-cpp, Eigen3, orocos-kdl, urdfdom, TinyXML, and TinyXML2.

```bash
cmake -S . -B build                     # Configure; copy runtime configuration
cmake --build build -j                  # Build libraries, applications, and tests
ctest --test-dir build --output-on-failure  # Run registered tests
ctest --test-dir build -R '^move_joint_test$' --output-on-failure
cd build/bin
./rocosAppMain --http_host=127.0.0.1 --http_port=8080
```

The application reads relative runtime paths, including `robot.urdf` and `scripts/`. Configure the intended hardware or simulator before starting it; current entry-point flags do not include `--sim`.

With CMake 3.21+ and Ninja, use the `debug`, `release`, `relwithdebinfo`, or `minsizerel` presets: `cmake --preset debug`, `cmake --build --preset debug --parallel 2`, then `ctest --preset debug`. Preset builds use separate `build/<preset>/` directories, with runtime files in `build/<preset>/bin/`. Local overrides belong in the ignored `CMakeUserPresets.json`.

## Coding Style & Naming Conventions

Use C++17, RAII, and explicit ownership; prefer `std::unique_ptr`. Follow nearby naming: snake_case filenames, PascalCase classes/public APIs, lowerCamelCase internal functions, and trailing underscores for member variables. Existing contributor instructions specify four spaces without tabs, while `.clang-format` uses Google's two-space default. Preserve surrounding indentation and avoid unrelated formatting changes. Use clang-format on touched code with this discrepancy in mind. Keep explanatory comments concise, generally in Chinese with English technical terms.

## Testing Guidelines

Use the bundled `test/doctest.h`. Name suites `test/<feature>_test.cc` and register executables with `add_test` in `CMakeLists.txt`. Add regression cases for changed motion, controller, or API behavior; no numeric coverage threshold is configured. Some standalone tests are excluded from CTest. Hardware-enabling and robot-motion integration tests require an explicitly prepared hardware/simulator environment.

## Commit & Pull Request Guidelines

History commonly uses `feat:`, `fix:`, `docs:`, `chore:`, and `example:` prefixes. Prefer these with specific, imperative summaries. PRs should describe behavior changes, related issues, validation commands/results, and hardware or simulation assumptions. Include screenshots for UI changes. Update HTTP API documentation when routes or return codes change. Edit source configurations under `config/`, not generated copies in `build/bin/`.

## Repository Skills

Before creating every Git commit, read and follow [rocos-version-maintenance](skills/rocos-version-maintenance/SKILL.md) to check and update the version, including documentation-only and maintenance commits. Determine whether the update is an internal-test increment or a patch increment from the user's explicit instructions; if unspecified, ask before choosing the final version or committing. Do not infer the choice from the current prerelease status or commit type. Include the version update in the same commit; reuse an update already prepared for that commit only when it matches the confirmed update type, rather than incrementing again on retries. Also use this skill for version bumps, internal test builds, release preparation, and SemVer decisions. Version maintenance alone does not authorize a Git commit, tag, push, or release publication.
