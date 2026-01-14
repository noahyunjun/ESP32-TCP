# Copilot / AI agent instructions for this repo

This is a small PlatformIO project targeting an ESP32 (Arduino framework).
Follow these concise rules to be immediately productive in the codebase.

1. Big picture
- This repository is a PlatformIO embedded project for an ESP32 board. See `[platformio.ini](platformio.ini)` for the active environment `esp32-gateway` (platform: `espressif32`, framework: `arduino`).
- The application entrypoint is `src/main.cpp` (Arduino `setup()`/`loop()` style). Headers belong in `include/` and local libraries under `lib/`.

2. Build / test / upload workflows (concrete commands)
- Build for the configured env: `platformio run -e esp32-gateway`
- Upload to device: `platformio run -e esp32-gateway -t upload` (or `pio run -e esp32-gateway -t upload`).
- Run unit tests (PlatformIO test runner): `platformio test`
- Clean build files: `platformio run -t clean`

3. Project-specific conventions
- Keep MCU code in `src/` using Arduino-style `setup()` and `loop()`.
- Put public headers in `include/` and follow typical C/C++ header guards.
- Local libraries: each library in `lib/YourLibName/` with `src/` and optional `library.json` if custom build flags are required.
- Do not rename or remove the environment `esp32-gateway` without updating `platformio.ini` and CI scripts.

4. Code patterns and examples
- Minimal function example exists in `src/main.cpp`: `int myFunction(int x, int y) { return x + y; }` — prefer small, testable functions.
- Use `#include <Arduino.h>` for Arduino APIs and use PlatformIO's LDF to auto-resolve dependencies.

5. Debugging notes
- PlatformIO supports debugger integration for supported boards. Use VS Code PlatformIO's Debug panel or `platformio debug -e esp32-gateway` (hardware support varies).
- For serial logging use `Serial.begin(baud)` in `setup()` and view output via `platformio device monitor`.

6. Integration points & external dependencies
- No external network/cloud integrations are present in this repository; add any third-party libs under `lib/` or declare them in `platformio.ini` using `lib_deps`.

7. What to avoid / quick pitfalls
- The project is PlatformIO/Arduino — avoid introducing ESP-IDF-only APIs unless you update `framework` in `platformio.ini`.
- Keep `include/` for cross-translation-unit headers only; don't duplicate headers inside `src/` subfolders.

8. When you need to change build/config
- Edit `platformio.ini` to add environments or `lib_deps`. Run full build after changes to confirm LDF behavior.

If anything above is unclear or you want extra sections (CI, flashing notes, or example unit tests), tell me which parts to expand.
