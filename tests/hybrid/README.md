# Standalone verification

From the repository root, with installed VS 2026 C++ build tools:

```powershell
cmake -S tests/hybrid -B x64/tests-build -G "Visual Studio 18 2026" -A x64
cmake --build x64/tests-build --config Release
x64/tests-build/Release/hybrid_tests.exe
```

If the Windows sandbox rejects MSBuild's FileTracker initialization, set
`$env:TrackFileAccess='false'` in the shell before configure/build. This disables
MSBuild file tracking, not compilation or testing. No downloads are required.
The core is also compatible with other C++17 CMake generators.

The executable supplies the simulator runner and tiny exception-based assertion
harness. Assertions remain active in Release. It prints metrics and exits
nonzero on any failed group; CTest is registered as well.

The end-to-end test sends simulated quantized IMU frames through the parser,
scaling, clock sync and fusion. Calibration uses separate guided motion and its
recovered result feeds the other profiles. Overall raw optical RMS holds the
last raw pose through face loss and includes outliers. The separately reported
visible clean comparison excludes face-lost frames and known synthetic outliers
from **both** estimators; it still includes camera latency, noise and the hold
between camera frames. Orientation errors are SO(3) angular distances against
truth at the 200 Hz output time, not component-wise Euler differences.
