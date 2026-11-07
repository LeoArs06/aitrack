# Qt hybrid adapter

`Presenter` owns a QThread and its ImuWorker. All serial, parser, clock, fusion,
calibration, simulator, CSV and Hybrid UDP operations execute on that thread.
Camera poses and commands are queued by value; diagnostics and calibration
results return to the GUI by queued signals. Both threads use HostClock.h.

Changing to Hybrid suppresses camera sends immediately. Changing to Webcam
waits for the worker's output-disable acknowledgement before camera sends resume.
Command generations discard acknowledgements from earlier mode changes. Stopping
tracking disables output and cancels calibration, retaining the USB connection;
Webcam selection disconnects USB. Shutdown queues port/socket cleanup and waits
at most 3 seconds, with deferred cleanup if that limit is exceeded.

Auto tries the last successful port in this session first, then preferred Arduino /
SparkFun VIDs, then other ports. Each probe is asynchronous, at 115200 baud with
DTR asserted and a `?` request. Only validated v1 INFO with the fixed v1 ranges
and 200 Hz rate enables sample ingestion. A new connection resets parser/clock;
fusion history survives outages for optical fallback. Counters are per connection.

Set `AITRACK_IMU_SIM=1` before launching AITrack, select Hybrid IMU, Connect, then
start tracking. No serial port is opened. A real-time 200 Hz timer replays the
core Simulator's guided-calibration motion through encoder, parser, scaling and
clock sync. Its 30-second sequence repeats with a marked discontinuity. The app
continues using the real camera for optical poses; simulated rotation therefore
does not follow the user's physical head. The adapter tests supply matching
synthetic optical poses to test guided calibration without a camera.

Main-window mode, port and strength changes sync prefs.ini immediately. Successful
calibration is applied on the worker and its serialized result is saved on the
GUI thread. Set `hybrid_log=true` in prefs.ini before launch to record
`logs/hybrid_<timestamp>.csv`, relative to the working directory. Rows share a
19-column header: `imu` rows contain host arrival microseconds and raw wire data;
`optical` rows contain capture/get_frame arrival microseconds, validity and the
six Presenter values (yaw, pitch, roll, x, y, z). Empty columns are inapplicable.
Full diagnostics remain available in the status label's tooltip during prompts.

## Verification

From PowerShell in the repository (Qt 5.15.2 / VS 2026 Build Tools):

```powershell
$env:QTDIR='C:\Users\leoar\Desktop\headtracker\software\deps\Qt\5.15.2\msvc2019_64'
& 'C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\MSBuild\Current\Bin\MSBuild.exe' Camera.sln -m -p:Configuration=Release -p:Platform=x64 -p:PlatformToolset=v145 "-p:QtInstall=$env:QTDIR" -p:WindowsTargetPlatformVersion=10.0.26100.0 -p:TrackFileAccess=false -p:ForceRebuild=true
Copy-Item "$env:QTDIR\bin\Qt5SerialPort.dll" x64/Release/Qt5SerialPort.dll -Force
```

The checked-in toolset remains v142. The two tracking overrides avoid this
sandbox's FileTracker denial, and do not alter project source. For CMake compiler
discovery, set them as environment properties too:

```powershell
$env:TrackFileAccess='false'; $env:ForceRebuild='true'
cmake -S tests/hybrid -B x64/tests-build -DCMAKE_SUPPRESS_REGENERATION=ON
cmake --build x64/tests-build --config Release
& x64/tests-build/Release/hybrid_tests.exe
cmake -S tests/hybrid_qt -B x64/hybrid-qt-tests-build "-DCMAKE_PREFIX_PATH=$env:QTDIR"
cmake --build x64/hybrid-qt-tests-build --config Release
$env:PATH="$env:QTDIR\bin;$env:PATH"
& x64/hybrid-qt-tests-build/Release/hybrid_qt_tests.exe
& x64/hybrid-qt-tests-build/Release/calibration_qt_tests.exe
& x64/hybrid-qt-tests-build/Release/serial_retry_qt_tests.exe
& x64/hybrid-qt-tests-build/Release/hybrid_ui_tests.exe
```

Tests exercise real worker threads and UDP sockets, packet units/order, live
destination changes, 200 Hz output, optical fallback, cancellation, successful
guided calibration and serialization, CSV contents, output ownership ordering,
absent-port retry and bounded shutdown. The UI test renders the actual .ui with
the Windows platform plugin and saves a PNG. Hardware unplug/replug, camera
latency and mounting quality still require a physical device.
