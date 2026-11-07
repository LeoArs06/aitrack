# Build ARES Tracker (AITrack hybrid fork) on Windows: dependencies, Qt, MSBuild, deploy.
# Usage (PowerShell, repo root):  .\tools\build.ps1 [-Dist <folder>] [-Tests]
param(
    [string]$QtDir = (Join-Path $PSScriptRoot "..\..\deps\Qt\5.15.2\msvc2019_64"),
    [string]$Dist = "",
    [switch]$Tests
)
$ErrorActionPreference = "Stop"
$repo = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $repo

# 1. Visual Studio Build Tools (2019 or newer, C++ workload)
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw "Visual Studio Build Tools with C++ not found" }
$msbuild = Join-Path $vs "MSBuild\Current\Bin\MSBuild.exe"
$toolset = (Get-ChildItem (Join-Path $vs "MSBuild\Microsoft\VC\*\Platforms\x64\PlatformToolsets\v*") -Directory |
    Sort-Object Name | Select-Object -Last 1).Name
$sdk = (Get-ChildItem "${env:ProgramFiles(x86)}\Windows Kits\10\Include" -Directory |
    Sort-Object Name | Select-Object -Last 1).Name
Write-Host "MSBuild: $msbuild  toolset: $toolset  SDK: $sdk"

# 2. Qt 5.15.2 (aqtinstall, no account needed)
if (-not (Test-Path (Join-Path $QtDir "bin\Qt5SerialPort.dll"))) {
    $qtRoot = Split-Path (Split-Path $QtDir)
    python -m pip install --quiet aqtinstall
    python -m aqt install-qt windows desktop 5.15.2 win64_msvc2019_64 -O $qtRoot
}
$QtDir = (Resolve-Path $QtDir).Path
$env:QTDIR = $QtDir

# 3. Submodule and prebuilt dependencies (same versions as upstream CMakeLists.txt)
git submodule update --init
$deps = Join-Path $repo "Dependencies"
New-Item -ItemType Directory -Force $deps | Out-Null
function Fetch($url, $out) { if (-not (Test-Path $out)) { Invoke-WebRequest $url -OutFile $out } }
if (-not (Test-Path "$deps\onnxruntime")) {
    Fetch "https://github.com/microsoft/onnxruntime/releases/download/v1.4.0/onnxruntime-win-x64-1.4.0.zip" "$deps\onnx.zip"
    Expand-Archive "$deps\onnx.zip" $deps -Force
    Rename-Item "$deps\onnxruntime-win-x64-1.4.0" "onnxruntime"
    Remove-Item "$deps\onnx.zip"
}
$sevenZip = (Get-Command 7z -ErrorAction SilentlyContinue).Source
if (-not $sevenZip) { $sevenZip = "$env:ProgramFiles\7-Zip\7z.exe" }
if (-not (Test-Path "$deps\libusb")) {
    Fetch "https://github.com/libusb/libusb/releases/download/v1.0.23/libusb-1.0.23.7z" "$deps\libusb.7z"
    & $sevenZip x -y "$deps\libusb.7z" "-o$deps\libusb" | Out-Null
    Remove-Item "$deps\libusb.7z"
}
if (-not (Test-Path "$deps\OpenCV")) {
    Fetch "https://github.com/opencv/opencv/releases/download/4.6.0/opencv-4.6.0-vc14_vc15.exe" "$deps\opencv.exe"
    & $sevenZip x -y "$deps\opencv.exe" "-o$deps\OpenCVextracted" | Out-Null
    $b = "$deps\OpenCVextracted\opencv\build"
    New-Item -ItemType Directory -Force "$deps\OpenCV\bin" | Out-Null
    Copy-Item -Recurse "$b\x64\vc15\lib" "$deps\OpenCV\"
    Copy-Item -Recurse "$b\include" "$deps\OpenCV\"
    Copy-Item "$b\include\opencv2\*.hpp" "$deps\OpenCV\include\"
    Copy-Item "$b\x64\vc15\bin\opencv_world460.dll" "$deps\OpenCV\bin\"
    Remove-Item -Recurse -Force "$deps\OpenCVextracted", "$deps\opencv.exe"
}

# 4. Build
& $msbuild Camera.sln -m -nologo -v:minimal -p:Configuration=Release -p:Platform=x64 `
    "-p:PlatformToolset=$toolset" "-p:QtInstall=$QtDir" "-p:WindowsTargetPlatformVersion=$sdk"
if ($LASTEXITCODE -ne 0) { throw "MSBuild failed" }

# 5. Deploy runtime next to AresTracker.exe
$rel = Join-Path $repo "x64\Release"
& "$QtDir\bin\windeployqt.exe" --release --no-translations --no-compiler-runtime "$rel\AresTracker.exe" | Out-Null
Copy-Item "$QtDir\bin\Qt5SerialPort.dll", "$deps\OpenCV\bin\opencv_world460.dll", "$deps\onnxruntime\lib\onnxruntime.dll" $rel -Force
Copy-Item -Recurse -Force (Join-Path $repo "models") $rel

# 6. Optional tests
if ($Tests) {
    cmake -S tests/hybrid -B x64/tests-build -A x64 | Out-Null
    cmake --build x64/tests-build --config Release | Out-Null
    & x64\tests-build\Release\hybrid_tests.exe | Select-Object -Last 1
}

# 7. Optional portable folder (no .lib/.pdb)
if ($Dist) {
    New-Item -ItemType Directory -Force $Dist | Out-Null
    robocopy $rel $Dist /E /XF *.lib *.pdb *.exp prefs.ini log.txt /XD logs | Out-Null
    Write-Host "Portable build: $Dist"
}
Write-Host "OK: $rel\AresTracker.exe"
exit 0
