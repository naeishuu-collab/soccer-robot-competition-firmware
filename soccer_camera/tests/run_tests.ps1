$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$common = Join-Path $root 'common/src'
$build = Join-Path $env:TEMP 'soccer-competition-tests'
$results = Join-Path $PSScriptRoot 'results_last_run.txt'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$logic = foreach ($sketch in @('main/main.ino', 'ir/ir.ino', 'motor/motor.ino')) {
    $source = Get-Content -Raw -Encoding UTF8 (Join-Path (Join-Path $root 'legacy_before_chase') $sketch)
    if ($source -notmatch '(?s)// BEGIN HOST-TESTED SKETCH LOGIC\s*(.*?)\s*// END HOST-TESTED SKETCH LOGIC') { throw "Missing testable sketch logic: $sketch" }
    $Matches[1]
}
$logic | Set-Content -Encoding UTF8 (Join-Path $build 'integrated_sketch_logic.h')
$mainSource = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'main/main.ino')
if ($mainSource -notmatch '(?s)// BEGIN HOST-TESTED CAMERA LOGIC\s*(.*?)\s*// END HOST-TESTED CAMERA LOGIC') { throw 'Missing testable camera logic' }
$Matches[1] | Set-Content -Encoding UTF8 (Join-Path $build 'chase_camera_logic.h')
$motorSource = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'motor/motor.ino')
if ($motorSource -notmatch '(?s)// BEGIN HOST-TESTED DRIVE HANDLER\s*(.*?)\s*// END HOST-TESTED DRIVE HANDLER') { throw 'Missing testable drive handler' }
$Matches[1] | Set-Content -Encoding UTF8 (Join-Path $build 'chase_drive_handler.h')
if ($motorSource -notmatch '(?s)// BEGIN HOST-TESTED MIXER\s*(.*?)\s*// END HOST-TESTED MIXER') { throw 'Missing testable wheel mixer' }
$mixer = $Matches[1]
$motorConfig = foreach ($name in @('MOTOR_IN1', 'MOTOR_IN2', 'WHEEL_DEGREES', 'WHEEL_SIGN')) {
    if ($motorSource -notmatch "(?s)(?:const uint8_t|constexpr int(?:8|16)_t)\s+$name\[4\]\s*=\s*\{[^}]+\};") { throw "Missing motor config: $name" }
    $Matches[0]
}
($motorConfig + $mixer) | Set-Content -Encoding UTF8 (Join-Path $build 'chase_mixer.h')
"Host tests started at $([DateTime]::UtcNow.ToString('yyyy-MM-dd HH:mm:ss')) UTC" | Set-Content -Encoding Unicode $results

$sources = @('test_core.cpp', 'test_integration.cpp', 'test_chase_camera.cpp', 'test_chase_motor.cpp', 'test_chase_mixer.cpp', 'test_motor_model.cpp', 'test_upgrade.cpp', 'test_review.cpp', 'test_loop.cpp', 'test_loop_legacy.cpp', 'test_continuity.cpp', 'test_zip_loop.cpp', 'test_default_omni.cpp', 'test_default_bno.cpp')
$gpp = Get-Command g++ -ErrorAction SilentlyContinue
$cl = Get-Command cl -ErrorAction SilentlyContinue

if ($gpp) {
    foreach ($source in $sources) {
        $name = [IO.Path]::GetFileNameWithoutExtension($source)
        $exe = Join-Path $build "$name.exe"
        "=== BUILD $source ===" | Tee-Object -FilePath $results -Append
        $output = & $gpp.Source -finput-charset=UTF-8 -std=c++17 -Wall -Wextra -I $common -I $build -I (Join-Path $root 'motor') -I (Join-Path $root 'main') -I (Join-Path $root 'ir') -I $PSScriptRoot -I (Join-Path $PSScriptRoot 'mock') (Join-Path $PSScriptRoot $source) -o $exe 2>&1
        $exitCode = $LASTEXITCODE
        $output | Tee-Object -FilePath $results -Append
        if ($exitCode -ne 0) { throw "Compile failed: $source" }
        "=== RUN $source ===" | Tee-Object -FilePath $results -Append
        $output = & $exe 2>&1
        $exitCode = $LASTEXITCODE
        $output | Tee-Object -FilePath $results -Append
        if ($exitCode -ne 0) { throw "Test failed: $source" }
    }
    exit 0
}

if (-not $cl) {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
    if (-not (Test-Path $vswhere)) { throw 'No g++ or Visual C++ compiler found.' }
    $vs = & $vswhere -latest -products '*' -property installationPath
    if (-not $vs) { throw 'Visual Studio C++ tools are not installed.' }
    $toolRoot = Join-Path $vs 'VC/Tools/MSVC'
    $toolVersion = Get-ChildItem $toolRoot -Directory |
        Sort-Object { [version]$_.Name } -Descending |
        Where-Object { (Test-Path (Join-Path $_.FullName 'bin/Hostx64/x64/cl.exe')) -and (Test-Path (Join-Path $_.FullName 'include/vcruntime.h')) } |
        Select-Object -First 1
    if (-not $toolVersion) { throw 'MSVC toolset not found.' }
    $clPath = Join-Path $toolVersion.FullName 'bin/Hostx64/x64/cl.exe'
    $sdkRoot = Join-Path ${env:ProgramFiles(x86)} 'Windows Kits/10'
    $sdkVersion = Get-ChildItem (Join-Path $sdkRoot 'Include') -Directory | Sort-Object { [version]$_.Name } -Descending | Select-Object -First 1
    if (-not (Test-Path $clPath) -or -not $sdkVersion) { throw 'MSVC or Windows SDK files are incomplete.' }
    $env:PATH = (Join-Path $toolVersion.FullName 'bin/Hostx64/x64') + ';' + $env:PATH
    $env:INCLUDE = @(
        (Join-Path $toolVersion.FullName 'include'),
        (Join-Path $sdkVersion.FullName 'ucrt'),
        (Join-Path $sdkVersion.FullName 'shared'),
        (Join-Path $sdkVersion.FullName 'um'),
        (Join-Path $sdkVersion.FullName 'winrt'),
        (Join-Path $sdkVersion.FullName 'cppwinrt')
    ) -join ';'
    $env:LIB = @(
        (Join-Path $toolVersion.FullName 'lib/x64'),
        (Join-Path $sdkRoot "Lib/$($sdkVersion.Name)/ucrt/x64"),
        (Join-Path $sdkRoot "Lib/$($sdkVersion.Name)/um/x64")
    ) -join ';'
} else {
    $clPath = $cl.Source
}

Push-Location $build
try {
    foreach ($source in $sources) {
        $name = [IO.Path]::GetFileNameWithoutExtension($source)
        "=== BUILD $source ===" | Tee-Object -FilePath $results -Append
        $output = & $clPath /nologo /EHsc /utf-8 /std:c++17 /W4 "/I$common" "/I$build" "/I$(Join-Path $root 'motor')" "/I$(Join-Path $root 'main')" "/I$(Join-Path $root 'ir')" "/I$PSScriptRoot" "/I$(Join-Path $PSScriptRoot 'mock')" (Join-Path $PSScriptRoot $source) "/Fe:$name.exe" 2>&1
        $exitCode = $LASTEXITCODE
        $output | Tee-Object -FilePath $results -Append
        if ($exitCode -ne 0) { throw "Compile failed: $source" }
        "=== RUN $source ===" | Tee-Object -FilePath $results -Append
        $output = & (Join-Path $build "$name.exe") 2>&1
        $exitCode = $LASTEXITCODE
        $output | Tee-Object -FilePath $results -Append
        if ($exitCode -ne 0) { throw "Test failed: $source" }
    }
} finally {
    Pop-Location
}
