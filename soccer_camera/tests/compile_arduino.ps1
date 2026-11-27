$ErrorActionPreference = 'Continue'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$cli = Get-Command arduino-cli -ErrorAction SilentlyContinue
if ($cli) {
    $cliPath = $cli.Source
} else {
    $cliPath = Join-Path ([Environment]::GetEnvironmentVariable('LOCALAPPDATA')) 'Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
}
if (-not (Test-Path -LiteralPath $cliPath)) { throw "arduino-cli not found: $cliPath" }

$builds = @(
    @{Name='MAIN RP2354B'; Variant='RP2350B'; Sketch=(Join-Path $root 'main')},
    @{Name='IR RP2354A'; Variant='RP2350A'; Sketch=(Join-Path $root 'ir')},
    @{Name='LINE RP2354B'; Variant='RP2350B'; Sketch=(Join-Path $root 'line')},
    @{Name='MOTOR RP2354A'; Variant='RP2350A'; Sketch=(Join-Path $root 'motor')}
)
$log = [System.Collections.Generic.List[string]]::new()
$log.Add("Build timestamp: $([DateTime]::UtcNow.ToString('yyyy-MM-dd HH:mm:ss UTC'))")
$log.Add("Arduino CLI: $(& $cliPath version)")
$log.Add("Core: rp2040:rp2040 6.3.2")
$log.Add('Target: Generic RP2350, selected A/B chip variant, 2MB flash; no upload performed.')
$failed = $false

foreach ($build in $builds) {
    $fqbn = "rp2040:rp2040:generic_rp2350:variantchip=$($build.Variant),flash=2097152_0"
    $log.Add("=== $($build.Name) [$fqbn] ===")
    Write-Host "Building $($build.Name)"
    $output = & $cliPath compile --fqbn $fqbn --warnings all --libraries (Join-Path $root 'libraries') $build.Sketch 2>&1
    $exitCode = $LASTEXITCODE
    foreach ($line in $output) { $log.Add([string]$line) }
    $log.Add("RESULT: $exitCode")
    if ($exitCode -ne 0) { $failed = $true }
}

$logPath = Join-Path $PSScriptRoot 'arduino_compile_current.txt'
Set-Content -LiteralPath $logPath -Value $log -Encoding utf8
Get-Content -LiteralPath $logPath
if ($failed) { exit 1 }
