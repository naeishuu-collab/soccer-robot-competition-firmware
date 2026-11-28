$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
foreach ($board in @('main','ir','motor')) {
    $variant = if ($board -eq 'main') { 'RP2350B' } else { 'RP2350A' }
    Write-Host "$board $variant"
    arduino-cli compile --fqbn "rp2040:rp2040:generic_rp2350:variantchip=$variant,flash=2097152_0" --libraries (Join-Path $root libraries) --warnings all (Join-Path $root $board)
    if ($LASTEXITCODE -ne 0) { throw "$board build failed ($LASTEXITCODE)" }
}
