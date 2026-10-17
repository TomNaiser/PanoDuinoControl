param(
    [string]$ToolDirectory = (Join-Path $PSScriptRoot '.r4-tools'),
    [string]$SensorLibraryRoot = (Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'Arduino\libraries'),
    [char]$DriveLetter = 'R',
    [switch]$UsbControl,
    [switch]$MotionTraceUsb,
    [switch]$BufferedPanTrace
)

$ErrorActionPreference = 'Stop'
if ($UsbControl -and ($MotionTraceUsb -or $BufferedPanTrace)) { throw 'USB-Steuerung und Bluetooth-Messspur sind getrennte Builds.' }
$toolRoot = (Resolve-Path -LiteralPath $ToolDirectory).Path
$cli = Join-Path $toolRoot 'arduino-cli.exe'
$core = Join-Path $toolRoot 'data\packages\arduino\hardware\renesas_uno\1.6.0'
if (-not (Test-Path -LiteralPath $cli)) { throw "Arduino CLI fehlt: $cli" }
if (-not (Test-Path -LiteralPath $core)) { throw "Nano-R4-Boardpaket 1.6.0 fehlt: $core" }
if (-not (Test-Path -LiteralPath $SensorLibraryRoot)) { throw "Sensorbibliotheken fehlen: $SensorLibraryRoot" }

# Der Windows-ARM-Compiler 7.2 kann seine eigenen C++-Header unter sehr langen
# Installationspfaden nicht zuverlaessig oeffnen. Die Abbildung ist temporaer.
$drive = "$DriveLetter`:"
if (Test-Path "$drive\") { throw "Laufwerk $drive ist bereits belegt." }
& subst $drive $toolRoot
if ($LASTEXITCODE -ne 0) { throw "Temporäres Laufwerk $drive konnte nicht erstellt werden." }
try {
    $mappedRoot = "$drive\"
    $userLibraries = $SensorLibraryRoot.Replace('\', '/')
    $configuration = "directories:`n  data: '$drive/data'`n  downloads: '$drive/downloads'`n  user: '$userLibraries'`n"
    Set-Content -LiteralPath (Join-Path $toolRoot 'arduino-cli.yaml') -Value $configuration

    $buildFolder = if ($BufferedPanTrace) { 'build-buffered-trace' } elseif ($UsbControl) { 'build-usb' } elseif ($MotionTraceUsb) { 'build-trace' } else { 'build' }
    $buildOptions = if ($BufferedPanTrace) { @('--build-property','compiler.cpp.extra_flags=-DPANODUINO_MOTION_TRACE -DPANODUINO_BUFFERED_PAN_TRACE') } elseif ($UsbControl) { @('--build-property','compiler.cpp.extra_flags=-DPANODUINO_USB_CONTROL') } elseif ($MotionTraceUsb) { @('--build-property','compiler.cpp.extra_flags=-DPANODUINO_MOTION_TRACE') } else { @() }
    $output = & (Join-Path $mappedRoot 'arduino-cli.exe') --config-dir $mappedRoot compile @buildOptions `
        --fqbn 'arduino:renesas_uno:nanor4' --build-path (Join-Path $mappedRoot $buildFolder) `
        --libraries $SensorLibraryRoot (Join-Path $PSScriptRoot '..\PanoDuinoControl') 2>&1
    $output | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0) { throw 'Nano-R4-Build fehlgeschlagen.' }

    $summary = $output -join "`n"
    $flash = [regex]::Match($summary, 'Sketch uses (\d+) bytes .*Maximum is (\d+) bytes')
    $ram = [regex]::Match($summary, 'Global variables use (\d+) bytes .*Maximum is (\d+) bytes')
    if (-not $flash.Success -or -not $ram.Success) { throw 'R4-Speicherbericht nicht erkannt.' }
    $flashUsed = [int]$flash.Groups[1].Value
    $flashMax = [int]$flash.Groups[2].Value
    $ramUsed = [int]$ram.Groups[1].Value
    $ramMax = [int]$ram.Groups[2].Value
    if ($flashUsed -gt [int]($flashMax * 0.80)) { throw 'R4-Flashreserve unter 20 Prozent.' }
    if ($ramUsed -gt [int]($ramMax * 0.70)) { throw 'R4-statische SRAM-Reserve unter 30 Prozent.' }
    & (Join-Path $PSScriptRoot 'Test-R4BootStack.ps1') -BuildDirectory (Join-Path $toolRoot $buildFolder) -ToolDirectory $toolRoot
    Write-Output "R4 reserve: $($flashMax - $flashUsed) bytes flash; $($ramMax - $ramUsed) bytes before stack/heap."
}
finally {
    & subst $drive /D | Out-Null
}
