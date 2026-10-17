param(
    [string]$Port = 'COM3',
    [string]$ToolDirectory = (Join-Path $PSScriptRoot '.r4-tools'),
    [char]$DriveLetter = 'R',
    [switch]$UsbControl,
    [switch]$MotionTraceUsb,
    [switch]$BufferedPanTrace
)
$ErrorActionPreference = 'Stop'
if ($UsbControl -and ($MotionTraceUsb -or $BufferedPanTrace)) { throw 'USB-Steuerung und Bluetooth-Messspur sind getrennte Builds.' }
$toolRoot = (Resolve-Path -LiteralPath $ToolDirectory).Path
$drive = "$DriveLetter`:"
if (Test-Path "$drive\") { throw "Laufwerk $drive ist bereits belegt." }
& subst $drive $toolRoot
if ($LASTEXITCODE -ne 0) { throw 'Toollaufwerk konnte nicht erstellt werden.' }
try {
    $mappedRoot = "$drive\"
    $cli = Join-Path $mappedRoot 'arduino-cli.exe'
    $boards = & $cli --config-dir $mappedRoot board list --format json | ConvertFrom-Json
    if ($LASTEXITCODE -ne 0) { throw 'Geraeteerkennung fehlgeschlagen.' }
    $device = $boards.detected_ports | Where-Object {
        $_.port.address -eq $Port -and $_.port.properties.vid -eq '0x2341' -and
        $_.port.properties.pid -eq '0x0074' -and $_.matching_boards.fqbn -contains 'arduino:renesas_uno:nanor4'
    }
    if (!$device) { throw "Kein eindeutig erkannter Nano R4 auf $Port; keine Uebertragung." }
    $folder = if ($BufferedPanTrace) { 'build-buffered-trace' } elseif ($UsbControl) { 'build-usb' } elseif ($MotionTraceUsb) { 'build-trace' } else { 'build' }
    $inputDirectory = Join-Path $mappedRoot $folder
    if (!(Test-Path -LiteralPath (Join-Path $inputDirectory 'PanoDuinoControl.ino.bin'))) { throw 'Passender Build fehlt.' }
    & (Join-Path $PSScriptRoot 'Test-R4BootStack.ps1') -BuildDirectory (Join-Path $toolRoot $folder) -ToolDirectory $toolRoot
    Write-Output "Uebertrage $folder auf Nano R4/$Port."
    & $cli --config-dir $mappedRoot upload --fqbn 'arduino:renesas_uno:nanor4' --port $Port --input-dir $inputDirectory (Join-Path $PSScriptRoot '..\PanoDuinoControl')
    if ($LASTEXITCODE -ne 0) { throw 'Nano-R4-Uebertragung fehlgeschlagen.' }
}
finally { & subst $drive /D | Out-Null }
