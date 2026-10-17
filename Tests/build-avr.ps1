param(
    [ValidateSet('Nano', 'Mega')][string]$Board = 'Nano',
    [string]$ArduinoRoot = (Join-Path $env:LOCALAPPDATA 'Arduino15\packages\arduino\hardware\avr\1.8.8'),
    [string]$CompilerRoot = (Join-Path $env:LOCALAPPDATA 'Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin'),
    [string]$SensorLibrary = (Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'Arduino\libraries\AMS_AS5048B-master'),
    [string]$OutputRoot = (Join-Path $PSScriptRoot '.avr-build')
)

$ErrorActionPreference = 'Stop'
$OutputRoot = Join-Path $OutputRoot $Board
if ($Board -eq 'Nano') {
    $mcu = 'atmega328p'
    $variant = 'eightanaloginputs'
    $boardDefine = '-DARDUINO_AVR_NANO'
    $maxProgramBytes = 30720
    $maxDataBytes = 2048
} else {
    $mcu = 'atmega2560'
    $variant = 'mega'
    $boardDefine = '-DARDUINO_AVR_MEGA2560'
    $maxProgramBytes = 253952
    $maxDataBytes = 8192
}
$sourceRoot = Join-Path $PSScriptRoot '..\PanoDuinoControl'
$coreRoot = Join-Path $ArduinoRoot 'cores\arduino'
$wireRoot = Join-Path $ArduinoRoot 'libraries\Wire\src'
$variantRoot = Join-Path $ArduinoRoot "variants\$variant"
$gxx = Join-Path $CompilerRoot 'avr-g++.exe'
$gcc = Join-Path $CompilerRoot 'avr-gcc.exe'
$size = Join-Path $CompilerRoot 'avr-size.exe'

foreach ($path in @($sourceRoot, $coreRoot, $wireRoot, $variantRoot, $SensorLibrary, $gxx, $gcc, $size)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Build-Abhängigkeit fehlt: $path" }
}
New-Item -ItemType Directory -Path $OutputRoot -Force | Out-Null

$includes = @(
    "-I$coreRoot", "-I$variantRoot", "-I$wireRoot",
    "-I$(Join-Path $wireRoot 'utility')", "-I$SensorLibrary", "-I$sourceRoot"
)
$common = @("-mmcu=$mcu", '-DF_CPU=16000000L', '-DARDUINO=10819',
    $boardDefine, '-DARDUINO_ARCH_AVR', '-Os', '-flto',
    '-ffunction-sections', '-fdata-sections') + $includes
$cppFlags = @('-std=gnu++11', '-fpermissive', '-fno-exceptions',
    '-fno-threadsafe-statics', '-Wno-error=narrowing')

$sources = @()
$sources += Get-ChildItem -LiteralPath $coreRoot -File | Where-Object { $_.Extension -in @('.c', '.cpp', '.S') }
$sources += Get-ChildItem -LiteralPath $wireRoot -Recurse -File | Where-Object { $_.Extension -in @('.c', '.cpp') }
$sources += Get-ChildItem -LiteralPath $SensorLibrary -File | Where-Object { $_.Extension -in @('.c', '.cpp') }
$sources += Get-ChildItem -LiteralPath $sourceRoot -File | Where-Object { $_.Extension -in @('.ino', '.cpp') }

$objects = @()
for ($index = 0; $index -lt $sources.Count; $index++) {
    $source = $sources[$index]
    $object = Join-Path $OutputRoot ("source-$index.o")
    $objects += $object
    if ($source.Extension -eq '.c') {
        & $gcc @common '-std=gnu11' '-c' $source.FullName '-o' $object
    } elseif ($source.Extension -eq '.S') {
        & $gcc @common '-x' 'assembler-with-cpp' '-c' $source.FullName '-o' $object
    } elseif ($source.Extension -eq '.ino') {
        & $gxx @common @cppFlags '-x' 'c++' '-c' $source.FullName '-o' $object
    } else {
        & $gxx @common @cppFlags '-c' $source.FullName '-o' $object
    }
    if ($LASTEXITCODE -ne 0) { throw "Kompilierung fehlgeschlagen: $($source.FullName)" }
}

$elf = Join-Path $OutputRoot 'PanoDuinoControl.elf'
& $gxx "-mmcu=$mcu" '-Os' '-flto' '-fuse-linker-plugin' '-Wl,--gc-sections' '-o' $elf @objects '-lm'
if ($LASTEXITCODE -ne 0) { throw 'AVR-Link fehlgeschlagen' }
$sizeOutput = & $size '-C' "--mcu=$mcu" $elf
if ($LASTEXITCODE -ne 0) { throw 'AVR-Größenmessung fehlgeschlagen' }
$sizeOutput
$sizeText = $sizeOutput -join "`n"
$programBytes = [int]([regex]::Match($sizeText, 'Program:\s+(\d+)').Groups[1].Value)
$dataBytes = [int]([regex]::Match($sizeText, 'Data:\s+(\d+)').Groups[1].Value)
"$Board usable flash: $programBytes / $maxProgramBytes bytes; SRAM: $dataBytes / $maxDataBytes bytes"
if ($programBytes -gt $maxProgramBytes) { throw "$Board-Firmware überschreitet den für Sketche nutzbaren Flash." }
if ($dataBytes -gt $maxDataBytes) { throw "$Board-Firmware überschreitet den SRAM." }
if ($Board -eq 'Nano' -and ($maxProgramBytes - $programBytes) -lt 1024) {
    throw 'Nano-Firmware unterschreitet die Flashreserve von 1.024 Byte.'
}
if ($Board -eq 'Nano' -and ($maxDataBytes - $dataBytes) -lt 512) {
    throw 'Nano-Firmware unterschreitet die statische SRAM-Reserve von 512 Byte.'
}
if ($Board -eq 'Mega' -and $programBytes -gt [int]($maxProgramBytes * 0.75)) {
    throw 'Mega-Firmware unterschreitet die vereinbarte Flashreserve von 25 Prozent.'
}
if ($Board -eq 'Mega' -and $dataBytes -gt [int]($maxDataBytes * 0.70)) {
    throw 'Mega-Firmware unterschreitet die statische SRAM-Reserve von 30 Prozent.'
}
