param(
    [string]$BuildDirectory=(Join-Path $PSScriptRoot '.r4-tools\build'),
    [string]$ToolDirectory=(Join-Path $PSScriptRoot '.r4-tools')
)
# Regression guard for global initialization before arduino_main disables the
# R4's 1024-byte stack monitor. This local-frame check is not a full call-tree
# or runtime stack measurement; reserve room for nested constructor calls.
$ErrorActionPreference='Stop'
$elf=Join-Path $BuildDirectory 'PanoDuinoControl.ino.elf'
$bin=Join-Path $ToolDirectory 'data\packages\arduino\tools\arm-none-eabi-gcc\7-2017q4\bin'
$symbols=& (Join-Path $bin 'arm-none-eabi-nm.exe') -S $elf
if($LASTEXITCODE -ne 0){throw 'R4-Symbole nicht lesbar.'}
$symbol=@($symbols | Where-Object {$_ -match '\b_ZN16MotionControllerC1Ev$'})
if($symbol.Count -ne 1 -or $symbol[0] -notmatch '^([0-9a-f]+)\s+([0-9a-f]+)\s+\w\s+'){
    throw 'Globaler MotionController-Konstruktor nicht eindeutig erkannt.'
}
$start=[Convert]::ToUInt32($Matches[1],16);$end=$start+[Convert]::ToUInt32($Matches[2],16)
$instructions=& (Join-Path $bin 'arm-none-eabi-objdump.exe') -d "--start-address=$start" "--stop-address=$end" $elf
if($LASTEXITCODE -ne 0){throw 'R4-Konstruktor nicht disassemblierbar.'}
$reserved=0
foreach($line in $instructions){
    if($line -match '\bsub(?:\.w)?\s+sp,\s*(?:sp,\s*)?#(\d+)'){$reserved+=[int]$Matches[1]}
    elseif($line -match '\bpush\s+\{([^}]+)\}'){$reserved+=4*@($Matches[1].Split(',')).Count}
    elseif($line -match '\bvpush\s+\{d(\d+)(?:-d(\d+))?\}'){
        $last=if($Matches[2]){[int]$Matches[2]}else{[int]$Matches[1]}
        $reserved+=8*($last-[int]$Matches[1]+1)
    }
}
if($reserved -le 0){throw 'Stack-Prolog nicht erkannt; keine vermeintlich erfolgreiche Pruefung.'}
if($reserved -gt 384){throw "R4-Startstack: Konstruktor belegt $reserved Byte; Budget 384 Byte mit Reserve fuer weitere Aufrufe."}
Write-Output "R4-Startstack-Regression: Konstruktor lokal $reserved Byte, Budget 384 Byte; reale Startpruefung bleibt erforderlich."
