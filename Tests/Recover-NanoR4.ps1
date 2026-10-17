param(
    [ValidateRange(5,300)][int]$WaitSeconds=120,
    [string]$ToolDirectory=(Join-Path $PSScriptRoot '.r4-tools')
)
# One bounded recovery attempt for the normal Bluetooth image. Only Arduino
# Nano R4 USB IDs, never another DFU device. Does not erase the bootloader.
$ErrorActionPreference='Stop'
$root=(Resolve-Path -LiteralPath $ToolDirectory).Path
$build=Join-Path $root 'build'
$image=Join-Path $build 'PanoDuinoControl.ino.bin'
$dfu=Join-Path $root 'data\packages\arduino\tools\dfu-util\0.11.0-arduino5\dfu-util.exe'
if(!(Test-Path -LiteralPath $image)){throw 'Normaler R4-Build fehlt.'}
& (Join-Path $PSScriptRoot 'Test-R4BootStack.ps1') -ToolDirectory $root -BuildDirectory $build
$timer=[Diagnostics.Stopwatch]::StartNew()
Write-Output 'Wiederherstellung bereit: warte ausschliesslich auf Nano-R4-USB 2341:0074/0374.'
while($timer.Elapsed.TotalSeconds -lt $WaitSeconds){
    $inventory=@(& $dfu -l 2>&1)
    if($LASTEXITCODE -ne 0){throw 'USB-DFU-Erkennung fehlgeschlagen.'}
    $nanoDevices=@($inventory | Where-Object {$_ -match 'Found (?:Runtime|DFU): \[2341:(?:0074|0374)\]'})
    if($nanoDevices.Count){
        $paths=@($nanoDevices | ForEach-Object {
            if($_ -notmatch 'path="([^"]+)"'){throw 'Nano-USB-Pfad nicht eindeutig.'}
            $Matches[1]
        } | Select-Object -Unique)
        if($paths.Count -ne 1){throw 'Mehrere Nano R4 erkannt; keine automatische Uebertragung.'}
        Write-Output "Nano R4 auf USB-Pfad $($paths[0]) erkannt; uebertrage geprueften normalen Build."
        & $dfu --device '2341:0074,:0374' --path $paths[0] -D $image -a 0 -Q
        if($LASTEXITCODE -ne 0){throw 'Wiederherstellungsupload nicht erfolgreich; keine automatische Wiederholung.'}
        Write-Output 'DFU-Uebertragung abgeschlossen. Firmwarestart separat ueber Bluetooth pruefen.'
        exit 0
    }
    Start-Sleep -Milliseconds 250
}
throw 'Nano R4 nicht ueber USB erreichbar. Kein Upload erfolgt; Zugriff auf RESET erforderlich.'
