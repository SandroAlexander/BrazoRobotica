<#
.SYNOPSIS
    Compiles and flashes the BrazoRobotica firmware to the ESP32-CAM using arduino-cli.

.DESCRIPTION
    Automates the build, dependency check, port detection, and upload process for the
    ESP32-CAM robotic arm controller firmware.

.PARAMETER Port
    The serial COM port (e.g. COM3). If omitted, the script lists available ports or auto-detects.

.PARAMETER FQBN
    The Fully Qualified Board Name. Default is esp32:esp32:esp32cam.

.PARAMETER CompileOnly
    Only compile the sketch without uploading.

.PARAMETER UploadOnly
    Only upload the previously compiled binary without compiling.

.PARAMETER CompileAndUpload
    Compile the sketch and immediately upload it.

.PARAMETER Monitor
    Open the serial monitor after uploading (115200 baud).

.EXAMPLE
    .\scripts\build_and_flash.ps1 -CompileOnly
    .\scripts\build_and_flash.ps1 -Port COM3 -CompileAndUpload
    .\scripts\build_and_flash.ps1 -Port COM3 -CompileAndUpload -Monitor
#>

[CmdletBinding()]
param(
    [string]$Port,
    [string]$FQBN = "esp32:esp32:esp32cam",
    [switch]$CompileOnly,
    [switch]$UploadOnly,
    [switch]$CompileAndUpload,
    [switch]$Monitor,
    [switch]$MonitorOnly,
    [int]$BaudRate = 115200
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
$SketchDir = Join-Path $ProjectRoot "arduino\BrazoRobotica"
$SketchFile = Join-Path $SketchDir "BrazoRobotica.ino"

Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "      BrazoRobotica - Build & Flash Tool          " -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "Project Root : $ProjectRoot"
Write-Host "Sketch Path  : $SketchFile"
Write-Host "Target FQBN  : $FQBN"

# 1. Verify arduino-cli installation (check PATH, workspace tools, and user directories)
$cliCmd = Get-Command "arduino-cli" -ErrorAction SilentlyContinue

# Check bundled tools directory inside the project workspace first
$workspaceCli = Join-Path $ProjectRoot "tools\arduino-cli\arduino-cli.exe"
if (-not $cliCmd -and (Test-Path $workspaceCli)) {
    $cliDir = Split-Path -Parent $workspaceCli
    $env:Path = "$cliDir;$env:Path"
    $cliCmd = Get-Command "arduino-cli" -ErrorAction SilentlyContinue
}

# If still not found, refresh PATH from registry in case it was updated in another process
if (-not $cliCmd) {
    try {
        $userPath = [Environment]::GetEnvironmentVariable("Path", "User")
        $sysPath  = [Environment]::GetEnvironmentVariable("Path", "Machine")
        $env:Path = "$userPath;$sysPath"
        $cliCmd = Get-Command "arduino-cli" -ErrorAction SilentlyContinue
    } catch { }
}

if (-not $cliCmd) {
    Write-Host "[ERROR] 'arduino-cli' was not found in PATH or workspace tools." -ForegroundColor Red
    Write-Host "Please install arduino-cli and ensure it is available in your PATH." -ForegroundColor Yellow
    Write-Host "Download from: https://arduino.github.io/arduino-cli/latest/installation/" -ForegroundColor Yellow
    exit 1
}

# 2. Check sketch existence
if (-not (Test-Path $SketchFile)) {
    Write-Host "[ERROR] Sketch file not found at: $SketchFile" -ForegroundColor Red
    exit 1
}

# 3. Handle Port detection if upload is requested
$NeedsUpload = $UploadOnly -or $CompileAndUpload -or (-not $CompileOnly)
if ($NeedsUpload -and [string]::IsNullOrWhiteSpace($Port)) {
    Write-Host "[INFO] Detecting available serial ports..." -ForegroundColor Gray
    try {
        $portsJson = arduino-cli board list --format json | ConvertFrom-Json
        $detectedPorts = @()
        if ($portsJson.detected_ports) {
            $detectedPorts = $portsJson.detected_ports
        }

        if ($detectedPorts.Count -eq 1) {
            $Port = $detectedPorts[0].port.address
            Write-Host "[INFO] Auto-detected COM Port: $Port" -ForegroundColor Green
        } elseif ($detectedPorts.Count -gt 1) {
            Write-Host "[WARN] Multiple serial ports detected:" -ForegroundColor Yellow
            foreach ($p in $detectedPorts) {
                Write-Host "  - Port: $($p.port.address) ($($p.port.label))"
            }
            Write-Host "Please specify a port using -Port <COM_PORT>" -ForegroundColor Red
            exit 1
        } else {
            Write-Host "[WARN] No connected Arduino/ESP32 boards auto-detected by arduino-cli." -ForegroundColor Yellow
            # Fallback to WMI serial ports list
            $wmiPorts = [System.IO.Ports.SerialPort]::GetPortNames()
            if ($wmiPorts.Length -eq 1) {
                $Port = $wmiPorts[0]
                Write-Host "[INFO] Found single system COM Port: $Port" -ForegroundColor Green
            } elseif ($wmiPorts.Length -gt 1) {
                Write-Host "Available system COM Ports: $($wmiPorts -join ', ')" -ForegroundColor Yellow
                Write-Host "Please specify one using -Port <COM_PORT>" -ForegroundColor Red
                exit 1
            } else {
                Write-Host "[ERROR] No serial ports found. Connect your ESP32-CAM via USB/programmer." -ForegroundColor Red
                exit 1
            }
        }
    } catch {
        Write-Host "[WARN] Failed to query board list. Please supply -Port <COM_PORT> explicitly." -ForegroundColor Yellow
        exit 1
    }
}

# 4. Handle MonitorOnly directly
if ($MonitorOnly) {
    if ([string]::IsNullOrWhiteSpace($Port)) {
        Write-Host "[ERROR] Please specify a port using -Port <COM_PORT> to monitor." -ForegroundColor Red
        exit 1
    }
    Write-Host "`n[INFO] Launching Serial Monitor on $Port @ $BaudRate baud (Ctrl+C to exit)..." -ForegroundColor Cyan
    & arduino-cli monitor -p $Port -c baudrate=$BaudRate
    exit 0
}

# 5. Compilation
$ShouldCompile = $CompileOnly -or $CompileAndUpload -or (-not $UploadOnly)
if ($ShouldCompile) {
    Write-Host "`n[1/2] Compiling sketch with arduino-cli..." -ForegroundColor Cyan
    $compileArgs = @("compile", "--fqbn", $FQBN, $SketchDir)
    
    $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    & arduino-cli @compileArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] Compilation failed!" -ForegroundColor Red
        exit $LASTEXITCODE
    }
    $stopwatch.Stop()
    Write-Host "[SUCCESS] Compilation completed successfully in $($stopwatch.Elapsed.TotalSeconds.ToString('F1'))s" -ForegroundColor Green
}

# 5. Uploading
if ($NeedsUpload) {
    Write-Host "`n[2/2] Uploading binary to $Port at $FQBN..." -ForegroundColor Cyan
    $uploadArgs = @("upload", "-p", $Port, "--fqbn", $FQBN, $SketchDir)
    
    & arduino-cli @uploadArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] Upload failed! Make sure GPIO 0 is grounded (if required by your programmer) and press RESET." -ForegroundColor Red
        exit $LASTEXITCODE
    }
    Write-Host "[SUCCESS] Upload complete!" -ForegroundColor Green
}

# 6. Serial Monitor
if ($Monitor -and -not [string]::IsNullOrWhiteSpace($Port)) {
    Write-Host "`n[INFO] Launching Serial Monitor on $Port @ $BaudRate baud (Ctrl+C to exit)..." -ForegroundColor Cyan
    & arduino-cli monitor -p $Port -c baudrate=$BaudRate
}
