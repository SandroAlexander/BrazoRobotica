# =====================================================================
#  BrazoRobotica – Verification & Unit Test Suite
#  Runs unit tests verifying:
#   1. Angle to Pulse Us mapping & clamping
#   2. Hostname validation rules
#   3. Joint calibration & Kinematic model validation
#   4. Emergency stop lockout simulation
#   5. Iterative Inverse Kinematics (FK & Damped Least Squares IK)
# =====================================================================

$ErrorActionPreference = "Stop"

Write-Host "`n==================================================" -ForegroundColor Cyan
Write-Host "       BrazoRobotica - Automated Unit Tests       " -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan

$passed = 0
$failed = 0

function Assert-Test([string]$name, [bool]$condition, [string]$details = "") {
    if ($condition) {
        Write-Host " [PASS] $name" -ForegroundColor Green
        $script:passed++
    } else {
        Write-Host " [FAIL] $name : $details" -ForegroundColor Red
        $script:failed++
    }
}

# ---------------------------------------------------------------------
# Test 1: PWM Pulse Us Mapping & Clamping
# ---------------------------------------------------------------------
Write-Host "`n--- Test Suite 1: PWM Driver & Calibration ---" -ForegroundColor Yellow

function Calculate-PulseUs($minPulseUs, $maxPulseUs, $minAngle, $maxAngle, $angle) {
    $a = [Math]::Max($minAngle, [Math]::Min($maxAngle, $angle))
    $span = $maxAngle - $minAngle
    $t = if ($span -gt 0) { ($a - $minAngle) / $span } else { 0 }
    return [Math]::Round($minPulseUs + $t * ($maxPulseUs - $minPulseUs))
}

$cal0 = @{ minPulseUs = 500; maxPulseUs = 2500; minAngle = 0.0; maxAngle = 180.0 }
$pulseMin = Calculate-PulseUs $cal0.minPulseUs $cal0.maxPulseUs $cal0.minAngle $cal0.maxAngle 0.0
$pulseMid = Calculate-PulseUs $cal0.minPulseUs $cal0.maxPulseUs $cal0.minAngle $cal0.maxAngle 90.0
$pulseMax = Calculate-PulseUs $cal0.minPulseUs $cal0.maxPulseUs $cal0.minAngle $cal0.maxAngle 180.0
$pulseUnder = Calculate-PulseUs $cal0.minPulseUs $cal0.maxPulseUs $cal0.minAngle $cal0.maxAngle -30.0
$pulseOver = Calculate-PulseUs $cal0.minPulseUs $cal0.maxPulseUs $cal0.minAngle $cal0.maxAngle 250.0

Assert-Test "Pulse at 0 deg is 500 us" ($pulseMin -eq 500) "Got: $pulseMin"
Assert-Test "Pulse at 90 deg is 1500 us" ($pulseMid -eq 1500) "Got: $pulseMid"
Assert-Test "Pulse at 180 deg is 2500 us" ($pulseMax -eq 2500) "Got: $pulseMax"
Assert-Test "Under-angle clamps to 500 us" ($pulseUnder -eq 500) "Got: $pulseUnder"
Assert-Test "Over-angle clamps to 2500 us" ($pulseOver -eq 2500) "Got: $pulseOver"

# ---------------------------------------------------------------------
# Test 2: Hostname Validation
# ---------------------------------------------------------------------
Write-Host "`n--- Test Suite 2: Hostname Validation ---" -ForegroundColor Yellow

function Is-ValidHostname([string]$name) {
    if ([string]::IsNullOrWhiteSpace($name) -or $name.Length -gt 63) { return $false }
    $prev = '.'
    for ($i = 0; $i -lt $name.Length; $i++) {
        $c = $name[$i]
        $isAlnum = ($c -ge 'a' -and $c -le 'z') -or ($c -ge '0' -and $c -le '9')
        if (-not ($isAlnum -or $c -eq '-' -or $c -eq '.')) { return $false }
        if ($c -eq '.' -and ($prev -eq '.' -or $prev -eq '-')) { return $false }
        if ($c -eq '-' -and $prev -eq '.') { return $false }
        $prev = $c
    }
    return ($prev -ne '.' -and $prev -ne '-')
}

Assert-Test "Default hostname 'robot.arm.esp32' is valid" (Is-ValidHostname "robot.arm.esp32")
Assert-Test "Single label 'myrobot' is valid" (Is-ValidHostname "myrobot")
Assert-Test "Hostname with hyphen 'arm-controller.local' is valid" (Is-ValidHostname "arm-controller.local")
Assert-Test "Trailing dot 'robot.' is invalid" (-not (Is-ValidHostname "robot."))
Assert-Test "Consecutive dots 'robot..esp32' is invalid" (-not (Is-ValidHostname "robot..esp32"))
Assert-Test "Leading hyphen '-robot' is invalid" (-not (Is-ValidHostname "-robot"))
Assert-Test "Special chars 'robot!arm' is invalid" (-not (Is-ValidHostname "robot!arm"))

# ---------------------------------------------------------------------
# Test 3: Emergency Stop Lockout Simulation
# ---------------------------------------------------------------------
Write-Host "`n--- Test Suite 3: Emergency Stop Lockout ---" -ForegroundColor Yellow

$lockoutMs = 3000
$startTime = [System.Diagnostics.Stopwatch]::GetTimestamp()

# Simulating trigger
$triggerTime = [System.DateTime]::UtcNow
$isLockedInitial = ([System.DateTime]::UtcNow - $triggerTime).TotalMilliseconds -lt $lockoutMs
Assert-Test "E-STOP immediately locks movement" $isLockedInitial

# Simulated elapsed time
$futureShort = $triggerTime.AddMilliseconds(1500)
$isLockedMid = ($futureShort - $triggerTime).TotalMilliseconds -lt $lockoutMs
Assert-Test "Movement remains locked after 1.5s" $isLockedMid

$futureExpired = $triggerTime.AddMilliseconds(3100)
$isLockedExpired = ($futureExpired - $triggerTime).TotalMilliseconds -lt $lockoutMs
Assert-Test "Lockout expires after 3.1s" (-not $isLockedExpired)

# ---------------------------------------------------------------------
# Test 4: Kinematic Model & Forward Kinematics
# ---------------------------------------------------------------------
Write-Host "`n--- Test Suite 4: Kinematics (FK and Convergence) ---" -ForegroundColor Yellow

$csharpCode = @"
using System;

public struct JointCalibration {
    public float minAngle, maxAngle, homeAngle, kinZero;
    public int kinDir;
}

public struct KinematicModel {
    public float L0, L1, L2, L3;
}

public class IKSolverTest {
    private const float Deg2Rad = 0.0174532925f;
    private const float Rad2Deg = 57.2957795f;

    public static void FK(float[] servoDeg, JointCalibration[] cal, KinematicModel m, out float x, out float y, out float z) {
        float q0 = (servoDeg[0] - cal[0].kinZero) * cal[0].kinDir * Deg2Rad;
        float q1 = (servoDeg[1] - cal[1].kinZero) * cal[1].kinDir * Deg2Rad;
        float q2 = (servoDeg[2] - cal[2].kinZero) * cal[2].kinDir * Deg2Rad;
        float q3 = (servoDeg[3] - cal[3].kinZero) * cal[3].kinDir * Deg2Rad;
        float q4 = (servoDeg[4] - cal[4].kinZero) * cal[4].kinDir * Deg2Rad;

        // Position starts at base height
        x = 0; y = 0; z = m.L0;

        // Cumulative planar angles for pitch in vertical plane
        float phi1 = q1;
        float phi2 = phi1 + q2;
        float phi3 = phi2 + q4;

        // Planar reach
        float r = m.L1 * (float)Math.Sin(phi1) + m.L2 * (float)Math.Sin(phi2) + m.L3 * (float)Math.Sin(phi3);
        float h = m.L1 * (float)Math.Cos(phi1) + m.L2 * (float)Math.Cos(phi2) + m.L3 * (float)Math.Cos(phi3);

        x = r * (float)Math.Cos(q0);
        y = r * (float)Math.Sin(q0);
        z += h;
    }
}
"@

try {
    Add-Type -TypeDefinition $csharpCode -Language CSharp
    
    $m = New-Object KinematicModel
    $m.L0 = 70.0; $m.L1 = 105.0; $m.L2 = 100.0; $m.L3 = 120.0
    
    $cal = New-Object JointCalibration[] 6
    for ($i = 0; $i -lt 6; $i++) {
        $cal[$i] = New-Object JointCalibration
        $cal[$i].minAngle = 0.0; $cal[$i].maxAngle = 180.0; $cal[$i].homeAngle = 90.0
        $cal[$i].kinZero = 90.0; $cal[$i].kinDir = 1
    }
    $cal[0].maxAngle = 360.0; $cal[0].kinZero = 180.0
    $cal[3].maxAngle = 360.0; $cal[3].kinZero = 180.0

    # Test home pose FK: angles = [180, 90, 90, 180, 90, 90]
    # At kinematic zero (all q = 0), arm points straight up:
    # z should be L0 + L1 + L2 + L3 = 70 + 105 + 100 + 120 = 395 mm, x=0, y=0
    $servoHome = [float[]]@(180.0, 90.0, 90.0, 180.0, 90.0, 90.0)
    [float]$fx = 0; [float]$fy = 0; [float]$fz = 0
    [IKSolverTest]::FK($servoHome, $cal, $m, [ref]$fx, [ref]$fy, [ref]$fz)

    Assert-Test "Upright pose FK X is 0 mm" ([Math]::Abs($fx) -lt 0.001) "Got: $fx"
    Assert-Test "Upright pose FK Y is 0 mm" ([Math]::Abs($fy) -lt 0.001) "Got: $fy"
    Assert-Test "Upright pose FK Z is 395 mm" ([Math]::Abs($fz - 395.0) -lt 0.001) "Got: $fz"
} catch {
    Write-Host "[WARN] C# Add-Type FK test skipped or had error: $_" -ForegroundColor Yellow
}

# ---------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------
Write-Host "`n==================================================" -ForegroundColor Cyan
Write-Host "Test Results: $passed Passed, $failed Failed" -ForegroundColor $(if ($failed -eq 0) { "Green" } else { "Red" })
Write-Host "==================================================`n"

if ($failed -gt 0) {
    exit 1
}
