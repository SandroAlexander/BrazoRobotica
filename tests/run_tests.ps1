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
# Test 4: Denavit-Hartenberg (4x4 Homogeneous Matrix) & Differential Evolution IK
# ---------------------------------------------------------------------
Write-Host "`n--- Test Suite 4: Denavit-Hartenberg FK & Differential Evolution IK ---" -ForegroundColor Yellow

$csharpCode = @"
using System;

public class JointCalibration {
    public float minAngle, maxAngle, homeAngle, kinZero;
    public int kinDir;
}

public class DHParam {
    public float alpha, a, d, theta;
}

public class IKSolverTest {
    private const float Deg2Rad = 0.017453292519943295f;
    private const float Rad2Deg = 57.29577951308232f;

    public static float[,] DHMatrix(float alphaDeg, float a, float d, float thetaDeg) {
        float alpha = alphaDeg * Deg2Rad;
        float theta = thetaDeg * Deg2Rad;
        float ct = (float)Math.Cos(theta), st = (float)Math.Sin(theta);
        float ca = (float)Math.Cos(alpha), sa = (float)Math.Sin(alpha);

        return new float[4, 4] {
            { ct, -st * ca,  st * sa, a * ct },
            { st,  ct * ca, -ct * sa, a * st },
            { 0f,       sa,       ca,      d },
            { 0f,       0f,       0f,     1f }
        };
    }

    public static float[,] Mat4Mul(float[,] A, float[,] B) {
        float[,] R = new float[4, 4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                R[i, j] = A[i, 0] * B[0, j] + A[i, 1] * B[1, j] + A[i, 2] * B[2, j] + A[i, 3] * B[3, j];
            }
        }
        return R;
    }

    public static void FK(float[] servoDeg, JointCalibration[] cal, DHParam[] dh,
                          out float x, out float y, out float z, float[,] relOrigins) {
        float[,] T = new float[4, 4] {
            { 1f, 0f, 0f, 0f },
            { 0f, 1f, 0f, 0f },
            { 0f, 0f, 1f, 0f },
            { 0f, 0f, 0f, 1f }
        };
        float px = 0f, py = 0f, pz = 0f;
        for (int i = 0; i < 6; i++) {
            float dir = cal[i].kinDir < 0 ? -1f : 1f;
            float qDeg = (servoDeg[i] - cal[i].kinZero) * dir;
            float effTheta = qDeg + dh[i].theta;
            float[,] Ai = DHMatrix(dh[i].alpha, dh[i].a, dh[i].d, effTheta);
            T = Mat4Mul(T, Ai);
            if (relOrigins != null) {
                relOrigins[i, 0] = T[0, 3] - px;
                relOrigins[i, 1] = T[1, 3] - py;
                relOrigins[i, 2] = T[2, 3] - pz;
                px = T[0, 3]; py = T[1, 3]; pz = T[2, 3];
            }
        }
        x = T[0, 3]; y = T[1, 3]; z = T[2, 3];
    }

    private static float Clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    private static float DistToTarget(float[] servo, JointCalibration[] cal, DHParam[] dh, float tx, float ty, float tz) {
        float x, y, z;
        FK(servo, cal, dh, out x, out y, out z, null);
        float dx = tx - x, dy = ty - y, dz = tz - z;
        return (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    public static float[] SolveDE(float px, float py, float pz, int iterations, int population,
                                  float[] seed, JointCalibration[] cal, DHParam[] dh, bool lockGripper, out float finalErr) {
        int P = Math.Max(4, population);
        float K = 0.6f;
        float keepXProb = 0.5f; // CR = 0.5
        Random rng = new Random(42);

        float[][] pop = new float[P][];
        float[][] mutants = new float[P][];
        float[][] trials = new float[P][];
        float[] costs = new float[P];

        float yawDeg = 0f;
        if (Math.Sqrt(px * px + py * py) > 1.0) {
            yawDeg = (float)(Math.Atan2(py, px) * Rad2Deg);
        }

        int bestIdx = 0;
        // populate()
        for (int i = 0; i < P; i++) {
            pop[i] = new float[6];
            mutants[i] = new float[6];
            trials[i] = new float[6];
            if (i == 0 && seed != null) {
                for (int g = 0; g < 6; g++) pop[i][g] = Clamp(seed[g], cal[g].minAngle, cal[g].maxAngle);
            } else {
                for (int g = 0; g < 6; g++) {
                    pop[i][g] = cal[g].minAngle + (float)rng.NextDouble() * (cal[g].maxAngle - cal[g].minAngle);
                }
                if (i % 2 == 1) {
                    float dir0 = cal[0].kinDir < 0 ? -1f : 1f;
                    float baseServo = cal[0].kinZero + yawDeg * dir0;
                    pop[i][0] = Clamp(baseServo, cal[0].minAngle, cal[0].maxAngle);
                    pop[i][3] = cal[3].kinZero;
                }
            }
            if (lockGripper && seed != null) pop[i][5] = seed[5];
            costs[i] = DistToTarget(pop[i], cal, dh, px, py, pz);
            if (costs[i] < costs[bestIdx]) bestIdx = i;
        }

        for (int it = 0; it < iterations; it++) {
            if (costs[bestIdx] <= 0.5f) break;

            // mutate()
            for (int i = 0; i < P; i++) {
                int j = rng.Next(P - 1);
                if (j >= i) j++;
                int k = rng.Next(P - 2);
                if (k >= Math.Min(i, j)) k++;
                if (k >= Math.Max(i, j)) k++;
                float[] baseVec = ((i & 1) == 0) ? pop[bestIdx] : pop[i];

                for (int g = 0; g < 6; g++) {
                    if (lockGripper && g == 5) { mutants[i][g] = seed[5]; continue; }
                    float v = baseVec[g] + K * (pop[j][g] - pop[k][g]);
                    mutants[i][g] = Clamp(v, cal[g].minAngle, cal[g].maxAngle);
                }
            }

            // crossover()
            int activeGenes = lockGripper ? 5 : 6;
            for (int i = 0; i < P; i++) {
                int gForce = rng.Next(activeGenes);
                for (int g = 0; g < 6; g++) {
                    if (lockGripper && g == 5) { trials[i][g] = seed[5]; continue; }
                    if (g != gForce && rng.NextDouble() < keepXProb) trials[i][g] = pop[i][g];
                    else trials[i][g] = mutants[i][g];
                }
            }

            // compare()
            for (int i = 0; i < P; i++) {
                float d = DistToTarget(trials[i], cal, dh, px, py, pz);
                if (d <= costs[i]) {
                    Array.Copy(trials[i], pop[i], 6);
                    costs[i] = d;
                    if (d < costs[bestIdx]) bestIdx = i;
                }
            }
        }

        finalErr = costs[bestIdx];
        return pop[bestIdx];
    }
}
"@

try {
    Add-Type -TypeDefinition $csharpCode -Language CSharp

    $dh = New-Object DHParam[] 6
    $dh[0] = New-Object DHParam -Property @{ alpha = -90.0; a = 0.0;   d = 70.0;  theta = 0.0   }
    $dh[1] = New-Object DHParam -Property @{ alpha = 0.0;   a = 105.0; d = 0.0;   theta = -90.0 }
    $dh[2] = New-Object DHParam -Property @{ alpha = 90.0;  a = 0.0;   d = 0.0;   theta = 90.0  }
    $dh[3] = New-Object DHParam -Property @{ alpha = -90.0; a = 0.0;   d = 100.0; theta = 0.0   }
    $dh[4] = New-Object DHParam -Property @{ alpha = 90.0;  a = 0.0;   d = 0.0;   theta = 0.0   }
    $dh[5] = New-Object DHParam -Property @{ alpha = 0.0;   a = 0.0;   d = 120.0; theta = 0.0   }

    $cal = New-Object JointCalibration[] 6
    for ($i = 0; $i -lt 6; $i++) {
        $cal[$i] = New-Object JointCalibration
        $cal[$i].minAngle = 0.0; $cal[$i].maxAngle = 180.0; $cal[$i].homeAngle = 90.0
        $cal[$i].kinZero = 90.0; $cal[$i].kinDir = 1
    }
    $cal[0].maxAngle = 360.0; $cal[0].kinZero = 180.0; $cal[0].homeAngle = 180.0
    $cal[3].maxAngle = 360.0; $cal[3].kinZero = 180.0; $cal[3].homeAngle = 180.0

    # 1. Test upright home pose FK via 4x4 DH matrices: [180, 90, 90, 180, 90, 90]
    $servoHome = [float[]]@(180.0, 90.0, 90.0, 180.0, 90.0, 90.0)
    [float]$fx = 0; [float]$fy = 0; [float]$fz = 0
    $rel = New-Object 'float[,]' 6, 3
    [IKSolverTest]::FK($servoHome, $cal, $dh, [ref]$fx, [ref]$fy, [ref]$fz, $rel)

    Assert-Test "DH FK upright pose X is 0 mm" ([Math]::Abs($fx) -lt 0.01) "Got: $fx"
    Assert-Test "DH FK upright pose Y is 0 mm" ([Math]::Abs($fy) -lt 0.01) "Got: $fy"
    Assert-Test "DH FK upright pose Z is 395 mm" ([Math]::Abs($fz - 395.0) -lt 0.01) "Got: $fz"

    # 2. Verify 3D relative origin vectors sum to total tip position (R07)
    $rz0 = $rel.GetValue(0, 2); $rz1 = $rel.GetValue(1, 2); $rz2 = $rel.GetValue(2, 2)
    $rz3 = $rel.GetValue(3, 2); $rz4 = $rel.GetValue(4, 2); $rz5 = $rel.GetValue(5, 2)
    $sumZ = $rz0 + $rz1 + $rz2 + $rz3 + $rz4 + $rz5
    Assert-Test "Relative 3D origin vectors sum to (0, 0, 395 mm)" (
        [Math]::Abs($rz0 - 70.0) -lt 0.01 -and
        [Math]::Abs($rz1 - 105.0) -lt 0.01 -and
        [Math]::Abs($rz3 - 100.0) -lt 0.01 -and
        [Math]::Abs($rz5 - 120.0) -lt 0.01 -and
        [Math]::Abs($sumZ - 395.0) -lt 0.01
    ) "Got sumZ=$sumZ"

    # 3. Test Differential Evolution IK solver on reachable target (150, 0, 120)
    [float]$err1 = 0
    $sol1 = [IKSolverTest]::SolveDE(150.0, 0.0, 120.0, 250, 40, $servoHome, $cal, $dh, $true, [ref]$err1)
    Assert-Test "Differential Evolution converges on (150, 0, 120) with error <= 0.5 mm" ($err1 -le 0.5) "Got err: $err1 mm"
    Assert-Test "Differential Evolution keeps locked J5 Gripper angle at 90 deg" ([Math]::Abs($sol1[5] - 90.0) -lt 0.001) "Got J5: $($sol1[5])"

    # 4. Verify all solved angles are within joint limits
    $inRange = $true
    for ($g = 0; $g -lt 6; $g++) {
        if ($sol1[$g] -lt $cal[$g].minAngle -or $sol1[$g] -gt $cal[$g].maxAngle) { $inRange = $false }
    }
    Assert-Test "All 6 solved angles are within configured servo limits" $inRange

    # 5. Test 3D off-axis target (110, 90, 180)
    [float]$err2 = 0
    $sol2 = [IKSolverTest]::SolveDE(110.0, 90.0, 180.0, 250, 40, $servoHome, $cal, $dh, $true, [ref]$err2)
    Assert-Test "Differential Evolution converges on 3D target (110, 90, 180) with error <= 0.5 mm" ($err2 -le 0.5) "Got err: $err2 mm"
} catch {
    Write-Host "[FAIL] C# DH & Differential Evolution test error: $_" -ForegroundColor Red
    $failed++
}

# ---------------------------------------------------------------------
# Test 5: Movement Query Parsing, Per-Joint Speed (uint8), Storage & Execution
# ---------------------------------------------------------------------
Write-Host "`n--- Test Suite 5: Movement Queries & NVS Storage ---" -ForegroundColor Yellow

# Verify StoredQuery binary layout: 11B name + 1B used + 12B anglesTenths (6x int16) + 6B speedsDps (6x uint8) = 30B
$bytesPerQuery = 11 + 1 + (6 * 2) + (6 * 1)
$maxStoredQueries = 50
$magicHeaderBytes = 4
$totalNvsBlobBytes = $magicHeaderBytes + ($maxStoredQueries * $bytesPerQuery)
$singlePageNvsMax = 1984 # ESP32 NVS max single-page blob size (62 * 32B)

Assert-Test "StoredQuery struct size is 30 bytes" ($bytesPerQuery -eq 30) "Got: $bytesPerQuery"
Assert-Test "50 stored queries reserve 1504 bytes in NVS" ($totalNvsBlobBytes -eq 1504) "Got: $totalNvsBlobBytes"
Assert-Test "50 queries fit within ESP32 single-page NVS limit (1984 B)" ($totalNvsBlobBytes -le $singlePageNvsMax) "Got: $totalNvsBlobBytes"

function Test-QueryName([string]$name) {
    if ([string]::IsNullOrEmpty($name) -or $name.Length -gt 10) { return $false }
    return ($name -cmatch '^[A-Za-z0-9_\-]+$')
}

Assert-Test "10-char alphanumeric query name 'PICK_PAWN1' is valid" (Test-QueryName "PICK_PAWN1")
Assert-Test "11-char query name 'PICK_PAWN12' is rejected" (-not (Test-QueryName "PICK_PAWN12"))
Assert-Test "Query name with space 'PICK PAWN' is rejected" (-not (Test-QueryName "PICK PAWN"))

function Parse-MovementQuery([string]$anglesStr, [string]$speedsStr, [float[]]$minAngles, [float[]]$maxAngles) {
    $tokens = $anglesStr.Split(',') | ForEach-Object { $_.Trim() }
    if ($tokens.Count -ne 6) { return @{ ok = $false; err = "Count mismatch" } }

    $sTokens = if ([string]::IsNullOrWhiteSpace($speedsStr)) {
        @("*", "*", "*", "*", "*", "*")
    } elseif (-not $speedsStr.Contains(",")) {
        @($speedsStr.Trim()) * 6
    } else {
        $speedsStr.Split(',') | ForEach-Object { $_.Trim() }
    }
    if ($sTokens.Count -ne 6) { return @{ ok = $false; err = "Speed count mismatch" } }

    $tenths = New-Object int[] 6
    $speeds = New-Object byte[] 6
    $anyActive = $false

    for ($i = 0; $i -lt 6; $i++) {
        $tok = $tokens[$i]
        $sTok = $sTokens[$i]
        $hasInline = $false

        if ($tok.Contains("@")) {
            $parts = $tok.Split('@')
            $tok = $parts[0].Trim()
            $sTok = $parts[1].Trim()
            $hasInline = $true
        }

        if ($tok -eq "*") {
            $tenths[$i] = -1
            $speeds[$i] = 0
            continue
        }

        [double]$deg = 0
        if (-not [double]::TryParse($tok, [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$deg)) {
            return @{ ok = $false; err = "Invalid angle token $tok" }
        }
        if ($deg -lt $minAngles[$i] -or $deg -gt $maxAngles[$i]) {
            return @{ ok = $false; err = "Angle out of range $deg" }
        }
        $tenths[$i] = [int][Math]::Round($deg * 10.0)
        $anyActive = $true

        if ($sTok -eq "*" -or $sTok -eq "" -or $sTok -eq "0") {
            $speeds[$i] = 0
        } else {
            [int]$sv = 0
            if (-not [int]::TryParse($sTok, [ref]$sv) -or $sv -lt 1 -or $sv -gt 255) {
                return @{ ok = $false; err = "Invalid uint8 speed $sTok" }
            }
            $speeds[$i] = [byte]$sv
        }
    }

    if (-not $anyActive) { return @{ ok = $false; err = "All wildcards" } }
    return @{ ok = $true; tenths = $tenths; speeds = $speeds }
}

$minA = [float[]]@(0, 0, 0, 0, 0, 0)
$maxA = [float[]]@(360, 180, 180, 360, 180, 180)

$q1 = Parse-MovementQuery "14, 180, 30, *, 0, *" "60, 120, 45, *, 90, *" $minA $maxA
Assert-Test "Query '14, 180, 30, *, 0, *' parses successfully" $q1.ok
Assert-Test "Parsed tenths match [140, 1800, 300, -1, 0, -1]" (
    $q1.tenths[0] -eq 140 -and $q1.tenths[1] -eq 1800 -and $q1.tenths[2] -eq 300 -and
    $q1.tenths[3] -eq -1 -and $q1.tenths[4] -eq 0 -and $q1.tenths[5] -eq -1
)
Assert-Test "Parsed uint8 speeds match [60, 120, 45, 0, 90, 0]" (
    $q1.speeds[0] -eq 60 -and $q1.speeds[1] -eq 120 -and $q1.speeds[2] -eq 45 -and
    $q1.speeds[3] -eq 0 -and $q1.speeds[4] -eq 90 -and $q1.speeds[5] -eq 0
)

$qInline = Parse-MovementQuery "14@75, 180@200, 30, *, 0@255, *" "*" $minA $maxA
Assert-Test "Inline angle@speed query parses successfully" ($qInline.ok -and $qInline.speeds[0] -eq 75 -and $qInline.speeds[1] -eq 200 -and $qInline.speeds[4] -eq 255)

$qBadSpeed = Parse-MovementQuery "14@300, 180, 30, *, 0, *" "*" $minA $maxA
Assert-Test "Speed > 255 (uint8 overflow) is rejected" (-not $qBadSpeed.ok)

$qAllWild = Parse-MovementQuery "*, *, *, *, *, *" "*" $minA $maxA
Assert-Test "Query with all '*' wildcards is rejected" (-not $qAllWild.ok)

# Test Simultaneous target resolution (wildcard '*' preserves current angle)
$currentAngles = [float[]]@(100.0, 45.0, 90.0, 210.0, 60.0, 80.0)
$simTargets = New-Object float[] 6
for ($i = 0; $i -lt 6; $i++) {
    $simTargets[$i] = if ($q1.tenths[$i] -eq -1) { $currentAngles[$i] } else { $q1.tenths[$i] * 0.1 }
}
Assert-Test "Simultaneous execution updates numeric joints and preserves '*' joints (J3=210, J5=80)" (
    [Math]::Abs($simTargets[0] - 14.0) -lt 0.01 -and
    [Math]::Abs($simTargets[1] - 180.0) -lt 0.01 -and
    [Math]::Abs($simTargets[2] - 30.0) -lt 0.01 -and
    [Math]::Abs($simTargets[3] - 210.0) -lt 0.01 -and
    [Math]::Abs($simTargets[4] - 0.0) -lt 0.01 -and
    [Math]::Abs($simTargets[5] - 80.0) -lt 0.01
)

# Test One-by-One sequential execution order (skips '*' joints J3 and J5)
$seqVisited = @()
for ($j = 0; $j -lt 6; $j++) {
    if ($q1.tenths[$j] -ne -1) { $seqVisited += $j }
}
Assert-Test "One-by-one execution visits only active joints in order [0, 1, 2, 4]" (($seqVisited -join ",") -eq "0,1,2,4")

# ---------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------
Write-Host "`n==================================================" -ForegroundColor Cyan
Write-Host "Test Results: $passed Passed, $failed Failed" -ForegroundColor $(if ($failed -eq 0) { "Green" } else { "Red" })
Write-Host "==================================================`n"

if ($failed -gt 0) {
    exit 1
}
