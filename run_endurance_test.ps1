# Endurance test: run gcode files sequentially, restart on completion
# Usage: .\run_endurance_test.ps1 [-MaxHours 24] [-Build] [-SpeedFactor 2.0]

param(
    [int]$MaxHours = 48,
    [double]$SpeedFactor = 1.0,
    [switch]$Build
)

$exe = Resolve-Path ".\klipper_host\x64\Release\klipper_host.exe"
$config = "configs\generic-duet3-6hc.cfg"
$logDir = "test_logs"
$summaryFile = "$logDir\endurance_summary.txt"
$lockFile = "$logDir\.lock"

# Gcode files to cycle through (largest first)
$gcodeFiles = @(
    "gcode\CE3E3V2_safety_spring_mount_front_m6_thread_mirror_x1_rev1.gcode"  # 1.95M lines ~10h
    "gcode\CE3E3V2_z_drive_rear_x1_rev4.gcode"                                # 470K lines
    "gcode\upper_(m6_thread)_x4_rev5.gcode"                                   # 442K lines
    "gcode\CE3E3V2_z_drive_front_left_x1_rev4.gcode"                          # 411K lines
    "gcode\CE3E3V2_z_drive_front_right_x1_rev4.gcode"                         # 409K lines
    "gcode\CE3E3V2_corner_drive_shaft_support_mirror_x2_rev4.gcode"           # 341K lines
    "gcode\2_piece_lower_x4_rev4.gcode"                                       # 335K lines
    "gcode\CE3E3V2_xy_cross_rail_mount_x2_rev4.gcode"                         # 296K lines
)

if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }

# Prevent multiple instances
if (Test-Path $lockFile) {
    $lockPid = Get-Content $lockFile -ErrorAction SilentlyContinue
    $lockProc = Get-Process -Id $lockPid -ErrorAction SilentlyContinue
    if ($lockProc) {
        Write-Host "Another instance is running (PID=$lockPid). Exiting." -ForegroundColor Red
        exit 1
    }
}
$PID | Out-File $lockFile -Encoding utf8
try {

# Optional rebuild
if ($Build) {
    Write-Host "Building Release..." -ForegroundColor Cyan
    & "C:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Current\Bin\amd64\MSBuild.exe" `
        klipper_host\klipper_host.vcxproj /p:Configuration=Release /p:Platform=x64 /v:minimal 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "BUILD FAILED" -ForegroundColor Red
        exit 1
    }
}

# Kill stale processes
Get-Process klipper_host -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 2

$startTime = Get-Date
$deadline = $startTime.AddHours($MaxHours)
$iteration = 0
$fileIdx = 0
$totalOk = 0
$totalFail = 0

$header = "=== Endurance test started $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') max=$MaxHours h speed=${SpeedFactor}x ==="
Write-Host $header -ForegroundColor Green
$header | Out-File $summaryFile -Encoding utf8

while ((Get-Date) -lt $deadline) {
    $iteration++
    $gcodeFile = $gcodeFiles[$fileIdx % $gcodeFiles.Count]
    $shortName = [System.IO.Path]::GetFileNameWithoutExtension($gcodeFile)
    $logFile = "$logDir\run_${iteration}_${shortName}.txt"
    $elapsed = (Get-Date) - $startTime

    $msg = "[$(Get-Date -Format 'HH:mm:ss')] === Iteration $iteration | $shortName | Total elapsed: $("{0:hh\:mm\:ss}" -f $elapsed) ==="
    Write-Host $msg -ForegroundColor Yellow
    $msg | Out-File $summaryFile -Append -Encoding utf8

    # Run test: Start-Process captures stdout+stderr to files, we monitor live
    $iterStart = Get-Date
    $stdoutLog = $logFile
    $stderrLog = "$logDir\run_${iteration}_stderr.txt"

    $args = @("gcode", $gcodeFile, "--port", "COM3", "--config", $config)
    if ($SpeedFactor -ne 1.0) { $args += "--speed-factor"; $args += $SpeedFactor.ToString() }

    $proc = Start-Process -FilePath $exe -ArgumentList $args `
        -RedirectStandardOutput $stdoutLog -RedirectStandardError $stderrLog `
        -NoNewWindow -PassThru

    # Live progress monitor: print last progress line every 30s
    while (-not $proc.HasExited) {
        Start-Sleep -Seconds 30
        if (Test-Path $stdoutLog) {
            $lastProgress = Select-String -Path $stdoutLog -Pattern "^\[[\d:.]+\] (L\d|Flush)" -SimpleMatch:$false |
                Select-Object -Last 1
            if ($lastProgress) {
                $runMin = [math]::Round(((Get-Date) - $iterStart).TotalMinutes, 1)
                Write-Host "  [$runMin min] $($lastProgress.Line)" -ForegroundColor DarkCyan
            }
        }
    }
    $proc.WaitForExit()
    $exitCode = $proc.ExitCode
    $iterElapsed = (Get-Date) - $iterStart

    # Merge stderr into main log
    if (Test-Path $stderrLog) {
        $stderrContent = Get-Content $stderrLog -ErrorAction SilentlyContinue
        if ($stderrContent) {
            $stderrContent | Out-File $stdoutLog -Append -Encoding utf8
        }
        Remove-Item $stderrLog -ErrorAction SilentlyContinue
    }

    # Parse result
    $lastLines = Get-Content $logFile -ErrorAction SilentlyContinue | Select-Object -Last 15
    $hasShutdown = ($lastLines | Where-Object { $_ -match "SHUTDOWN|ERROR" }).Count -gt 0
    $finished = ($lastLines | Where-Object { $_ -match "Print finished.*0 errors" }).Count -gt 0

    if ($finished -and -not $hasShutdown -and $exitCode -eq 0) {
        $status = "OK"
        $totalOk++
        $color = "Green"
    } else {
        $status = "FAIL (exit=$exitCode)"
        $totalFail++
        $color = "Red"
        # Extract error details
        $errorLine = $lastLines | Where-Object { $_ -match "SHUTDOWN|ERROR" } | Select-Object -First 1
        if ($errorLine) { $status += " $errorLine" }
    }

    $result = "[$(Get-Date -Format 'HH:mm:ss')]   Result: $status | Duration: $("{0:hh\:mm\:ss}" -f $iterElapsed) | OK=$totalOk FAIL=$totalFail"
    Write-Host $result -ForegroundColor $color
    $result | Out-File $summaryFile -Append -Encoding utf8

    # On failure, wait before retry (MCU may need time to recover)
    if ($status -match "FAIL") {
        Write-Host "  Waiting 10s for MCU recovery..." -ForegroundColor DarkYellow
        Get-Process klipper_host -ErrorAction SilentlyContinue | Stop-Process -Force
        Start-Sleep -Seconds 10
    } else {
        # Brief pause between successful runs
        Start-Sleep -Seconds 3
    }

    $fileIdx++
}

$totalElapsed = (Get-Date) - $startTime
$footer = "=== Endurance test finished $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') | Total: $("{0:hh\:mm\:ss}" -f $totalElapsed) | OK=$totalOk FAIL=$totalFail ==="
Write-Host $footer -ForegroundColor Green
$footer | Out-File $summaryFile -Append -Encoding utf8

} finally {
    Remove-Item $lockFile -ErrorAction SilentlyContinue
}
