# =============================================================================
# Sequential x8 speed test for all gcode files
# Results are logged to test_all_x8_results.log
# =============================================================================

param(
    [double]$SpeedFactor = 8.0,
    [string]$Configuration = "Debug",
    [string]$LogFile = "test_all_x8_results.log",
    [string]$IncludePattern = "*.gcode",
    [int]$MaxFiles = 0,
    [switch]$PauseAtEnd
)

$ErrorActionPreference = "Continue"

$exeCandidates = @(
    ".\klipper_host\x64\$Configuration\klipper_host.exe",
    ".\klipper_host\x64\Debug\klipper_host.exe",
    ".\klipper_host\x64\Release\klipper_host.exe"
)
$exe = $null
foreach ($c in $exeCandidates) {
    if (Test-Path $c) {
        $exe = $c
        break
    }
}
if (-not $exe) {
    throw "klipper_host.exe not found. Build first (Debug or Release)."
}

Write-Host "Scanning gcode files..." -ForegroundColor Cyan

# Fast line counting via StreamReader (avoids loading entire file into memory)
function Count-Lines([string]$path) {
    $count = 0
    try {
        $reader = [System.IO.StreamReader]::new($path)
        while ($null -ne $reader.ReadLine()) { $count++ }
        $reader.Close()
    } catch { $count = 0 }
    return $count
}

# Collect all gcode files, count lines quickly, sort smallest first
$files = @()
Get-ChildItem -LiteralPath "gcode" -Filter "*.gcode" |
    Where-Object { $_.Name -like $IncludePattern } |
    ForEach-Object {
        Write-Host "  Counting: $($_.Name) ... " -NoNewline
        $lc = Count-Lines $_.FullName
        Write-Host "$lc lines" -ForegroundColor Gray
        $files += [PSCustomObject]@{ Name = $_.Name; Path = $_.FullName; Lines = $lc }
    }
$files = @($files | Sort-Object Lines)
if ($MaxFiles -gt 0) {
    $files = @($files | Select-Object -First $MaxFiles)
}

$total = $files.Count
$passed = 0
$failed = 0
$skipped = 0

# Header
$header = @"

==============================================================================
  Klipper Host C++ - Sequential x8 Speed Test
  Date : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
    Speed: x$SpeedFactor
    Exe  : $exe
  Files: $total
==============================================================================
"@
Write-Host $header -ForegroundColor Cyan
$header | Out-File -FilePath $logFile -Encoding utf8

foreach ($f in $files) {
    $idx = [array]::IndexOf($files, $f) + 1
    $separator = "`n" + ("=" * 78)
    $testHeader = "$separator`n[$idx/$total] $($f.Name) ($($f.Lines) lines)"

    Write-Host $testHeader -ForegroundColor Yellow
    $testHeader | Out-File -FilePath $logFile -Append -Encoding utf8

    # Skip empty/broken files
    if ($f.Lines -le 10) {
        $msg = "  SKIPPED: file too small ($($f.Lines) lines)"
        Write-Host $msg -ForegroundColor DarkGray
        $msg | Out-File -FilePath $logFile -Append -Encoding utf8
        $skipped++
        continue
    }

    $safeName = $f.Name -replace '[^a-zA-Z0-9_\.]','_'
    $outFile = "test_x8_$safeName.txt"
    $startTime = Get-Date

    Write-Host "  Start: $(Get-Date -Format 'HH:mm:ss')  Log: $outFile" -ForegroundColor Gray

    # Run test: stream merged stdout/stderr in real time and save to file
    "" | Out-File -FilePath $outFile -Encoding utf8

    $lastFlush = ""
    & (Resolve-Path $exe).Path gcode "$($f.Path)" --speed-factor $SpeedFactor 2>&1 |
        ForEach-Object {
            $line = "$_"
            $line | Out-File -FilePath $outFile -Append -Encoding utf8
            if ($line -match "^$") { return }
            if ($line -match "Flush #(\d+)") {
                $flushNum = [int]$Matches[1]
                if ($flushNum % 100 -eq 0) {
                    Write-Host "  $line" -ForegroundColor DarkCyan
                }
                $lastFlush = $line
            } else {
                Write-Host "  $line"
            }
        }
    $exitCode = $LASTEXITCODE

    $endTime = Get-Date
    $duration = $endTime - $startTime
    $durStr = "{0:hh\:mm\:ss}" -f $duration

    # Check for SHUTDOWN
    $hasShutdown = $false
    $shutdownMsg = ""
    if (Test-Path $outFile) {
        # Avoid false positives from identify JSON/static strings containing "shutdown".
        $shutdownLine = Select-String -LiteralPath $outFile -Pattern "!!! MCU SHUTDOWN|ERROR: MCU SHUTDOWN" | Select-Object -First 1
        if ($shutdownLine) {
            $hasShutdown = $true
            $shutdownMsg = $shutdownLine.Line
        }
    }

    # Determine result
    if ($exitCode -eq 0 -and -not $hasShutdown) {
        $result = "PASS"
        $color = "Green"
        $passed++
    } else {
        $result = "FAIL (exit=$exitCode)"
        $color = "Red"
        $failed++
    }

    $summary = @"

  *** Result  : $result
  *** Duration: $durStr
  *** ExitCode: $exitCode
"@
    if ($hasShutdown) {
        $summary += "`n  *** Shutdown: $shutdownMsg"
    }
    if ($lastFlush) {
        $summary += "`n  *** Last: $lastFlush"
    }

    Write-Host $summary -ForegroundColor $color
    $summary | Out-File -FilePath $logFile -Append -Encoding utf8
}

# Final summary
$finalSummary = @"

==============================================================================
  FINAL SUMMARY
==============================================================================
  Total : $total
  Passed: $passed
  Failed: $failed
  Skipped: $skipped
  Date  : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
==============================================================================
"@
Write-Host $finalSummary -ForegroundColor Cyan
$finalSummary | Out-File -FilePath $logFile -Append -Encoding utf8

Write-Host "`nResults saved to: $logFile" -ForegroundColor White
if ($PauseAtEnd) {
    Write-Host "Press any key to close..." -ForegroundColor DarkGray
    $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
}
