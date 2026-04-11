# =============================================================================
# Sequential x8 speed test for all gcode files
# Results are logged to test_all_x8_results.log
# =============================================================================

$ErrorActionPreference = "Continue"
$exe = ".\klipper_host\x64\Release\klipper_host.exe"
$logFile = "test_all_x8_results.log"
$speedFactor = 8.0

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
Get-ChildItem -LiteralPath "gcode" -Filter "*.gcode" | ForEach-Object {
    Write-Host "  Counting: $($_.Name) ... " -NoNewline
    $lc = Count-Lines $_.FullName
    Write-Host "$lc lines" -ForegroundColor Gray
    $files += [PSCustomObject]@{ Name = $_.Name; Path = $_.FullName; Lines = $lc }
}
$files = $files | Sort-Object Lines

$total = $files.Count
$passed = 0
$failed = 0
$skipped = 0

# Header
$header = @"

==============================================================================
  Klipper Host C++ - Sequential x8 Speed Test
  Date : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
  Speed: x$speedFactor
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

    # Run test: redirect all output to file, stream to console line by line
    $process = New-Object System.Diagnostics.Process
    $process.StartInfo.FileName = (Resolve-Path $exe).Path
    $process.StartInfo.Arguments = "gcode `"$($f.Path)`" --speed-factor $speedFactor"
    $process.StartInfo.WorkingDirectory = (Get-Location).Path
    $process.StartInfo.UseShellExecute = $false
    $process.StartInfo.RedirectStandardOutput = $true
    $process.StartInfo.RedirectStandardError = $true
    $process.StartInfo.CreateNoWindow = $true

    # Clear output file
    "" | Out-File -FilePath $outFile -Encoding utf8

    $process.Start() | Out-Null

    # Read stdout line-by-line for real-time display
    $lastFlush = ""
    while (-not $process.StandardOutput.EndOfStream) {
        $line = $process.StandardOutput.ReadLine()
        $line | Out-File -FilePath $outFile -Append -Encoding utf8
        # Show every 10th flush and all non-flush lines to keep console readable
        if ($line -match "^$") { continue }
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
    # Drain stderr
    $stderr = $process.StandardError.ReadToEnd()
    if ($stderr) {
        $stderr | Out-File -FilePath $outFile -Append -Encoding utf8
        Write-Host "  $stderr" -ForegroundColor Red
    }

    $process.WaitForExit()
    $exitCode = $process.ExitCode

    $endTime = Get-Date
    $duration = $endTime - $startTime
    $durStr = "{0:hh\:mm\:ss}" -f $duration

    # Check for SHUTDOWN
    $hasShutdown = $false
    $shutdownMsg = ""
    if (Test-Path $outFile) {
        $shutdownLine = Select-String -LiteralPath $outFile -Pattern "SHUTDOWN" -SimpleMatch | Select-Object -First 1
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
Write-Host "Press any key to close..." -ForegroundColor DarkGray
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
