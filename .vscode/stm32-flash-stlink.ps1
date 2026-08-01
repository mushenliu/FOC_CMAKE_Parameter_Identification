param([string]$Bin)
# 若未指定固件或文件不存在，自动搜索 build 目录下最新的 .elf
if (-not $Bin -or -not (Test-Path $Bin)) {
    $Bin = Get-ChildItem -Path (Join-Path $PSScriptRoot '..\build') -Recurse -Filter *.elf -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName
}
# ============================================================
# STM32 烧录脚本 [ST-Link] —— 纯中文步骤提示 + 实时进度
# - 用 .NET Process 实时捕获 cube 输出，解析 "NN%" 进度写进度文件
#   %TEMP%\stm32-flash-progress.txt（扩展侧边栏按钮轮询它显示进度条）
# - cube 详细输出进 %TEMP%\stm32-flash.log 备查，终端 100% 中文
# - 去掉 -q：保留 cube 进度输出用于解析百分比（捕获到变量/文件，
#   不经终端显示，所以不会再有进度条乱码问题）
# ============================================================

chcp 65001 > $null
Write-Host ""
Write-Host "=========== STM32 烧录工具 [ST-Link] ==========="

if (-not $Bin -or -not (Test-Path $Bin)) {
    Write-Host " [失败] 未找到固件文件，请先在面板点「编译」。"
    exit 1
}

$logFile  = "$env:TEMP\stm32-flash.log"
$progFile = "$env:TEMP\stm32-flash-progress.txt"
Remove-Item $logFile, $progFile -ErrorAction SilentlyContinue

Write-Host " [1/3] 正在连接 ST-Link 调试器 ..."
Write-Host " [2/3] 正在下载固件到芯片 ..."

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = "cube"
$psi.Arguments = "programmer --connect port=swd --download `"$Bin`" incremental -v fast -rst"
$psi.UseShellExecute = $false
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true
$psi.CreateNoWindow = $true

try {
    $p = [System.Diagnostics.Process]::Start($psi)
} catch {
    Write-Host " [失败] 无法启动 cube 命令：$($_.Exception.Message)"
    Write-Host " [提示] 请确认 STM32CubeIDE for VS Code 扩展包已安装。"
    exit 1
}

# 实时读取 cube 输出，解析百分比（参考 EIDE 的做法：正则 \d+%）
$buf   = New-Object char[] 2048
$logSb = New-Object System.Text.StringBuilder
$lastPct = -1
while (-not $p.HasExited -or $p.StandardOutput.Peek() -ge 0) {
    if ($p.StandardOutput.Peek() -ge 0) {
        $n = $p.StandardOutput.Read($buf, 0, $buf.Length)
        if ($n -gt 0) {
            $chunk = -join $buf[0..($n - 1)]
            [void]$logSb.Append($chunk)
            $m = [regex]::Matches($chunk, "(\d+)\s*%")
            if ($m.Count -gt 0) {
                $pct = [int]$m[$m.Count - 1].Groups[1].Value
                if ($pct -ne $lastPct) {
                    $lastPct = $pct
                    [System.IO.File]::WriteAllText($progFile, "$pct")
                    Write-Host " [烧录中] ${pct}%"
                }
            }
        }
    }
    Start-Sleep -Milliseconds 30
}
$p.WaitForExit()
$code = $p.ExitCode

# 补读 stderr 并写入日志
$errOut = $p.StandardError.ReadToEnd()
if ($errOut) { [void]$logSb.Append($errOut) }
[System.IO.File]::WriteAllText($logFile, $logSb.ToString())

if ($code -eq 0) {
    Write-Host " [3/3] 固件下载成功，目标已复位运行。"
    Write-Host ""
    Write-Host " ============ 烧录完成 ============"
    Write-Host ""
} else {
    Write-Host " [3/3] 烧录失败！"
    Write-Host "       详细错误见日志：$logFile"
    exit 1
}