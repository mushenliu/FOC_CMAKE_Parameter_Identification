param([string]$Bin)
# 若未指定固件或文件不存在，自动搜索 build 目录下最新的 .elf
if (-not $Bin -or -not (Test-Path $Bin)) {
    $Bin = Get-ChildItem -Path (Join-Path $PSScriptRoot '..\build') -Recurse -Filter *.elf -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName
}
# ============================================================
# STM32 烧录脚本 [J-Link] —— 基于 SEGGER JLink.exe 命令行
# - 需安装 SEGGER J-Link 软件包（JLink.exe）
# - 首次使用前请按你的实际环境修改两处：
#     1) $JLinkExe —— JLink.exe 的安装路径
#     2) $Device   —— J-Link 支持的器件名（在 J-Link 安装目录
#        Doc/ 下搜 "DeviceSupport" 或官网查询准确写法，如 STM32G431CB）
# - 实时解析 JLink 输出 "NN%" 写进度文件 %TEMP%\stm32-flash-progress.txt，
#   详细输出进 %TEMP%\stm32-flash.log 备查，终端 100% 中文
# ============================================================

chcp 65001 > $null
Write-Host ""
Write-Host "=========== STM32 烧录工具 [J-Link] ==========="

if (-not $Bin -or -not (Test-Path $Bin)) {
    Write-Host " [失败] 未找到固件文件，请先在面板点「编译」。"
    exit 1
}

# ---- 按实际环境修改这两行 ----
$JLinkExe = "C:\Program Files\SEGGER\JLink\JLink.exe"   # JLink.exe 路径
$Device   = "STM32G431CB"                                # 器件名（J-Link 准确写法）
# ------------------------------

if (-not (Test-Path $JLinkExe)) {
    Write-Host " [失败] 找不到 JLink.exe：$JLinkExe"
    Write-Host " [提示] 请安装 SEGGER J-Link 软件，并修改脚本里的路径。"
    exit 1
}

# 生成 J-Link 命令脚本
$Script = "$env:TEMP\stm32-flash.jlink"
"loadfile $Bin" | Set-Content $Script
"r"             | Add-Content $Script
"g"             | Add-Content $Script
"exit"          | Add-Content $Script

$logFile  = "$env:TEMP\stm32-flash.log"
$progFile = "$env:TEMP\stm32-flash-progress.txt"
Remove-Item $logFile, $progFile -ErrorAction SilentlyContinue

Write-Host " [1/3] 正在连接 J-Link 调试器 ..."
Write-Host " [2/3] 正在下载固件到芯片 ..."

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = $JLinkExe
$psi.Arguments = "-device $Device -if SWD -speed 4000 -autoconnect 1 -CommanderScript `"$Script`""
$psi.UseShellExecute = $false
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true
$psi.CreateNoWindow = $true

try {
    $p = [System.Diagnostics.Process]::Start($psi)
} catch {
    Write-Host " [失败] 无法启动 JLink.exe：$($_.Exception.Message)"
    Write-Host " [提示] 请确认 SEGGER J-Link 软件已安装，路径配置正确。"
    exit 1
}

# 实时读取 JLink 输出，解析百分比
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