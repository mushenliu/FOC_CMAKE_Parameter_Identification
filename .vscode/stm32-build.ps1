param(
    [string]$BuildDir = "build/Debug"   # 构建目录，Release 构建改为 build/Release
)
# ============================================================
# STM32 编译脚本 —— 捕获 cube-cmake 输出并做中文提示
# - "ninja: no work to do" → "项目无改动，无需重新编译"
# - 其余编译输出（警告/错误）原样显示，保证报错信息不丢失
# ============================================================

chcp 65001 > $null
Write-Host ""
Write-Host "=========== STM32 编译工具 ==========="
Write-Host " [编译] 构建目录：$BuildDir"
Write-Host ""

& cube-cmake --build $BuildDir 2>&1 | ForEach-Object {
    if ($_ -match "ninja: no work to do") {
        Write-Host " [提示] 项目无改动，无需重新编译。"
    } else {
        Write-Host $_
    }
}

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host " ============ 编译完成 ============"
    Write-Host ""
} else {
    Write-Host ""
    Write-Host " [失败] 编译出错，请检查上方错误信息。"
    Write-Host ""
    exit 1
}
