# ==============================================================================
# cube_regen.ps1
# One-click CubeMX code regeneration with encoding safety.
#
# Problem: CubeMX reads/writes source as UTF-8, but this project uses GBK.
#          Regenerating a GBK file corrupts Chinese USER CODE (锟斤拷 / ????).
# Solution: Convert GBK->UTF-8 before CubeMX, launch CubeMX, wait for it to
#           close, then convert UTF-8->GBK. Files stay GBK on disk (Keil happy).
#
# Usage:
#   .\cube_regen.ps1
#     - Runs the encoding helper -Pre (GBK -> UTF-8)
#     - Launches STM32CubeMX with nx_car.ioc
#     - Waits for you to finish and CLOSE CubeMX
#     - Runs the encoding helper -Post (UTF-8 -> GBK)
#
# If CubeMX is already open, close it first, then run this script.
# ==============================================================================

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$iocPath     = Join-Path $projectRoot "nx_car.ioc"
$helper      = Join-Path $projectRoot "cubemx_encoding_helper.ps1"
$cubemxExe   = "D:\MyProgram\CubeMX\STM32CubeMX.exe"

if (-not (Test-Path $helper)) {
    Write-Error "cubemx_encoding_helper.ps1 not found at $helper"
    exit 1
}
if (-not (Test-Path $iocPath)) {
    Write-Error "nx_car.ioc not found at $iocPath"
    exit 1
}
if (-not (Test-Path $cubemxExe)) {
    Write-Error "STM32CubeMX.exe not found at $cubemxExe"
    exit 1
}

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host " Step 1/4: Pre  - converting managed files GBK -> UTF-8" -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan
& powershell -ExecutionPolicy Bypass -File $helper -Pre
if ($LASTEXITCODE -ne 0) { Write-Error "Pre step failed"; exit 1 }

Write-Host ""
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host " Step 2/4: Launching STM32CubeMX ..." -ForegroundColor Cyan
Write-Host "          Make your changes, then click GENERATE CODE" -ForegroundColor Cyan
Write-Host "          and CLOSE CubeMX when done." -ForegroundColor Yellow
Write-Host "========================================================" -ForegroundColor Cyan
$proc = Start-Process -FilePath $cubemxExe -ArgumentList $iocPath -PassThru
$proc.WaitForExit()

Write-Host ""
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host " Step 3/4: CubeMX closed. Post - converting UTF-8 -> GBK" -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan
& powershell -ExecutionPolicy Bypass -File $helper -Post
if ($LASTEXITCODE -ne 0) { Write-Error "Post step failed"; exit 1 }

Write-Host ""
Write-Host "========================================================" -ForegroundColor Green
Write-Host " Step 4/4: Done. All managed files are back to GBK." -ForegroundColor Green
Write-Host " Chinese comments preserved. Safe to build in Keil." -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Green