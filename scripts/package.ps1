<#
.SYNOPSIS
    Packages the SRPlatform build into a standalone runnable zip.

.DESCRIPTION
    Copies the built executables, the w64devkit runtime DLLs, assets, docs
    and README into dist/SRPlatform-<version>-win64 and zips the folder.
    Run from the repository root. Use -BuildDir to package another build
    configuration (e.g. build/ninja-release).

.EXAMPLE
    ./scripts/package.ps1
#>
param(
    [string]$BuildDir = "build/ninja-debug",
    [string]$OutDir = "dist"
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

$BuildDirFull = Join-Path $Root $BuildDir
if (-not (Test-Path (Join-Path $BuildDirFull "srp_window.exe"))) {
    throw "Build directory $BuildDirFull does not contain srp_window.exe; build first."
}

$Version = "0.1.0"
$VersionFile = Join-Path $BuildDirFull "srp_version.txt"
if (Test-Path $VersionFile) {
    $Version = (Get-Content $VersionFile).Trim()
}

$PackageName = "SRPlatform-$Version-win64"
$PackageDir = Join-Path $Root (Join-Path $OutDir $PackageName)

if (Test-Path $PackageDir) {
    Remove-Item -LiteralPath $PackageDir -Recurse -Force
}
New-Item -ItemType Directory -Path $PackageDir -Force | Out-Null

$BinDir = Join-Path $PackageDir "bin"
New-Item -ItemType Directory -Path $BinDir -Force | Out-Null

Copy-Item (Join-Path $BuildDirFull "srp_window.exe") $BinDir
Copy-Item (Join-Path $BuildDirFull "srp_cli.exe") $BinDir

# Copy the MinGW runtime DLLs the executables link against.
$ToolchainBin = "C:\Program Files\w64devkit\bin"
$RuntimeDlls = @(
    "libgcc_s_seh-1.dll",
    "libstdc++-6.dll",
    "libwinpthread-1.dll"
)
foreach ($dll in $RuntimeDlls) {
    $source = Join-Path $ToolchainBin $dll
    if (Test-Path $source) {
        Copy-Item $source $BinDir
    }
}

Copy-Item (Join-Path $Root "assets") (Join-Path $PackageDir "assets") -Recurse
Copy-Item (Join-Path $Root "docs") (Join-Path $PackageDir "docs") -Recurse
Copy-Item (Join-Path $Root "README.md") $PackageDir
Copy-Item (Join-Path $Root "CONTRIBUTING.md") $PackageDir

$ZipPath = Join-Path (Join-Path $Root $OutDir) "$PackageName.zip"
if (Test-Path $ZipPath) {
    Remove-Item -LiteralPath $ZipPath -Force
}
Compress-Archive -Path $PackageDir -DestinationPath $ZipPath -CompressionLevel Optimal

Write-Host "Packaged: $ZipPath"
