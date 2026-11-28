param([Parameter(Mandatory)][string]$MapRoot, [Parameter(Mandatory)][string]$CloudCompare)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path -LiteralPath $MapRoot).Path
$cc = (Resolve-Path -LiteralPath $CloudCompare).Path
$out = New-Item -ItemType Directory -Force (Join-Path $root spatial_5cm)
foreach ($stage in 'A','B','C') {
    $folder = if ($stage -eq 'C') { 'structure' } else { 'pgo' }
    $version = if ($stage -eq 'A') { 'before' } else { 'after' }
    foreach ($type in 'geometry','rgb') {
        $prefix = if ($type -eq 'rgb') { 'map_rgb' } else { 'map' }
        $file = Join-Path $root "$folder/${prefix}_${version}.pcd"
        $dest = Join-Path $out.FullName "${stage}_${type}.pcd"
        if (Test-Path -LiteralPath $dest) { throw "Output exists: $dest" }
        & $cc -SILENT -AUTO_SAVE OFF -O $file -SS SPATIAL 0.05 -C_EXPORT_FMT PCD -SAVE_CLOUDS FILE $dest | Out-Null
        if ($LASTEXITCODE -ne 0 -or !(Test-Path -LiteralPath $dest)) { throw "PCD export failed: $dest" }
        Get-Item -LiteralPath $dest | Select-Object Name,Length
    }
}
