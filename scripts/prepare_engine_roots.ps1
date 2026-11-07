# Immutable engine/content are shared; every writable engine file belongs to one project slot.
param([string]$SourceRoot = 'F:\Andrea\holoocean_patched_root')
$ErrorActionPreference = 'Stop'
$source = (Resolve-Path -LiteralPath $SourceRoot).Path
$ocean = Join-Path $source '2.3.0\worlds\Ocean'
$sourceExe = Join-Path $ocean 'Windows\Holodeck\Binaries\Win64\Holodeck.exe'
$hash = (Get-FileHash -LiteralPath $sourceExe -Algorithm SHA256).Hash
foreach ($slot in 0, 1) {
    $target = $source + '_slot' + $slot
    if (Test-Path -LiteralPath $target) { throw "Refusing to overwrite existing root: $target" }
    $dest = Join-Path $target '2.3.0\worlds\Ocean'
    New-Item -ItemType Directory -Path (Join-Path $dest 'Windows\Holodeck') -Force | Out-Null
    Get-ChildItem -LiteralPath $ocean -File | Copy-Item -Destination $dest
    Get-ChildItem -LiteralPath (Join-Path $ocean 'Windows') -File | Copy-Item -Destination (Join-Path $dest 'Windows')
    Copy-Item -LiteralPath (Join-Path $ocean 'Windows\Holodeck\Binaries') -Destination (Join-Path $dest 'Windows\Holodeck') -Recurse
    New-Item -ItemType Junction -Path (Join-Path $dest 'Windows\Engine') -Target (Join-Path $ocean 'Windows\Engine') | Out-Null
    New-Item -ItemType Junction -Path (Join-Path $dest 'Windows\Holodeck\Content') -Target (Join-Path $ocean 'Windows\Holodeck\Content') | Out-Null
    foreach ($name in 'Octrees', 'Saved') {
        New-Item -ItemType Directory -Path (Join-Path $dest "Windows\Holodeck\$name") | Out-Null
    }
    $info = Get-Content -LiteralPath (Join-Path $source 'PATCH_INFO.json') -Raw | ConvertFrom-Json
    $info | Add-Member -NotePropertyName isolation_source -NotePropertyValue $source -Force
    $info | Add-Member -NotePropertyName isolation_slot -NotePropertyValue $slot -Force
    $info | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $target 'PATCH_INFO.json') -Encoding utf8
    $copiedHash = (Get-FileHash -LiteralPath (Join-Path $dest 'Windows\Holodeck\Binaries\Win64\Holodeck.exe') -Algorithm SHA256).Hash
    if ($copiedHash -ne $hash) { throw "Engine copy hash mismatch: $target" }
    foreach ($path in 'materials.csv', 'Windows\Holodeck\Octrees', 'Windows\Holodeck\Saved') {
        if ((Get-Item -LiteralPath (Join-Path $dest $path)).LinkType) { throw "Writable path is linked: $path" }
    }
    Write-Output "Prepared $target (engine SHA256 $copiedHash)"
}
