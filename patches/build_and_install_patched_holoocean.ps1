# Build the patched Holodeck engine (UE 5.3, Win64 Development) and install the executable into the
# separate HoloOcean root used by HoloHybridAutomaton.  The official installation in
# %LOCALAPPDATA%\holoocean is never modified.
param(
    [string]$UE = "C:\Program Files\Epic Games\UE_5.3",
    [string]$Root = "F:\Andrea\holoocean_patched_root"
)
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$log = Join-Path $here "build_holodeck_development.log"
& "$UE\Engine\Build\BatchFiles\Build.bat" Holodeck Win64 Development "-Project=$here\engine\Holodeck.uproject" -WaitMutex -NoHotReloadFromIDE *> $log
if ($LASTEXITCODE -ne 0) { Get-Content $log -Tail 30; throw "build failed ($LASTEXITCODE)" }

$official = "$env:LOCALAPPDATA\holoocean\2.3.0\worlds\Ocean"
$dst = "$Root\2.3.0\worlds\Ocean"
New-Item -ItemType Directory -Force "$dst\Windows\Holodeck\Binaries\Win64" | Out-Null
Get-ChildItem $official -File | ForEach-Object { Copy-Item $_.FullName $dst -Force }
Get-ChildItem "$official\Windows" -File | ForEach-Object { Copy-Item $_.FullName "$dst\Windows" -Force }
if (-not (Test-Path "$dst\Windows\Engine")) { New-Item -ItemType Junction -Path "$dst\Windows\Engine" -Target "$official\Windows\Engine" | Out-Null }
if (-not (Test-Path "$dst\Windows\Holodeck\Content")) { New-Item -ItemType Junction -Path "$dst\Windows\Holodeck\Content" -Target "$official\Windows\Holodeck\Content" | Out-Null }
$bin = "$official\Windows\Holodeck\Binaries\Win64"
foreach ($f in "OpenImageDenoise.dll", "tbb.dll", "tbb12.dll", "tbbmalloc.dll") { Copy-Item "$bin\$f" "$dst\Windows\Holodeck\Binaries\Win64" -Force }
Copy-Item "$bin\D3D12" "$dst\Windows\Holodeck\Binaries\Win64" -Recurse -Force
Copy-Item "$here\engine\Binaries\Win64\Holodeck.exe" "$dst\Windows\Holodeck\Binaries\Win64\Holodeck.exe" -Force
Copy-Item "$here\engine\Binaries\Win64\Holodeck.pdb" "$dst\Windows\Holodeck\Binaries\Win64\Holodeck.pdb" -Force
# a new engine build invalidates every cached octree of the patched root
if (Test-Path "$dst\Windows\Holodeck\Octrees") { Remove-Item -Recurse -Force "$dst\Windows\Holodeck\Octrees" }
$hash = (Get-FileHash "$dst\Windows\Holodeck\Binaries\Win64\Holodeck.exe" -Algorithm SHA256).Hash
$commit = (git -C $here rev-parse --short HEAD)
@{ patch = "sonar-octree-rebuild"; source = "$here (branch sonar-octree-rebuild)"; source_commit = $commit;
   holoocean = "2.3.0"; unreal = "5.3.2"; exe_sha256 = $hash; built = (Get-Date).ToString("s") } |
    ConvertTo-Json | Out-File -Encoding utf8 "$Root\PATCH_INFO.json"
Get-Content "$Root\PATCH_INFO.json"
