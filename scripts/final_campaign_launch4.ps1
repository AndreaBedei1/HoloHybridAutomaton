param([string]$WtA, [string]$WtB, [string]$Raw, [string]$List, [string]$Tag)
$bash = 'C:\Program Files\Git\bin\bash.exe'
$lane = (Join-Path $PSScriptRoot 'final_campaign_lane.sh').Replace([char]92,[char]47)
$lanes = @(@($WtA,'F:/Andrea/holoocean_patched_root','A0'), @($WtB,'F:/Andrea/holoocean_patched_root_b','B0'), @($WtA,'F:/Andrea/holoocean_patched_root','A1'), @($WtB,'F:/Andrea/holoocean_patched_root_b','B1'))
foreach ($l in $lanes) {
  $p = Start-Process -FilePath $bash -ArgumentList @('-c', "`"'$lane' '$($l[0])' '$($l[1])' '$Raw' '$List' '$Tag$($l[2])'`"") -WindowStyle Hidden -PassThru
  Write-Output "lane $($l[2]) bash pid $($p.Id)"
  Start-Sleep -Seconds 20
}
