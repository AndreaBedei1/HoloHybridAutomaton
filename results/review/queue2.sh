#!/usr/bin/env bash
cd /f/Andrea/HoloHybridAutomaton || exit 1
PY=/c/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe
export HOLO_FLEET_ENGINE_SLOTS=2
while powershell -c "@(Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { \$_.CommandLine -like '*raw_adversarial *' -or \$_.CommandLine -like '*raw_adversarial\"*' -or \$_.CommandLine -like '*raw_adversarial'+[char]32+'*' }).Count" | grep -qv '^0'; do sleep 60; done
$PY scripts/run_sensor_robustness_campaign.py --phase targeted --only st_rear_pusher st_rear_pusher_close --protocols B2_MEDIUM --profiles MEDIUM-ALL --seeds 7000 7001 7002 --raw results/review/raw_adversarial_v2 > results/review/adv_v2.log 2>&1
echo "ADV2 DONE $(date)" > results/review/adv2_done.txt
