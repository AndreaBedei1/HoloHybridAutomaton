import json, glob, gzip, collections, sys
R = "results/robustness/raw_b2"
out = []
for p in sorted(glob.glob(f"{R}/*/*/*/s*/run_metrics.json")):
    r = json.load(open(p))
    if r.get("class") in ("PASS", "INVALID"):
        continue
    d = p.rsplit("/", 1)[0].replace("\\", "/")
    term = r.get("terminal") or {}
    p3 = r.get("P3") or {}
    au = r.get("automaton") or {}
    ce = r.get("conservative_events") or {}
    print(f"{r['protocol']:11s} {r['profile']:13s} {r['scenario']:31s} s{r['seed']} {r['class'][:4]} dl {term.get('deadline_s')} "
          f"fs_entries {au.get('failsafe_entries')} fs_sensor {ce.get('failsafe_sensor_fault_entries')} fs_t {au.get('failsafe_time_healthy_s')} "
          f"falsevac {ce.get('false_vacancy')} miss {p3.get('missing_episodes')} fmiss {p3.get('false_missing_episodes')} "
          f"tmd {p3.get('true_missing_delay_s')} wait {p3.get('wait_rejoin_s_mean')} deg {p3.get('degraded_entries')} "
          f"end {[m[:12] for m in au.get('end_modes', [])]}")
