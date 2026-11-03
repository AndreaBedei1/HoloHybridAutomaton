import sys, numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from gate_metrics import load, RAW
from p3_failures import per_drone_dev
RUNS = ["B2_BLACKOUT/MEDIUM-ALL/formation_recovery_head_current/s1", "B2_BLACKOUT/MEDIUM-ALL/formation_six/s2", "B2_BLACKOUT/MEDIUM-ALL/lost_drone_rejoin/s1",
        "B2_BLACKOUT/MEDIUM-ALL/p1_vertical_escape/s1", "B2_LOW/LOW-ALL/formation_six/s0", "B2_MEDIUM/MEDIUM-ALL/formation_six/s1", "B2_MEDIUM/MEDIUM-ONE/formation_gust/s2",
        "B2_MEDIUM/MEDIUM-ONE/formation_square/s0", "B2_MEDIUM/MEDIUM-ONE/formation_square/s2", "B2_MEDIUM/MEDIUM-SUBSET/formation_six/s2", "B2_MEDIUM/MEDIUM-SUBSET/formation_square/s0",
        "B2_MEDIUM/MEDIUM-ALL/p1_two_lines/s2"]
for rel in RUNS:
    d = RAW.joinpath(*rel.split("/")); rm, sc, ts, n, T, P, st, ev = load(d)
    tau = [st[k][max(st[k])]["form"].get("tau") for k in range(n)]
    tm = sc.template.offsets()
    # along-track (path-tangent) component of each drone's deviation at the end
    X = P[-1]; c = X.mean(axis=0); s_c, _l, _k = sc.path.project(c[:2]); _p, tan, nrm = sc.path.frame_at(s_c)
    rel_f = np.stack([(X - c)[:, :2] @ tan, (X - c)[:, :2] @ nrm, (X - c)[:, 2]], axis=1) - (tm - tm.mean(axis=0))
    # tau at 3 times
    taus = {tt: [round(st[k][min(st[k], key=lambda x: abs(x - tt))]["form"].get("tau") or 0.0, 1) for k in range(n)] for tt in (30, 60, T[-1])}
    print(rel.split("/", 1)[1], "| tau(t):", {round(k): v for k, v in taus.items()}, "| along err", np.round(rel_f[:, 0], 2).tolist(), "lat", np.round(rel_f[:, 1], 2).tolist(),
          "| v_nom*(tau-mean)", np.round(0.3 * (np.array(tau) - np.mean(tau)), 2).tolist())
