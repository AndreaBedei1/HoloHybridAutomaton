"""B2 communication made explicit: passive one-way beacons, freshness, delays, losses, simultaneous commitments, local knowledge.

No drone reads another drone's true state.  Each drone i decides on its OWN local state only: its own clock and commitment age, the
latest status report it RECEIVED from every other drone (phase, emission slot, hence age on its own clock), the time it last heard
each one, and its own sonar observations.  Another drone's true state enters only through two channels:

* the acoustic channel: drone j emits one packet per TDMA period ``beacon.period_s`` in its slot (offset o_j in [0, P), dated by the
  sender's clock, offset delta_j, |delta_j| <= ``acoustic.clock_skew_s``); a packet is lost (arbitrarily, adversarially) or delivered
  after a latency in [0.10, ``acoustic.latency_max_s``] (a later arrival carries no content: it is a loss); its content is j's gate
  phase as of one control step before the emission (truthful sender: COMMITTED / ADMITTED from its commitment until its passage is
  verified beyond the CR, perception/passage.py); the receiver keeps the latest report per sender for ever;
* the sonar: a function of both true positions, and only where the deployed geometry makes an echo certain (B2C-G1: from the
  admission zone, under the declared gate-frame navigation error, nowhere in the CR); elsewhere free (seen or not).  Free sonar is
  an over-approximation of the observations: the safety results below do not use the sonar at all.

Premises (declared, each a check of another suite or a scope statement): (Pi1) the P2 filter - no centre in the CR before admission
(control/gate_safety.py, formal/check_gate_safety.py); (Pi2) an admitted drone progresses at >= GateRule.v_cross_min until it has left
the CR (failures inside the CR are outside the scope, holo_fleet/scope.py), so it leaves the CR within GateRule.t_cross_max of its
admission; (Pi3) truthful self-reports (payload.py, passage.report_phase: S9 of formal/check_status.py).

The admission rule of the corrected controller (perception/gate_perception.py ``admit_wait_s``, control/flows.py): no claim held,
commitment age >= T_c = ``commit_admit_min_s``, and either a FRESH report (age <= T_f = ``admit_fresh_s``) of every other drone, or a
commitment age >= T_c + D, D = ``t_cross_max``.  The former controller: no claim held (a claim released by rule D after
``t_silent_gate_s`` of silence), nothing else from the channel.

Timeline model (Z3, exact real time, every message pattern): the two admissions of a violating pair f (admitted first) and s.
  B2C-E1   proved: P2 for every loss pattern, latency, slot and clock offset, unless the pair is MUTUALLY unheard (neither holds a fresh
           report of the other at its admission).  Case split: a fresh report of a drone committed >= T_c > T_f + 2 skew + dt earlier
           is a claim; a degraded admission waits D >= t_cross_max, so the first drone's admission falls inside the second's commitment.
  B2C-E2   proved: with at most 2 consecutive losses per link the reports are always fresh: P2 holds without any sonar or B0 argument.
  B2C-E2k  proved: with at most 40 consecutive losses per link (k_max from T_c + D) P2 holds as well: a mutually degraded pair has both
           commitments older than T_c + D, so whatever it receives within that time is a claim.
  B2C-Ea/Eb/Ee proved: concurrent commitment before hearing each other; one drone stationary (committed and holding) whose messages
           never reach the other; a live drone with a failed transmitter (3 drones, the receiver hears a third one).
  B2C-E3   proved: 3 drones - a third drone's reports (any content, any freshness) cannot create a violation.
  B2C-EL   declared limit: a mutually unheard pair (total blackout of that pair) - P2 then rests on the B0 mechanisms only (queue
           precedence and occupancy latch, formal/check_mutex.py M2/M3 on idealised positions; the stage-2 sonar is not certified, G1).
  B2C-Es   scope: k_max + 1 = 41 consecutive losses on both links allow the mutual degradation of B2C-EL (the bound is tight).
  B2C-O*   mutations: the former controller (no commitment age, no freshness, rule D) violates P2 in cases (a), (b), (d), (e) and (c);
           B2C-O0: on a lossless channel the former controller is safe (proved), the race needs losses.
  B2C-Mt/Mw mutations: T_c without the margin over T_f, or D below t_cross_max.
  B2C-W    witnesses: both branches admit (non-vacuity).
Local decisions:
  B2C-L    structural: the admission decision of drone i has no free symbol but its own local view (timeline and step models).
  B2C-Mo   proved: monotone - an added claim, a staler or missing report never enables an admission (B2C-Mom: rule D does).
  B2C-D    proved: an exclusion (status_excluded, the source of every permissive use: phantom echoes, acoustic exit) needs a fresh positive
           report or PASSED: silence never excludes (B2C-Dm: rule D).  Cross-checked on the deployed class (randomized).
Step model (Z3, bounded model checking, dt = 0.5 s): two drones with explicit local state (latch, quiet timer, commitment age,
  received status, sonar observations free), true positions on the gate axis with the speed bound, Pi1, Pi2, TDMA slots of 2 drones,
  per-emission loss, give-up at any time.  B2C-S1 bounded: P2 over 100 s with at most 2 consecutive losses; B2C-S1o mutation: the
  former rule; B2C-S1b limit: a total blackout.
Geometry: B2C-G1 reported - the CR observability from the admission zone (deployed classifier, bars as occluders): certain for no CR
  slice under the declared navigation error, so no admission argument of this suite uses the stage-2 sonar.
Liveness: B2C-S2 is a bounded two-vehicle negotiation model following concurrent commitment and receipt of the lower-priority
vehicle's claim. The higher-priority vehicle requires a delivered withdrawal and a feasible admission, with conditional physical
clearance from Pi2.
B2C-Lv reports the extra admission delays and the timed give-up behavior beyond the early concession window.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import z3

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common import CheckResult, Report, check  # noqa: E402

from holo_fleet.config import DEFAULT, BeaconConfig, FleetConfig  # noqa: E402
from holo_fleet.control.flows import RELOC_ZONE_M  # noqa: E402
from holo_fleet.mission import abreast_slots, queue_line  # noqa: E402
from holo_fleet.perception.exit_witness import speed_bound  # noqa: E402

ENC = "perception/gate_perception.py (admit_wait_s) + perception/acoustic_gate.py + control/flows.py + formal/check_b2_comm.py"
B2 = replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))
LAT_MIN = 0.10
DT_CTRL = 0.1


@dataclass(frozen=True)
class Prm:
    P: float
    lat_min: float
    lat_max: float
    dt_rep: float
    skew: float
    T_f: float
    T_c: float
    D: float
    T_cross: float
    T_silent: float
    T_kin: float


def params(cfg: FleetConfig = B2, **over) -> Prm:
    a, G = cfg.acoustic, cfg.gate
    p = Prm(P=cfg.beacon.period_s, lat_min=LAT_MIN, lat_max=a.latency_max_s, dt_rep=DT_CTRL, skew=a.clock_skew_s,
            T_f=a.admit_fresh_s, T_c=a.commit_admit_min_s, D=G.t_cross_max, T_cross=G.t_cross_max,
            T_silent=a.t_silent_gate_s, T_kin=0.0)
    return replace(p, **over)


def t_kin(cfg: FleetConfig = B2, n: int = 2) -> float:
    """Smallest time from a commitment at the queue point (estimate within queue_tol) to the admission zone (estimate >= stage-2 hold -
    RELOC_ZONE_M) at the speed bound, for the abreast queue of n drones (a physical bound, used only by the former controller's
    counterexamples, never by a proof)."""
    G = cfg.gate
    q_s = queue_line(abreast_slots(n), G, cfg.perc)
    return max(0.0, (G.stage2_hold_s - RELOC_ZONE_M) - (q_s + G.queue_tol)) / speed_bound(cfg)


# ----------------------------------------------------------------------------------------------------------- local views

class RxView:
    """What receiver b holds about sender a (local symbols only): no report ever, the latest report is a claim, its age on b's
    clock (from the slot schedule), the time since b last heard a, and whether the last fix placed a outside the exclusion zone."""

    def __init__(self, b: str, a: str):
        pre = f"L{b}_rx{a}_"
        self.none = z3.Bool(pre + "none")
        self.claim = z3.Bool(pre + "claim")
        self.age = z3.Real(pre + "age")
        self.silence = z3.Real(pre + "silence")
        self.outside = z3.Bool(pre + "outside")

    def blocking(self):
        return z3.And(z3.Not(self.none), self.claim)

    def fresh(self, T_f):
        return z3.And(z3.Not(self.none), self.age <= T_f)


class LocalView:
    """The whole local state an admission decision of drone b may read."""

    def __init__(self, b: str, others: List[str]):
        self.b = b
        self.ca = z3.Real(f"L{b}_commit_age")
        self.sonar_ok = z3.Bool(f"L{b}_sonar_ok")
        self.rx = {a: RxView(b, a) for a in others}


def admit_new(L: LocalView, p: Prm, ignore_claim: Optional[str] = None):
    """Corrected rule: no claim held; commitment age >= T_c; all others fresh, or commitment age >= T_c + D; the B0 sonar checks."""
    no_claim = z3.And(*[z3.Not(r.blocking()) for peer, r in L.rx.items() if peer != ignore_claim])
    fresh_all = z3.And(*[r.fresh(p.T_f) for r in L.rx.values()])
    return z3.And(L.sonar_ok, no_claim, z3.Or(z3.And(fresh_all, L.ca >= p.T_c), L.ca >= p.T_c + p.D))


def admit_old(L: LocalView, p: Prm, rule_d: bool = True):
    """Former rule: no claim held, a claim of a drone dead by rule D released; the B0 sonar checks; nothing else from the channel."""
    def released(a, r):
        if not rule_d:
            return z3.BoolVal(False)
        hears = z3.Or(*[q.silence < p.T_silent for k, q in L.rx.items() if k != a]) if len(L.rx) > 1 else z3.BoolVal(False)
        return z3.And(r.silence >= p.T_silent, hears, r.outside)
    return z3.And(L.sonar_ok, *[z3.Not(z3.And(r.blocking(), z3.Not(released(a, r)))) for a, r in L.rx.items()])


def free_symbols(e) -> set:
    seen, out, stack = set(), set(), [e]
    while stack:
        x = stack.pop()
        if x.get_id() in seen:
            continue
        seen.add(x.get_id())
        if z3.is_const(x) and x.decl().kind() == z3.Z3_OP_UNINTERPRETED:
            out.add(x.decl().name())
        stack.extend(x.children())
    return out


# ----------------------------------------------------------------------------------------------------------- timeline model

class Drone:
    """True timeline of one drone of the violating pair (or a bystander): latest commitment c, admission t, slot, clock offset."""

    def __init__(self, name: str, p: Prm):
        self.n = name
        self.c, self.t = z3.Real(f"c_{name}"), z3.Real(f"t_{name}")
        self.o, self.delta = z3.Real(f"o_{name}"), z3.Real(f"delta_{name}")
        self.cons = [self.o >= 0, self.o < p.P, self.delta >= -p.skew, self.delta <= p.skew]


def channel(tag: str, snd: Drone, rcv: Drone, r: RxView, t_dec, p: Prm, end, loss_k: Optional[int] = None,
            snd_claims=True, silent_from=None):
    """The latest report of ``snd`` held by ``rcv`` at its decision instant t_dec, by the channel law; returns constraints and the
    true emission instant.  ``end``: the sender is committed (truthful claims) on [c, end]; ``loss_k``: at most k consecutive losses
    on this link; ``silent_from``: no packet of the sender emitted at or after this instant is delivered (failed transmitter)."""
    m, lam, rho = z3.Int(f"m_{tag}"), z3.Real(f"lam_{tag}"), z3.Real(f"rho_{tag}")
    heard = z3.Real(f"heard_{tag}")
    t_tx = m * p.P + snd.o - snd.delta
    cons = [lam >= p.lat_min, lam <= p.lat_max, rho >= 0, rho <= p.dt_rep,
            z3.Implies(z3.Not(r.none), z3.And(t_tx + lam <= t_dec, r.age == (t_dec + rcv.delta) - (m * p.P + snd.o))),
            heard <= t_dec, r.silence == t_dec - heard, z3.Implies(z3.Not(r.none), heard >= t_tx + lam)]
    if snd_claims:
        cons.append(z3.Implies(z3.And(z3.Not(r.none), t_tx - rho > snd.c, t_tx <= end), r.claim))
    if silent_from is not None:
        cons += [z3.Implies(z3.Not(r.none), t_tx < silent_from), heard < silent_from + p.lat_max]
    if loss_k is not None:
        m_last = z3.Int(f"mlast_{tag}")
        cons += [z3.Not(r.none), m_last * p.P + snd.o - snd.delta + p.lat_max <= t_dec,
                 (m_last + 1) * p.P + snd.o - snd.delta + p.lat_max > t_dec, m >= m_last - loss_k,
                 heard >= t_tx + lam]
    return cons, t_tx


def timeline(p: Prm, rule: str = "new", n: int = 2, loss: Dict[str, Optional[int]] = None, mutual: Optional[bool] = None,
             extra=None, kin: float = 0.0, rule_d: bool = True, witness: bool = False):
    """Constraints of a P2 violation by the pair (f, s): c_f < t_f <= t_s <= tau, c_s < t_s, tau <= t_f + T_cross (Pi2), both
    admissions taken by ``rule`` on the local views, the channel law on every link.  ``n`` = 3 adds a bystander k whose reports
    (arbitrary content: its own state is unconstrained) reach f and s.  ``loss``: link -> k bound ("fs" = f's reports at s).
    ``mutual``: True / False restricts to (not) mutually unheard pairs.  ``witness``: no violation - f's passage is verified at
    end_f (its later reports are PASSED) and s is admitted after it (non-vacuity of the rule).  Returns (constraints, views, drones)."""
    loss = loss or {}
    f, s = Drone("f", p), Drone("s", p)
    tau = z3.Real("tau")
    names = ["f", "s"] + (["k"] if n == 3 else [])
    k = Drone("k", p) if n == 3 else None
    Lf, Ls = LocalView("f", [x for x in names if x != "f"]), LocalView("s", [x for x in names if x != "s"])
    cons = f.cons + s.cons + (k.cons if k else [])
    end_f = tau
    if witness:
        end_f = z3.Real("end_f")
        cons += [f.c < f.t, f.t < end_f, end_f + 0.5 <= s.t, s.c < s.t, tau == s.t]
    else:
        cons += [f.c < f.t, f.t <= s.t, s.t <= tau, s.c < s.t, tau <= f.t + p.T_cross]
    cons += [Lf.ca == f.t - f.c, Ls.ca == s.t - s.c, Lf.ca >= kin, Ls.ca >= kin]
    if k is not None:
        cons += [k.o != f.o, k.o != s.o]
    cons.append(f.o != s.o)
    c1, _ = channel("sf", s, f, Lf.rx["s"], f.t, p, tau, loss.get("sf"), silent_from=(extra or {}).get("s_silent_from"))
    c2, t2 = channel("fs", f, s, Ls.rx["f"], s.t, p, end_f, loss.get("fs"), silent_from=(extra or {}).get("f_silent_from"))
    cons += c1 + c2
    if witness:
        cons.append(z3.Implies(t2 - z3.Real("rho_fs") > end_f, z3.Not(Ls.rx["f"].claim)))
    if k is not None:
        cons += channel("kf", k, f, Lf.rx["k"], f.t, p, tau, loss.get("kf"), snd_claims=False)[0]
        cons += channel("ks", k, s, Ls.rx["k"], s.t, p, tau, loss.get("ks"), snd_claims=False)[0]
    if rule == "rank_waiver":
        cons += [admit_new(Lf, p, ignore_claim="s"), admit_new(Ls, p)]
    else:
        adm = admit_new if rule == "new" else (lambda L, q: admit_old(L, q, rule_d))
        cons += [adm(Lf, p), adm(Ls, p)]
    if mutual is not None:
        mu = z3.And(z3.Not(Lf.rx["s"].fresh(p.T_f)), z3.Not(Ls.rx["f"].fresh(p.T_f)))
        cons.append(mu if mutual else z3.Not(mu))
    return cons, (Lf, Ls), (f, s, tau)


def locality(p: Prm) -> dict:
    """Free symbols of the admission decisions (both rules, 3 drones) against the local views."""
    bad = {}
    for rule in ("new", "old"):
        for b in ("f", "s", "k"):
            L = LocalView(b, [x for x in ("f", "s", "k") if x != b])
            e = admit_new(L, p) if rule == "new" else admit_old(L, p)
            extra = {x for x in free_symbols(e) if not x.startswith(f"L{b}_")}
            if extra:
                bad[f"{rule}:{b}"] = sorted(extra)
    return bad


def concession_progress(p: Prm, obstruction: Optional[str] = None):
    """Two committed vehicles, ordered by the shared queue: left/top vehicle retains the other's already received claim.

    The lower-priority vehicle must hear a fresh claim while calm and robustly upstream. After it withdraws, packets
    reflect the sender's actual claim at their emission. The higher-priority vehicle stays committed and may admit only
    with its retained claim cleared and all the remaining runtime guards true. One abstract tick includes delivery and
    local processing. Delivery deadlines and readiness are hypotheses, parameterized using the channel timing budget.
    """
    dt = 0.5
    delivery = math.ceil((3 * p.P + p.lat_max + p.dt_rep + 2 * p.skew) / dt)
    age = math.ceil(p.T_c / dt)
    max_report_age = math.floor(p.T_f / dt)
    degraded_age = math.ceil((p.T_c + p.D) / dt)
    K = max(2 * delivery, age) + 2
    claims = z3.Bools([f"claims_{k}" for k in range(K + 1)])
    held = z3.Bools([f"held_{k}" for k in range(K + 1)])
    admitted = z3.Bools([f"admitted_{k}" for k in range(K + 1)])
    fresh = z3.Bools([f"fresh_{k}" for k in range(K)])
    receive = z3.Bools([f"receive_{k}" for k in range(K)])
    ready = z3.Bools([f"ready_{k}" for k in range(K)])
    can_concede = z3.BoolVal(obstruction != "infeasible_concession")
    cons = [claims[0], held[0], z3.Not(admitted[0])]
    cons += [z3.Not(z3.Or(*fresh))] if obstruction == "claim_lost" else [z3.Or(*fresh[:delivery])]
    cons += [z3.Not(z3.Or(*ready))] if obstruction == "infeasible_admission" else [ready[-1]]
    if obstruction == "withdrawal_lost":
        cons.append(z3.Not(z3.Or(*receive)))
    else:
        for k in range(1, delivery + 1):
            cons.append(z3.Implies(z3.And(claims[k - 1], z3.Not(claims[k])), z3.Or(*receive[k:k + delivery])))
        cons.append(z3.Or(*receive[max(0, K - 1 - max_report_age):K - 1]))
    for k in range(K):
        fresh_report = z3.Or(*receive[max(0, k - max_report_age):k])
        wait_elapsed = z3.Or(z3.And(k >= age, fresh_report), k >= degraded_age)
        cons += [claims[k + 1] == z3.And(claims[k], z3.Not(z3.And(fresh[k], can_concede))),
                 held[k + 1] == z3.If(receive[k], claims[k], held[k]),
                 admitted[k + 1] == z3.Or(admitted[k], z3.And(wait_elapsed, ready[k], z3.Not(held[k])))]
    return cons, admitted[-1], K * dt


def monotone(p: Prm, rule: str = "new") -> list:
    """admit(L') and not admit(L) where L' is L with every link at least as blocking and at most as fresh (an added claim, a staler or
    missing report, a longer silence) and the same own state: expect UNSAT for a monotone rule."""
    L, L2 = LocalView("i", ["a", "b"]), LocalView("i2", ["a", "b"])
    cons = [L.ca == L2.ca, L.sonar_ok == L2.sonar_ok]
    for a in ("a", "b"):
        r, r2 = L.rx[a], L2.rx[a]
        cons += [z3.Implies(r.blocking(), r2.blocking()), z3.Implies(r2.fresh(p.T_f), r.fresh(p.T_f)),
                 z3.Implies(z3.Not(r2.none), z3.And(z3.Not(r.none), r2.age >= r.age)), r2.silence >= r.silence,
                 r2.outside == r.outside, r.silence >= 0]
    adm = admit_new if rule == "new" else admit_old
    return cons + [adm(L2, p), z3.Not(adm(L, p))]


def exclusion(p: Prm, rule: str = "new") -> list:
    """status_excluded (perception/acoustic_gate.py) on a symbolic report: excluded although no FRESH positive report (not engaged or of
    another gate, age <= status_fresh_s) and no PASSED: expect UNSAT (rule "old": the dead branch of rule D)."""
    ph = z3.Int("ph")
    age, sil, outside, other_gate, hears = z3.Real("age"), z3.Real("silence"), z3.Bool("outside"), z3.Bool("other_gate"), z3.Bool("hears")
    fresh = age <= B2.acoustic.status_fresh_s
    positive = z3.Or(z3.And(fresh, z3.Or(other_gate, ph == 0)), z3.And(z3.Not(other_gate), ph == 3))
    dead = z3.And(sil >= p.T_silent, hears, outside)
    excl = z3.Or(positive, dead) if rule == "old" else positive
    return [ph >= 0, ph <= 3, age >= 0, sil >= 0, excl, z3.Not(positive)]


def deployed_exclusion(n_cases: int = 300, seed: int = 41) -> dict:
    """Randomized on the deployed class: a silent drone (last report of any phase, then silence up to 400 s while another drone is
    heard, last fix anywhere) is never excluded, never loses its claim, and never makes a corridor echo a phantom."""
    import dataclasses

    from holo_fleet.mission import GateSpec, MissionPlan, Path as MPath
    from holo_fleet.perception import payload as pl
    from holo_fleet.perception.acoustic import AcousticTracker
    from holo_fleet.perception.gate_perception import GatePerception

    rng = np.random.default_rng(seed)
    g = GateSpec("G", np.array([0., 0., -5.]), np.array([1., 0., 0.]), np.array([0., -1., 0.]), 2., 2.)
    bad = 0
    for _ in range(n_cases):
        tr = AcousticTracker(B2)
        tr.set_identities({0: 0, 1: 1, 2: 2})
        slots = abreast_slots(3)
        qs = queue_line(slots, B2.gate)
        path = MPath(np.array([[-30., 0.], [30., 0.]]), -5.)
        P0 = np.array([qs, 0., -5.])
        plan = MissionPlan(drone_id="d0", slot_index=0, slots=slots, path=path, gates=[g], structures=[g], launch_position=P0,
                           launch_yaw_deg=0., static_rank=0, s_end=40., queue_lateral=0., queue_s=qs)
        gp = GatePerception(plan, B2, acoustic=tr)
        ae = gp.acoustic_exit
        ae.others = [1, 2]
        ae.g, ae.gate_idx = g, 0
        phase = int(rng.integers(0, 3))
        q = np.array([rng.uniform(-30.0, 2.0), rng.uniform(-6.0, 6.0), -5.0])
        d = q - P0
        r = float(np.linalg.norm(d))
        t0 = 1.0
        fr = pl.frame_index(t0 - 0.05 - r / B2.beacon.sound_speed, B2.beacon.period_s)
        tr.update(t0, [(1, t0 - 0.05, r, math.atan2(d[1], d[0]), math.asin(d[2] / r), pl.encode(fr, q, 0.5, 0, phase))], P0, 0., 0.5)
        t = t0 + float(rng.uniform(B2.acoustic.status_fresh_s + 0.5, 400.0))
        tr.heard[2] = t - 0.5
        if ae.status_excluded(1, t) or (phase in (pl.GP_COMMITTED, pl.GP_ADMITTED) and ae.claims(t) != {1}):
            bad += 1
        if ae.no_fleet_drone(t, g, P0, 0.0, [abs(qs) - 1.0]):
            bad += 1
    return {"cases": n_cases, "violations": bad}


# ----------------------------------------------------------------------------------------------------------- step model

def step_model(cfg: FleetConfig, p: Prm, rule: str = "new", H_s: float = 100.0, dt: float = 0.5, loss_k: Optional[int] = 2,
               blackout: bool = False):
    """Two drones, explicit local state, bounded horizon.  Returns (constraints, locality violations).  True state of drone i:
    s (gate axis), com, adm; local state: commitment age, latest received report (phase, emission step), quiet timer and latch fed by
    free sonar observations.  Emissions: drone 0 at even steps, drone 1 at odd steps (TDMA, 2 drones, P = 1 s = 2 steps); a delivered
    packet is received at the next step (latency <= 0.4 s < dt).  Pi1: an unadmitted centre stays before the CR; Pi2: an admitted
    drone advances by [v_cross_min dt, v_bound dt] per step until it is beyond the CR."""
    G = cfg.gate
    vb = speed_bound(cfg)
    K = int(round(H_s / dt))
    q_s = queue_line(abreast_slots(2), G, cfg.perc)
    s_zone = G.stage2_hold_s - RELOC_ZONE_M - G.nav_err_m
    s_cap = G.stage2_hold_s + G.hold_err_m + G.nav_err_m
    n_f = int(math.floor(p.T_f / dt + 1e-9))
    n_c = int(math.ceil(p.T_c / dt - 1e-9))
    n_d = int(math.ceil((p.T_c + p.D) / dt - 1e-9))
    cons, loc_bad = [], []
    X = {}
    for i in (0, 1):
        X[i] = {"s": [z3.Real(f"s{i}_{k}") for k in range(K + 1)], "com": [z3.Bool(f"com{i}_{k}") for k in range(K + 1)],
                "adm": [z3.Bool(f"adm{i}_{k}") for k in range(K + 1)], "done": [z3.Bool(f"done{i}_{k}") for k in range(K + 1)],
                "Lca": [z3.Int(f"L{i}_ca_{k}") for k in range(K + 1)], "Lph": [z3.Int(f"L{i}_rxph_{k}") for k in range(K + 1)],
                "Ltx": [z3.Int(f"L{i}_rxtx_{k}") for k in range(K + 1)], "Lobs": [z3.Bool(f"L{i}_obs_{k}") for k in range(K + 1)],
                "Lbusy": [z3.Bool(f"L{i}_busy_{k}") for k in range(K + 1)], "Lq": [z3.Int(f"L{i}_quiet_{k}") for k in range(K + 1)],
                "Lrel": [z3.Bool(f"L{i}_rel_{k}") for k in range(K + 1)],
                "go": [z3.Bool(f"go{i}_{k}") for k in range(K + 1)], "quit": [z3.Bool(f"quit{i}_{k}") for k in range(K + 1)]}
        x = X[i]
        cons += [x["s"][0] <= q_s, x["s"][0] >= q_s - 3.0, z3.Not(x["com"][0]), z3.Not(x["adm"][0]), z3.Not(x["done"][0]),
                 x["Lca"][0] == 0, x["Lph"][0] == -1, x["Ltx"][0] == -1000, z3.Not(x["Lbusy"][0]), x["Lq"][0] == 0]
    deliv = {(j, i): [z3.Bool(f"del{j}{i}_{k}") for k in range(K + 1)] for j in (0, 1) for i in (0, 1) if i != j}
    for (j, i), dl in deliv.items():
        em = [k for k in range(K + 1) if k % 2 == j]
        if blackout:
            cons += [z3.Not(dl[k]) for k in em]
        elif loss_k is not None:
            for a in range(len(em) - loss_k):
                cons.append(z3.Or(*[dl[em[a + b]] for b in range(loss_k + 1)]))
    s_out = G.cr_s_max + 3.0

    def phase(j, k):
        x = X[j]
        return z3.If(x["done"][k], 3, z3.If(x["adm"][k], 2, z3.If(x["com"][k], 1, 0)))

    def claim_held(i, k):
        return z3.Or(X[i]["Lph"][k] == 1, X[i]["Lph"][k] == 2)

    def local_admit(i, k):
        """Admission decision of drone i at step k from its local state only."""
        x = X[i]
        fresh = z3.And(x["Ltx"][k] >= 0, k - x["Ltx"][k] <= n_f)
        quiet = x["Lq"][k] >= 1
        if rule == "new":
            return z3.And(quiet, z3.Not(claim_held(i, k)), z3.Or(z3.And(fresh, x["Lca"][k] >= n_c), x["Lca"][k] >= n_d))
        return z3.And(quiet, z3.Not(claim_held(i, k)))

    def local_commit(i, k):
        """Commitment guard of drone i (local): latch FREE (no echo, no claim held); precedence is a free choice (go)."""
        x = X[i]
        return z3.And(z3.Not(x["Lbusy"][k]), z3.Not(claim_held(i, k)))

    for k in range(K):
        for i in (0, 1):
            j = 1 - i
            x = X[i]
            s0, s1 = x["s"][k], x["s"][k + 1]
            dec, dcom = local_admit(i, k), local_commit(i, k)
            loc_bad += [n for n in free_symbols(z3.And(dec, dcom)) if not n.startswith(f"L{i}_")]
            commit = z3.And(x["go"][k], z3.Not(x["com"][k]), z3.Not(x["done"][k]), dcom, s0 >= q_s - G.queue_tol - G.nav_err_m)
            admit = z3.And(x["go"][k], x["com"][k], z3.Not(x["adm"][k]), s0 >= s_zone, dec)
            giveup = z3.And(x["quit"][k], x["com"][k], z3.Not(x["adm"][k]))
            fin = z3.And(x["adm"][k], s0 > s_out)
            cons += [x["done"][k + 1] == z3.Or(x["done"][k], fin),
                     x["com"][k + 1] == z3.And(z3.Not(fin), z3.Or(commit, z3.And(x["com"][k], z3.Not(giveup)))),
                     x["adm"][k + 1] == z3.And(z3.Not(fin), z3.Or(admit, x["adm"][k])),
                     x["Lca"][k + 1] == z3.If(x["com"][k + 1], z3.If(x["com"][k], x["Lca"][k] + 1, 0), 0)]
            cons.append(z3.If(x["done"][k + 1], z3.And(s1 >= s0, s1 <= s0 + vb * dt),
                              z3.If(x["adm"][k + 1], z3.And(s1 - s0 >= G.v_cross_min * dt, s1 - s0 <= vb * dt),
                                    z3.And(s1 - s0 >= -vb * dt, s1 - s0 <= vb * dt, s1 < G.cr_s_min,
                                           z3.Implies(x["com"][k + 1], s1 <= s_cap)))))
            got = z3.And(deliv[(j, i)][k], z3.BoolVal(k % 2 == j))
            cons += [x["Lph"][k + 1] == z3.If(got, phase(j, k), x["Lph"][k]), x["Ltx"][k + 1] == z3.If(got, k, x["Ltx"][k]),
                     x["Lbusy"][k + 1] == z3.Or(x["Lobs"][k + 1], claim_held(i, k + 1), z3.And(x["Lbusy"][k], z3.Not(x["Lrel"][k]))),
                     x["Lq"][k + 1] == z3.If(x["Lobs"][k + 1], 0, x["Lq"][k] + 1)]
    viol = [z3.And(X[0]["s"][k] >= G.cr_s_min, X[0]["s"][k] <= G.cr_s_max, X[1]["s"][k] >= G.cr_s_min, X[1]["s"][k] <= G.cr_s_max)
            for k in range(K + 1)]
    cons.append(z3.Or(*viol))
    return cons, sorted(set(loc_bad)), K


# ----------------------------------------------------------------------------------------------------------- geometry

def cr_observability(cfg: FleetConfig = DEFAULT, step_s: float = 0.1) -> dict:
    """Which s-slices of the CR are CERTAIN in the FRONT sonar of a drone in the admission zone (every observer pose: estimate in the
    zone, true position within nav_err_m along and across, heading +-heading_tol; every hull centre of the slice: l, dz in {-1, 0, 1}
    x the CR half sizes): unexplained (outside the structure window grown by its tolerances), unoccluded by a bar, within r_far.
    Also the nominal case (observer exactly at the hold)."""
    from check_stage2 import _segment_hits_box

    from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
    from holo_fleet.perception.sonar_geometry import AXES, HULL_HALF, MOUNTS
    from holo_fleet.perception.sonar_processing import EchoClassifier

    G, pc = cfg.gate, cfg.perc
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    clf = EchoClassifier(gate_bars=list(g.bars), cfg=cfg)
    yaw0 = math.atan2(g.axis[1], g.axis[0])
    hull = np.array([[x, y, z] for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1) if (x, y, z) != (0, 0, 0)], float) * HULL_HALF
    half = math.radians(pc.sonar.half_angle_deg)

    def R(y):
        return np.array([[math.cos(y), -math.sin(y), 0.0], [math.sin(y), math.cos(y), 0.0], [0.0, 0.0, 1.0]])

    son0 = np.array([G.stage2_hold_s + float(MOUNTS["FRONT"][0]), 0.0, 0.0])
    r_far = max(np.linalg.norm(son0 - G.cr_center() - G.cr_half() * np.array([a, b, c])) for a in (-1, 1) for b in (-1, 1)
                for c in (-1, 1)) + cfg.env.eps_range_near

    def seen(obs, herr, tgt):
        Ro = R(yaw0 + math.radians(herr))
        p = g.from_gate_frame(obs)
        son, ax = p + Ro @ MOUNTS["FRONT"], Ro @ AXES["FRONT"]
        sw = clf.structure_window(son, ax)
        surf = g.from_gate_frame(tgt) + hull @ R(yaw0).T
        d = surf - son
        r = np.linalg.norm(d, axis=1)
        inc = np.arccos(np.clip((d @ ax) / np.maximum(r, 1e-9), -1, 1)) <= half
        for x, rr in sorted(zip(surf[inc], r[inc]), key=lambda z: z[1]):
            if any(_segment_hits_box(son, x, b) for b in g.bars):
                continue
            return (sw is None or rr < sw[0] - pc.structure_tol_m or rr > sw[1] + pc.structure_tol_far_m) and rr <= r_far
        return False

    e = G.nav_err_m
    lat = G.hold_lateral_allow + e
    poses = [(s, l, herr) for s in (G.stage2_hold_s - RELOC_ZONE_M - e, G.stage2_hold_s - e, G.stage2_hold_s,
                                    G.stage2_hold_s + G.hold_err_m + e) for l in (-lat, 0.0, lat) for herr in (-G.heading_tol_deg, G.heading_tol_deg)]
    slices = np.round(np.arange(G.cr_s_min, G.cr_s_max + 1e-9, step_s), 3)
    tgts = [(l, dz) for l in (-G.cr_half_width, 0.0, G.cr_half_width) for dz in (-G.cr_half_height, 0.0, G.cr_half_height)]
    certain = [float(sv) for sv in slices if all(seen([ps, pl_, 0.0], h, [sv, l, dz]) for ps, pl_, h in poses for l, dz in tgts)]
    nominal = [float(sv) for sv in slices if all(seen([G.stage2_hold_s, 0.0, 0.0], 0.0, [sv, l, dz]) for l, dz in tgts)]
    return {"certain_slices": certain, "nominal_slices": nominal, "n_slices": len(slices), "poses": len(poses), "r_far": round(r_far, 2)}


# ----------------------------------------------------------------------------------------------------------- run

def run(cfg: FleetConfig = B2, verbose: bool = True, step: bool = True) -> Report:
    rep = Report("b2_comm")
    p = params(cfg)
    kin2 = t_kin(cfg, 2)
    Tm = f"T_f = {p.T_f} s, T_c = {p.T_c} s, D = t_cross_max = {p.T_cross:.2f} s, P = {p.P} s, latency [{p.lat_min}, {p.lat_max}] s, skew {p.skew} s"
    ok_t = p.T_c > p.T_f + 2 * p.skew + p.dt_rep and p.T_f >= 3 * p.P + p.lat_max and p.D >= p.T_cross
    rep.add(CheckResult("B2C-T derived timing: T_c > T_f + 2 skew + dt, T_f >= 3 P + latency_max (2 consecutive losses stay fresh), D >= t_cross_max",
                        "parameter arithmetic", "holo_fleet/config.py (AcousticConfig, GateRule.t_cross_max)", "holds",
                        verdict="holds" if ok_t else "violated", passed=ok_t, note=Tm), verbose)

    bad = locality(p)
    rep.add(CheckResult("B2C-L structural: the admission decision of every drone (both rules, 3 drones) reads only its own local view",
                        "free symbols(admit_i) subset of L_i (commitment age, received reports, silence, own sonar)", ENC, "holds",
                        verdict="holds" if not bad else "violated", passed=not bad,
                        note="no true state of another drone (c, t, tau, its slot or clock) in a decision" if not bad else str(bad)), verbose)

    note_e = "every loss pattern, latency, TDMA slot and clock offset; sonar free (not used)"
    rep.add(check("B2C-E1 P2 for every message pattern unless the pair is mutually unheard at both admissions (2 drones)",
                  "admit_new(f) and admit_new(s) and Pi1 and Pi2 and channel law and not mutual -> not both in the CR", ENC,
                  timeline(p, "new", mutual=False)[0], note=note_e + "; " + Tm), verbose)
    k_max = int(math.ceil((p.T_c + p.D - p.lat_max - p.dt_rep) / p.P - 1e-9)) - 2
    rep.add(check("B2C-E2 P2 with at most 2 consecutive losses per link (2 drones): never degraded, no sonar or B0 argument",
                  "loss <= 2 consecutive on both links -> not both in the CR", ENC,
                  timeline(p, "new", loss={"fs": 2, "sf": 2})[0], note="the latest report is then at most 3 P + latency_max old"), verbose)
    rep.add(check(f"B2C-E2k P2 with at most {k_max} consecutive losses per link (2 drones): a degraded pair still hears each other's claims",
                  f"loss <= {k_max} consecutive on both links -> not both in the CR", ENC,
                  timeline(p, "new", loss={"fs": k_max, "sf": k_max})[0],
                  note=f"k_max = ceil((T_c + D - latency_max - dt) / P) - 2 (the largest k with (k + 1) P + latency_max + dt < T_c + D): a "
                       f"mutually degraded pair has both commitments older than T_c + D, so the latest report is a claim"), verbose)
    rep.add(check("B2C-Ea concurrent commitment before hearing each other (|c_f - c_s| <= P), at most 2 consecutive losses: P2",
                  "case (a)", ENC, timeline(p, "new", loss={"fs": 2, "sf": 2})[0] + [z3.Abs(z3.Real("c_f") - z3.Real("c_s")) <= p.P]), verbose)
    stat = [z3.Real("t_f") - z3.Real("c_f") >= 30.0]
    rep.add(check("B2C-Eb one drone stationary (committed and holding >= 30 s), its messages never reaching the other: P2",
                  "case (b): f -> s link silent from f's commitment, s -> f at most 2 consecutive losses", ENC,
                  timeline(p, "new", loss={"sf": 2}, extra={"f_silent_from": z3.Real("c_f")})[0] + stat), verbose)
    rep.add(check("B2C-Eb2 one drone stationary, the other committed without its messages, the stationary admitted second: P2",
                  "case (b), roles swapped: the stationary drone s is admitted after f", ENC,
                  timeline(p, "new", loss={"fs": 2}, extra={"s_silent_from": z3.Real("c_s")})[0]
                  + [z3.Real("t_s") - z3.Real("c_s") >= 30.0]), verbose)
    rep.add(check("B2C-Ee live drone with a failed transmitter (3 drones, the receiver hears a third drone, silence >= t_silent_gate_s): P2",
                  "case (e): f silent from before its commitment at s; s -> f at most 2 consecutive losses", ENC,
                  timeline(p, "new", n=3, loss={"sf": 2}, extra={"f_silent_from": z3.Real("c_f") - p.T_silent})[0]), verbose)
    rep.add(check("B2C-E3 3 drones: a bystander's reports (any content, any freshness) cannot create a violation unless the pair is mutually unheard",
                  "as B2C-E1 with a third drone k", ENC, timeline(p, "new", n=3, mutual=False)[0]), verbose)
    rep.add(check("B2C-Ed partial loss and delay: one link lossless, the other arbitrary (2 drones): P2",
                  "case (d)", ENC, timeline(p, "new", loss={"sf": 0})[0]), verbose)
    rep.add(check("B2C-EL declared limit: a mutually unheard pair (total blackout of the pair, case (c)) - P2 rests on the B0 mechanisms only",
                  "expect counterexample: both degraded admissions after D, the first still in the CR", ENC,
                  timeline(p, "new", mutual=True)[0], expect="sat",
                  note="B0 then: queue precedence and occupancy latch (formal/check_mutex.py M2/M3, idealised positions); the stage-2 sonar "
                       "is not certified (B2C-G1)"), verbose)
    rep.add(check(f"B2C-Es scope: {k_max + 1} consecutive losses on both links allow the mutual degradation of B2C-EL",
                  "expect counterexample (the end of the loss bound of B2C-E2k)", ENC,
                  timeline(p, "new", loss={"fs": k_max + 1, "sf": k_max + 1})[0], expect="sat"), verbose)
    rep.add(check("B2C-O0 former controller on a lossless channel (physical queue-to-zone time, n = 2): P2",
                  "old rule, loss 0 on both links", ENC, timeline(p, "old", loss={"fs": 0, "sf": 0}, kin=kin2)[0],
                  note=f"commit to admission >= {kin2:.2f} s > P + latency_max: the claim always arrives first"), verbose)
    rep.add(check("B2C-Oa mutation: former controller, concurrent commitment before hearing each other, 2 consecutive losses: violates P2",
                  "expect counterexample (case (a))", ENC,
                  timeline(p, "old", loss={"fs": 2, "sf": 2}, kin=kin2)[0] + [z3.Abs(z3.Real("c_f") - z3.Real("c_s")) <= p.P],
                  expect="sat"), verbose)
    rep.add(check("B2C-Ob mutation: former controller, a stationary committed drone whose messages never reach the other: violates P2",
                  "expect counterexample (case (b))", ENC,
                  timeline(p, "old", loss={"sf": 2}, kin=kin2, extra={"f_silent_from": z3.Real("c_f")})[0] + stat, expect="sat"), verbose)
    rep.add(check("B2C-Oc mutation: former controller, total blackout: violates P2", "expect counterexample (case (c))", ENC,
                  timeline(p, "old", kin=kin2, extra={"f_silent_from": z3.Real("c_f") - 1e3, "s_silent_from": z3.Real("c_s") - 1e3})[0],
                  expect="sat"), verbose)
    rep.add(check("B2C-Oe mutation: former controller, rule D releases the claim of a live drone with a failed transmitter: violates P2",
                  "expect counterexample (case (e), 3 drones): the claim was received, then t_silent_gate_s of silence", ENC,
                  timeline(p, "old", n=3, loss={"sf": 2}, kin=kin2, extra={"f_silent_from": z3.Real("c_f") + 1.5})[0]
                  + [z3.Bool("Ls_rxf_claim"), z3.Not(z3.Bool("Ls_rxf_none"))], expect="sat"), verbose)
    rep.add(check("B2C-Mt mutation: T_c = T_f (no margin for the clock skew and the report age): violates P2",
                  "expect counterexample", ENC, timeline(replace(p, T_c=p.T_f), "new", mutual=False)[0], expect="sat"), verbose)
    rep.add(check("B2C-Mw mutation: degraded wait D = t_cross_max - 2 s: violates P2", "expect counterexample", ENC,
                  timeline(replace(p, D=p.T_cross - 2.0), "new", mutual=False)[0], expect="sat"), verbose)
    cons, (Lf, _Ls), (f, s, tau) = timeline(p, "rank_waiver", loss={"sf": 0}, mutual=False,
                                            extra={"f_silent_from": z3.Real("c_f") - 1.0})
    rep.add(check("B2C-Mr mutation: a higher-priority vehicle ignoring a lower-priority claim can violate P2",
                  "expect overlap after concurrent commitment with loss of the higher-priority transmitter", ENC,
                  cons + [f.c == s.c, Lf.rx["s"].blocking(), tau < f.t + p.T_cross], expect="sat"), verbose)
    cons, (Lf, Ls), _ = timeline(p, "new", loss={"fs": 0, "sf": 0}, witness=True)
    rep.add(check("B2C-W1 witness: both drones admitted by the fresh branch, each within T_c + 0.5 s of its commitment, the second after the first passed",
                  "expect SAT (non-vacuity)", ENC, cons + [Ls.ca <= p.T_c + 0.5, Lf.ca <= p.T_c + 0.5], expect="sat"), verbose)
    cons, (Lf, Ls), _ = timeline(p, "new", witness=True)
    rep.add(check("B2C-W2 witness: a degraded admission (no fresh report) after T_c + D",
                  "expect SAT (non-vacuity)", ENC, cons + [z3.Not(Ls.rx["f"].fresh(p.T_f))], expect="sat"), verbose)

    rep.add(check("B2C-Mo monotone: an added claim, a staler or missing report, a longer silence never enables an admission",
                  "admit_new(L') and not admit_new(L), L' at least as blocking and at most as fresh", ENC, monotone(p, "new")), verbose)
    rep.add(check("B2C-Mom mutation: the former rule is not monotone (rule D: a longer silence enables an admission)",
                  "expect counterexample", ENC, monotone(p, "old"), expect="sat"), verbose)
    rep.add(check("B2C-D an exclusion needs a fresh positive report or PASSED: silence never excludes (status_excluded)",
                  "excluded and not (fresh positive or PASSED)", ENC, exclusion(p, "new")), verbose)
    rep.add(check("B2C-Dm mutation: rule D excludes a silent drone", "expect counterexample", ENC, exclusion(p, "old"), expect="sat"), verbose)
    dx = deployed_exclusion()
    rep.add(CheckResult("B2C-Dc deployed class: a silent drone (any last phase, up to 400 s, another drone heard) is never excluded, keeps its claim, "
                        "never makes a corridor echo a phantom", "randomized on AcousticExit", "perception/acoustic_gate.py", "holds",
                        verdict="holds" if dx["violations"] == 0 else "violated", passed=dx["violations"] == 0, note=str(dx)), verbose)

    if step:
        cons, lb, K = step_model(cfg, p, "new", loss_k=2)
        rep.add(CheckResult("B2C-Ls structural (step model): the admission decision of drone i at every step reads only its local state",
                            "free symbols of local_admit(i, k) subset of L_i", ENC, "holds", verdict="holds" if not lb else "violated",
                            passed=not lb, note=f"{K} steps x 2 drones" if not lb else str(lb[:10])), verbose)
        rep.add(check("B2C-S1 step model, 100 s at dt = 0.5 s, at most 2 consecutive losses per link: never both in the CR",
                      "explicit local state, free sonar, give-up at any time, Pi1, Pi2", ENC, cons, timeout_ms=1800000,
                      note="BMC from the queue (both uncommitted, nothing received)"), verbose)
        cons, _, _ = step_model(cfg, p, "old", loss_k=2)
        rep.add(check("B2C-S1o mutation: step model, former rule, at most 2 consecutive losses: violates P2", "expect counterexample", ENC,
                      cons, expect="sat", timeout_ms=1800000), verbose)
        cons, _, _ = step_model(cfg, p, "new", loss_k=None, blackout=True)
        rep.add(check("B2C-S1b declared limit: step model, total blackout - P2 rests on B0", "expect counterexample", ENC, cons,
                      expect="sat", timeout_ms=1800000), verbose)

    cons, admitted, horizon = concession_progress(p)
    note = ("Two concurrent unadmitted claimants; the winner holds the loser's claim and remains committed without any other "
            "claimant. The loser is calm, unadmitted and robustly upstream when it receives the winner's fresh claim. The "
            "winner receives the truthful withdrawal. At the deadline all remaining local admission guards are true, including "
            "healthy, quiet FRONT, validated localization, no possible occupant and a fresh report. The deadline is measured "
            "from concurrent commitment with the winner already holding the loser's claim. Once admitted, clearance additionally "
            "requires Pi2. This model does not establish the delivery or physical premises.")
    rep.add(check(f"B2C-S2 conditional admission within {horizon:.1f} s in the abstract two-claimant model",
                  "bounded delivery and feasible concession/admission -> higher-priority vehicle admitted by the horizon",
                  ENC, cons + [z3.Not(admitted)], note=note), verbose)
    rep.add(check("B2C-S2w witness: a timely concession and admission are possible in the bounded model",
                  "the delivery and feasibility premises are satisfiable", ENC, cons + [admitted], expect="sat"), verbose)
    for obstruction, premise in (("claim_lost", "a fresh higher-priority claim never arrives"),
                                 ("withdrawal_lost", "the withdrawal is never received"),
                                 ("infeasible_concession", "an upstream calm concession remains infeasible"),
                                 ("infeasible_admission", "the remaining admission guards never become true")):
        cons, admitted, _ = concession_progress(p, obstruction)
        rep.add(check(f"B2C-S2 declared limit: the bounded admission can stall when {premise}",
                      "expect a run without an admission in the horizon", ENC, cons + [z3.Not(admitted)], expect="sat"), verbose)

    geo = cr_observability(cfg)
    rep.add(CheckResult("B2C-G1 CR slices certain in the FRONT sonar from the admission zone under the declared navigation error (reported)",
                        "every observer pose and every hull centre of the slice: unexplained, unoccluded, within r_far",
                        "perception/sonar_processing.py structure window + bars as occluders (numeric)", "reported", verdict="reported",
                        passed=True, note=f"certain: {geo['certain_slices'] or 'none'} of {geo['n_slices']} slices over {geo['poses']} poses; "
                                          f"nominal pose (no error): {geo['nominal_slices']}; r_far {geo['r_far']} m"), verbose)
    G = cfg.gate
    q2 = queue_line(abreast_slots(2), G, cfg.perc)
    nat = ((G.stage2_hold_s - RELOC_ZONE_M) - (q2 + G.queue_tol)) / G.v_pass
    rep.add(CheckResult("B2C-Lv liveness cost (reported)", "added admission delays and bounded give-up", "control/flows.py", "reported",
                        verdict="reported", passed=True,
                        note=f"fresh branch: T_c = {p.T_c} s after the commitment, below the natural queue-to-zone time {nat:.1f} s at v_pass "
                             f"(n = 2): no slowdown; degraded branch (some drone not freshly heard): T_c + D = {p.T_c + p.D:.1f} s after "
                             f"the commitment, the hold give-up timer (t_occ_max + t_clear) starts then; a claim of a silent drone never "
                             f"expires: a drone that failed holding a claim keeps the gate closed; outside the early concession "
                             f"window the timed give-up and retry still apply"), verbose)
    return rep


if __name__ == "__main__":
    r = run(step="--no-step" not in sys.argv)
    r.save()
    print(f"b2_comm: {sum(x.passed for x in r.results)}/{len(r.results)} as expected")
    sys.exit(0 if r.ok else 1)
