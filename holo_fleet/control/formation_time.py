"""Local formation time: reduced progress while an expected neighbour is missing (DI-31).

The formation reference of every drone is the shared FormationClock of the mission plan.  When a drone is
lost, the others slow the formation down - they do not stop - so that it catches up faster, without any
communication.  Slowing the vehicle alone would not do: its slot would keep moving at the planned speed and the
difference would be a formation error.  So each drone evaluates the clock at its OWN formation time

    t_f = t - tau,     d t_f / d t = rate,     d tau / d t = 1 - rate,

and lowers ``rate`` while, from its own observation, the formation is incomplete (an expected neighbour
MISSING and not declared vacant): its slot then advances at v_wait instead of the planned speed v_plan
(rate = v_wait / v_plan) and the slot-tracking feed-forward follows (Flows.formation).  The reference speed
rate * v_plan changes by at most ``a_wait`` per second, into and out of the reduced progress (a ramp, no
step).  The reduced progress lasts at most ``t_rejoin`` of moving formation time per episode: the bound after
which a waiting drone declares the slot vacant and continues at the planned speed (DEGRADED_FORMATION).
``tau`` is kept afterwards: the formation stays behind the plan by tau seconds, it does not speed up to
recover the schedule.

Who slows down, each from its own observation only:

* a drone keeping its slot while a neighbour is missing: FORMATION_WAIT_REJOIN;
* a drone out of its slot (FORMATION_RECOVERY, FAILSAFE, avoidance) that misses its expected neighbours.
  The missing relation is symmetric (range between two drones against their template distance), so this is
  the drone the others are waiting for: it applies the same rule to its own reference, which keeps its slot
  level with the slowed fleet (otherwise it would rejoin a slot ahead of the fleet by the accumulated lag);
  its budget is the same t_rejoin, so it stops slowing when the others time out.  Once started, its episode
  lasts until it is back near its own slot (the controller keeps ``incomplete`` while it is out of formation
  keeping and farther than e_ok from its slot): seen from behind its neighbours fit the range gate before it
  reaches its slot, while they end their episode only when they see it in its slot - otherwise it would stop
  slowing seconds before them and its reference would run ahead of the fleet;
* the same lane decides between a drone and the neighbour directly ahead of it (``front_residual``: its
  range minus the template distance, seen by the own sonar).  Drones notice an absence at different times,
  or not at all - range-only sensing: an echo of another neighbour can stand in for the missing one - and a
  rear drone at the survey speed behind a slowed one would close on it (the warning band of a same-lane pair
  begins about 0.23 m closer than the 3.5 m spacing).  So:

  - the drone ahead is closer than planned (by more than ``follow_on_m``, until below ``follow_off_m``): slow
    down too, with the faster ramp ``a_follow`` - the slowdown spreads backward by normal perception;
  - while keeping its slot, a drone that is slowing keeps slowing until the drone ahead pulls away (farther
    than planned by ``follow_on_m``), even after its own reason ended, and does not start slowing while the
    drone ahead is pulling away: in a lane the slowdown ends front to back, and a drone that notices the
    absence after the drone ahead already resumed does not fall behind it.  Out of its slot (recovery) the
    residual measures its own displacement, so only the first rule applies.

  Other pairs (abreast, diagonal) only shear or stretch, which the sonar progress consensus (sigma,
  Flows.formation) absorbs.

In the mutex zone (gate flow) the rate returns to nominal and the episode timer is frozen: the gate protocol
has its own timing and the clock holds at the rendezvous.  Planned holds stay as planned (v_plan = 0: nothing
to slow down).  Onboard data only, nothing shared between drones: tau is a private state of each drone.
"""

from __future__ import annotations

from typing import Optional

from holo_fleet.config import DEFAULT, FleetConfig


class FormationTime:
    def __init__(self, clock, cfg: FleetConfig = DEFAULT):
        self.clock = clock
        self.cfg = cfg
        self.tau = 0.0                 # delay of the own formation reference behind the plan [s]
        self.rate = 1.0                # d t_f / d t
        self.episode_t = 0.0           # moving formation time of the current incomplete episode [s]
        self.following = False         # the drone ahead on the same lane is closer than planned (hysteresis)
        self.ahead_away = False        # ... farther than planned: it is not slowing (hysteresis)
        self.reduced = False           # reduced progress requested at the last update

    def now(self, t: float) -> float:
        """Own formation time at mission time t."""
        return t - self.tau

    def plan_speed(self, t: float, dt: float = 0.1) -> float:
        """Planned speed of the formation reference at the own formation time [m/s]."""
        if self.clock is None:
            return 0.0
        tf = self.now(t)
        return max(0.0, (self.clock.s(tf + dt) - self.clock.s(tf)) / dt)

    def speed(self, t: float, dt: float = 0.1) -> float:
        """Speed of the own formation reference [m/s]."""
        return self.rate * self.plan_speed(t, dt)

    def update(self, t: float, dt: float, incomplete: bool, mutex: bool = False,
               front_residual: Optional[float] = None, keeping: bool = True) -> None:
        """Advance by one control step.  ``incomplete``: an expected neighbour is missing and not vacant (own
        perception); ``mutex``: the drone follows the gate flow (mutex zone or committed); ``front_residual``: range
        residual of the neighbour directly ahead on the same lane, None when there is none or it is not seen;
        ``keeping``: the drone is in a formation-keeping mode (on its slot)."""
        fr = self.cfg.form
        v_plan = self.plan_speed(t, dt)
        moving = v_plan > 1e-6
        if not incomplete:
            self.episode_t = 0.0
        elif moving and not mutex:
            self.episode_t += dt
        own = incomplete and self.episode_t < fr.t_rejoin
        r = front_residual
        self.following = r is not None and (r <= -fr.follow_on_m or (self.following and r < -fr.follow_off_m))
        self.ahead_away = r is not None and (r >= fr.follow_on_m or (self.ahead_away and r > fr.follow_off_m))
        if mutex:
            reduced = False
        elif r is not None and keeping:                       # same lane: keep the spacing to the drone ahead
            reduced = self.following or (not self.ahead_away and (own or self.reduced))
        else:
            reduced = own or self.following
        self.reduced = reduced
        if not moving:
            self.rate = 1.0                                   # planned hold: nothing to slow down
        else:
            target = min(1.0, fr.v_wait / v_plan) if self.reduced else 1.0
            a_down = fr.a_follow if self.following else fr.a_wait
            a_up = fr.a_follow if self.ahead_away else fr.a_wait
            self.rate += max(-a_down * dt / v_plan, min(a_up * dt / v_plan, target - self.rate))
        self.tau += (1.0 - self.rate) * dt

    def record(self, t: float, dt: float = 0.1) -> dict:
        return {"tau": round(self.tau, 2), "v_ref": round(self.speed(t, dt), 3), "reduced": self.reduced,
                "follow": "slower" if self.following else ("away" if self.ahead_away else ""),
                "wait_t": round(self.episode_t, 1)}
