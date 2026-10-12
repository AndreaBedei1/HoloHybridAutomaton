"""Leaderless, communication-free UUV fleet built from identical local hybrid automata.

Package layout (import boundaries are enforced by tests/test_isolation.py):

* ``holo_fleet.ha``          hybrid-automaton specification + runtime (no simulator access)
* ``holo_fleet.perception``  onboard-sensor perception (no simulator access)
* ``holo_fleet.control``     per-drone controller = perception + automaton + low level
* ``holo_fleet.sim``         HoloOcean wrapper (owns ground truth, filters sensors)
* ``holo_fleet.referee``     online validator (the only reader of ground truth, never feeds a drone)
* ``holo_fleet.ui``          fleet view for the demos (reads controller records and the referee)

There is no communication channel between drones.
"""

__version__ = "0.1.0"
