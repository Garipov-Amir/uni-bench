"""Common trajectory format (R5): the only interface between planners and evaluator.

For each robot a trajectory is a sequence of waypoints ``(x, y, psi, t)``;
every segment between consecutive waypoints carries a label

* ``service`` -- tool active, contributes to coverage,
* ``transit`` -- tool inactive, contributes to time/distance only,
* ``turn``    -- in-place heading change (tool inactive unless declared),
* ``wait``    -- standing still for a declared duration.

The evaluator ignores planner time stamps and re-times every trajectory with
the motion model of the instance (:func:`retime`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .util import wrap_angle

SERVICE, TRANSIT, TURN, WAIT = "service", "transit", "turn", "wait"
LABELS = (SERVICE, TRANSIT, TURN, WAIT)
_LABEL_CODE = {lab: k for k, lab in enumerate(LABELS)}
EPS = 1e-9


@dataclass
class Trajectory:
    robot_id: str
    xy: np.ndarray  # (K+1, 2)
    psi: np.ndarray  # (K+1,)
    labels: np.ndarray  # (K,) str
    active: np.ndarray  # (K,) bool -- tool active during the segment
    wait: np.ndarray  # (K,) declared durations for wait segments
    t: np.ndarray | None = None  # (K+1,) time stamps (planner-supplied or re-timed)

    def __post_init__(self):
        self.xy = np.asarray(self.xy, dtype=float).reshape(-1, 2)
        self.psi = np.asarray(self.psi, dtype=float).reshape(-1)
        self.labels = np.asarray(self.labels, dtype="<U7").reshape(-1)
        self.active = np.asarray(self.active, dtype=bool).reshape(-1)
        self.wait = np.asarray(self.wait, dtype=float).reshape(-1)
        if self.t is not None:
            self.t = np.asarray(self.t, dtype=float).reshape(-1)

    # --------------------------------------------------------------- basics
    @property
    def K(self) -> int:
        return len(self.labels)

    def seg_lengths(self) -> np.ndarray:
        if self.K == 0:
            return np.zeros(0)
        return np.hypot(*np.diff(self.xy, axis=0).T)

    def heading_changes(self) -> np.ndarray:
        if self.K == 0:
            return np.zeros(0)
        return np.abs(wrap_angle(np.diff(self.psi)))

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if self.t is not None and len(self.t) else 0.0

    @property
    def end_time(self) -> float:
        return float(self.t[-1]) if self.t is not None and len(self.t) else 0.0

    def length(self, label: str | None = None) -> float:
        L = self.seg_lengths()
        if label is None:
            return float(L.sum())
        return float(L[self.labels == label].sum())

    def position_at(self, times: np.ndarray) -> np.ndarray:
        """Positions at the given times (robot parked before start / after end)."""
        if self.t is None:
            raise ValueError("trajectory is not timed")
        t = self.t
        # np.interp needs strictly increasing xp: add tiny ramp for zero-duration segments
        tt = t + np.arange(len(t)) * 1e-12
        x = np.interp(times, tt, self.xy[:, 0])
        y = np.interp(times, tt, self.xy[:, 1])
        return np.c_[x, y]

    # --------------------------------------------------------------- io
    def to_dict(self) -> dict:
        d = {
            "robot_id": self.robot_id,
            "xy": np.round(self.xy, 9).tolist(),
            "psi": np.round(self.psi, 9).tolist(),
            "labels": "".join(str(_LABEL_CODE[lab]) for lab in self.labels),
            "active": "".join("1" if a else "0" for a in self.active),
        }
        if np.any(self.wait):
            d["wait"] = self.wait.tolist()
        if self.t is not None:
            d["t"] = np.round(self.t, 9).tolist()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Trajectory":
        labels = d["labels"]
        if isinstance(labels, str):
            labels = [LABELS[int(c)] for c in labels]
        active = d.get("active")
        if isinstance(active, str):
            active = [c == "1" for c in active]
        elif active is None:
            active = [lab == SERVICE for lab in labels]
        K = len(labels)
        xy = np.asarray(d["xy"], dtype=float).reshape(-1, 2)
        psi = d.get("psi")
        if psi is None:
            psi = headings_from_xy(xy)
        return cls(robot_id=d["robot_id"], xy=xy, psi=psi, labels=labels, active=active,
                   wait=d.get("wait", np.zeros(K)), t=d.get("t"))


@dataclass
class TrajectorySet:
    trajectories: dict[str, Trajectory] = field(default_factory=dict)
    metadata: dict = field(default_factory=dict)

    def __getitem__(self, rid: str) -> Trajectory:
        return self.trajectories[rid]

    def __iter__(self):
        return iter(self.trajectories.values())

    def __len__(self):
        return len(self.trajectories)

    def add(self, traj: Trajectory):
        self.trajectories[traj.robot_id] = traj

    def to_dict(self) -> dict:
        return {"trajectories": [t.to_dict() for t in self], "metadata": self.metadata}

    @classmethod
    def from_dict(cls, d: dict) -> "TrajectorySet":
        ts = cls(metadata=d.get("metadata", {}))
        for t in d["trajectories"]:
            ts.add(Trajectory.from_dict(t))
        return ts


def headings_from_xy(xy: np.ndarray) -> np.ndarray:
    xy = np.asarray(xy, dtype=float)
    psi = np.zeros(len(xy))
    if len(xy) < 2:
        return psi
    d = np.diff(xy, axis=0)
    h = np.arctan2(d[:, 1], d[:, 0])
    moving = np.hypot(d[:, 0], d[:, 1]) > EPS
    last = 0.0
    for k in range(len(d)):
        if moving[k]:
            last = h[k]
        psi[k + 1] = last
    first = np.flatnonzero(moving)
    psi[0] = h[first[0]] if len(first) else 0.0
    return psi


# --------------------------------------------------------------------- builder
class TrajectoryBuilder:
    """Incremental construction with automatic insertion of turn segments.

    Motion segments always point in the direction of travel; whenever the
    heading changes an in-place ``turn`` segment is inserted, with the tool
    active if ``tool_active_in_turns`` is declared (e.g. by a planner manifest).
    """

    def __init__(self, robot_id: str, x: float, y: float, psi: float = 0.0,
                 tool_active_in_turns: bool = False, angle_tol: float = 1e-6):
        self.robot_id = robot_id
        self.xy = [(float(x), float(y))]
        self.psi = [float(psi)]
        self.labels: list[str] = []
        self.active: list[bool] = []
        self.wait: list[float] = []
        self.turn_active = tool_active_in_turns
        self.angle_tol = angle_tol
        self._last_motion_active = False

    @property
    def pos(self) -> tuple[float, float]:
        return self.xy[-1]

    @property
    def heading(self) -> float:
        return self.psi[-1]

    def _append(self, xy, psi, label, active, wait=0.0):
        self.xy.append((float(xy[0]), float(xy[1])))
        self.psi.append(float(psi))
        self.labels.append(label)
        self.active.append(bool(active))
        self.wait.append(float(wait))

    def turn_to(self, psi: float, active: bool | None = None):
        d = float(wrap_angle(psi - self.heading))
        if abs(d) <= self.angle_tol:
            return self
        if active is None:
            active = self.turn_active and self._last_motion_active
        self._append(self.pos, self.heading + d, TURN, active)
        return self

    def move_to(self, x: float, y: float, label: str = SERVICE, turn_active: bool | None = None):
        if label not in (SERVICE, TRANSIT):
            raise ValueError("move_to expects a service or transit label")
        dx, dy = x - self.pos[0], y - self.pos[1]
        if math.hypot(dx, dy) <= EPS:
            return self
        h = math.atan2(dy, dx)
        if turn_active is None:
            turn_active = self.turn_active and label == SERVICE and self._last_motion_active
        self.turn_to(h, active=turn_active)
        self._append((x, y), self.heading, label, label == SERVICE)
        self._last_motion_active = label == SERVICE
        return self

    def follow(self, pts, label: str = SERVICE, skip_first: bool = True):
        pts = np.asarray(pts, dtype=float)
        for p in pts[1:] if skip_first else pts:
            self.move_to(p[0], p[1], label)
        return self

    def follow_curve(self, samples, label: str = SERVICE):
        """Follow a sampled curve (x, y, psi) whose heading varies smoothly."""
        samples = np.asarray(samples, dtype=float)
        for x, y, _ in samples[1:]:
            self.move_to(x, y, label, turn_active=label == SERVICE)
        return self

    def wait_for(self, duration: float):
        if duration > 0:
            self._append(self.pos, self.heading, WAIT, False, duration)
        return self

    def build(self) -> Trajectory:
        return Trajectory(self.robot_id, np.array(self.xy), np.array(self.psi), np.array(self.labels, dtype="<U7"),
                          np.array(self.active, dtype=bool), np.array(self.wait, dtype=float))


def from_labelled_polyline(robot_id: str, pts, seg_labels, start_psi: float | None = None,
                           tool_active_in_turns: bool = False) -> Trajectory:
    """Build a trajectory from a polyline with one service/transit label per segment."""
    pts = np.asarray(pts, dtype=float)
    psi0 = start_psi if start_psi is not None else headings_from_xy(pts)[0]
    b = TrajectoryBuilder(robot_id, pts[0, 0], pts[0, 1], psi0, tool_active_in_turns)
    for p, lab in zip(pts[1:], seg_labels):
        b.move_to(p[0], p[1], lab)
    return b.build()


# ---------------------------------------------------------------------- timing
def retime(traj: Trajectory, speed: float, turn_cost: float) -> Trajectory:
    """Re-time a trajectory with the instance motion model.

    motion: length / v; turn: |dpsi| * turn_cost (+ length / v for arcs);
    wait: declared duration.  Planner time stamps are discarded.
    """
    L = traj.seg_lengths()
    dpsi = traj.heading_changes()
    dt = L / speed
    is_turn = traj.labels == TURN
    dt[is_turn] = dpsi[is_turn] * turn_cost + L[is_turn] / speed
    is_wait = traj.labels == WAIT
    dt[is_wait] = traj.wait[is_wait]
    t = np.r_[0.0, np.cumsum(dt)]
    return Trajectory(traj.robot_id, traj.xy.copy(), traj.psi.copy(), traj.labels.copy(), traj.active.copy(),
                      traj.wait.copy(), t)


def retime_set(ts: TrajectorySet, team) -> TrajectorySet:
    robots = {r.id: r for r in team}
    out = TrajectorySet(metadata=dict(ts.metadata))
    for tr in ts:
        r = robots[tr.robot_id]
        out.add(retime(tr, r.speed, r.turn_cost))
    return out


# ------------------------------------------------------------------ validation
@dataclass
class Validity:
    valid: bool = True
    reasons: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def fail(self, msg: str):
        self.valid = False
        self.reasons.append(msg)


def validate_set(ts: TrajectorySet, inst, pos_tol: float, check_turn_radius: bool = False) -> Validity:
    """Format validity of a trajectory set for an instance (Section: common trajectory format)."""
    v = Validity()
    ids = {r.id for r in inst.team}
    got = set(ts.trajectories)
    if got - ids:
        v.fail(f"unknown robots {sorted(got - ids)}")
    if ids - got:
        v.fail(f"missing trajectories for {sorted(ids - got)}")
    for r in inst.team:
        if r.id not in ts.trajectories:
            continue
        tr = ts[r.id]
        K = tr.K
        if not (len(tr.xy) == K + 1 and len(tr.psi) == K + 1 and len(tr.active) == K and len(tr.wait) == K):
            v.fail(f"{r.id}: inconsistent array lengths")
            continue
        if not np.all(np.isfinite(tr.xy)) or not np.all(np.isfinite(tr.psi)):
            v.fail(f"{r.id}: non-finite coordinates")
            continue
        bad = set(tr.labels.tolist()) - set(LABELS)
        if bad:
            v.fail(f"{r.id}: unknown labels {sorted(bad)}")
            continue
        sx, sy, spsi = r.start
        if math.hypot(tr.xy[0, 0] - sx, tr.xy[0, 1] - sy) > pos_tol:
            v.fail(f"{r.id}: does not start at its start pose")
        L = tr.seg_lengths()
        stationary = np.isin(tr.labels, (TURN, WAIT))
        if np.any(L[stationary] > pos_tol):
            v.fail(f"{r.id}: discontinuity (a turn or wait segment moves the robot)")
        if np.any(tr.wait[tr.labels == WAIT] < 0):
            v.fail(f"{r.id}: negative wait duration")
        moving = ~stationary & (L > EPS)
        if np.any(moving):
            d = np.diff(tr.xy, axis=0)[moving]
            h = np.arctan2(d[:, 1], d[:, 0])
            err = np.abs(wrap_angle(h - tr.psi[:-1][moving])) + np.abs(wrap_angle(h - tr.psi[1:][moving]))
            if np.any(err > 1e-3):
                v.warnings.append(f"{r.id}: motion heading differs from waypoint heading")
        if inst.task.return_to_depot:
            depot = r.depot if r.depot is not None else r.start[:2]
            if math.hypot(tr.xy[-1, 0] - depot[0], tr.xy[-1, 1] - depot[1]) > pos_tol:
                v.fail(f"{r.id}: does not end at its depot")
        if tr.t is not None:
            if len(tr.t) != K + 1 or np.any(np.diff(tr.t) < -1e-9):
                v.warnings.append(f"{r.id}: planner time stamps are not monotone (ignored; re-timed)")
        if check_turn_radius and r.min_turn_radius:
            ok, worst = turning_radius_ok(tr, r.min_turn_radius)
            if not ok:
                v.fail(f"{r.id}: violates the minimum turning radius (local radius {worst:.3g} < {r.min_turn_radius:.3g})")
    return v


def turning_radius_ok(tr: Trajectory, r_min: float, rel_tol: float = 0.02):
    """Fillet criterion for a minimum turning radius.

    Heading changes of consecutive in-place turn segments are merged into
    corner turns theta at the vertices of the travelled polyline.  The polyline
    is drivable with radius r_min if every corner can be replaced by a circular
    fillet of radius r_min inside its adjacent segments:

        r_min * (tan(theta_a / 2) + tan(theta_b / 2)) <= L   for every segment,

    where theta_a, theta_b are the corner turns at the segment's ends (a turn at
    the start pose or at the end must fit into the first / last segment).
    Corners separated by chords shorter than 5 % of r_min are merged first.
    Densely sampled arcs of radius >= r_min satisfy it; a reversal never does.
    Returns (ok, largest fillet radius that fits everywhere).
    """
    L = tr.seg_lengths()
    dpsi = tr.heading_changes()
    moving = L > EPS
    seg_len, corner = [], [0.0]
    for k in range(tr.K):
        if moving[k]:
            seg_len.append(L[k])
            corner.append(0.0)
        else:
            corner[-1] += dpsi[k]
    if not seg_len:
        return (not np.any(dpsi > 1e-6)), (math.inf if not np.any(dpsi > 1e-6) else 0.0)
    # chords much shorter than the radius (uneven sampling of a curve) do not separate two
    # corners: merge such corners and hand the chord length to the neighbouring segments
    merge_len = 0.05 * r_min
    j = 0
    while j < len(seg_len) and len(seg_len) > 1:
        if seg_len[j] < merge_len:
            half = seg_len[j] / 2
            if j > 0:
                seg_len[j - 1] += half
            if j + 1 < len(seg_len):
                seg_len[j + 1] += half
            corner[j] += corner[j + 1]
            del corner[j + 1]
            del seg_len[j]
        else:
            j += 1
    theta = np.asarray(corner)
    if np.any(theta[1:-1] >= math.pi - 1e-6):
        return False, 0.0
    # the first corner (turn before any motion) and the last one (after all motion) have no room
    tan_half = np.tan(np.minimum(theta, math.pi - 1e-6) / 2)
    need = tan_half[:-1] + tan_half[1:]
    seg_len = np.asarray(seg_len)
    with np.errstate(divide="ignore"):
        radius = np.where(need > 1e-12, seg_len / need, math.inf)
    worst = float(radius.min())
    return bool(worst >= r_min * (1 - rel_tol)), worst
