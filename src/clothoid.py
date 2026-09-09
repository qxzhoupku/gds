"""True Euler-spiral (clothoid) geometry for curvature-continuous routes.

Why not a circular arc?  The guided mode of a straight waveguide and that of a
curved one are different fields: bending shifts the mode outwards.  Where a
straight meets an arc the curvature jumps from 0 to 1/R in zero length, and
that step has to be absorbed by an abrupt field mismatch, which converts power
into higher-order modes.  An *Euler* bend ramps curvature **linearly with arc
length**, so the mode deforms continuously and no single interface carries a
discontinuity.

This module is pure geometry: no ``gdstk``, no layers, no ports.  It answers
two questions — *which segments join these two ports* (:func:`plan_route`) and
*where are the vertices* (:func:`plan_points`).  :func:`src.place.route_clothoid`
wraps the pair up as the ``clothoid`` router.

The bend primitive
------------------
One bend turns through a signed angle ``turn`` with peak curvature exactly
``1/R``.  Curvature ramps 0 -> 1/R over the first half and 1/R -> 0 over the
second, so it is **zero at both ends** — which is what lets a bend be butted
straight against a straight section, or against another bend, without
reintroducing a curvature step.  With ``t = |turn|`` and arc length ``s``::

    Lc = p*t*R        length of one clothoid ramp
    La = (1-p)*t*R    length of the optional circular middle
    L  = t*(1+p)*R    total arc length
    kappa(s)  = (1/R) * s/Lc          on the entry ramp
              = (1/R)                 on the middle
              = (1/R) * (L-s)/Lc      on the exit ramp
    phi(s)    = s^2/(2*Lc*R)                      on the entry ramp
              = p*t/2 + (s-Lc)/R                  on the middle
              = t - (L-s)^2/(2*Lc*R)              on the exit ramp

``p`` is the fraction of the turn contributed by the two ramps.  ``p = 1`` is
the pure double clothoid — curvature is a triangle wave that touches ``1/R``
at a single point — and is the default, because it is the most gradual
curvature ramp available for a given ``R`` and turn.  Its price is length and
footprint: it is exactly ``1 + p`` times as long as the circular arc of the
same radius and turn, and a 90 degree pure Euler bend needs the corner box of
a 1.87 R circular bend.  Lowering ``p`` buys that footprint back by spending
part of the turn on a constant-curvature middle; curvature stays continuous
for any ``p > 0``.  ``p = 0`` would be a plain arc and is rejected.

Because the curvature profile is symmetric about the midpoint, the bend's
entry-to-exit chord always lies on the **bisector** of the turn, exactly as a
circular arc's does::

    offset(turn) = R * chord(|turn|, p) * (cos(turn/2), sin(turn/2))

so the whole shape reduces to one scalar function :func:`unit_chord`, for
which there is a closed form in the Fresnel integrals (see its docstring).
That is what makes the solver below cheap enough to search against.

Composing a route
-----------------
The route is assembled from straights and bends, and because every bend ends
with zero curvature the composition is curvature-continuous throughout.
:func:`plan_route` solves three shapes and prefers them in this order:

``corner``   ``straight - bend(Delta) - straight``.  One bend, both straights
             axis-aligned with a port.  The natural L-bend, and the preferred
             answer whenever it fits.
``sbend``    ``straight - bend(alpha) - bend(Delta-alpha) - straight`` with
             equal outer straights.  Handles a lateral offset at any turn,
             including the ``Delta = 0`` S-bend, which ``corner`` cannot express.
``uturn``    ``bend(alpha) - straight - bend(Delta-alpha)``.  The general
             fallback; unlike ``sbend`` it stays well conditioned when the
             ports face the same way and ``Delta`` approaches 180 degrees.

Fewest bends first, then the form that keeps straights parallel to the ports.
The segment lists above are the general forms; at the radius actually chosen a
``corner`` usually loses one straight and a two-bend shape can lose its whole
turn into one bend, so read ``EulerPlan.segments`` rather than assuming.

``Rmin`` is a floor, not the operating point.  Both drivers of mode conversion
fall with radius — peak curvature as ``1/R``, curvature rate as ``1/R^2`` — so
each shape is solved for the **largest** radius its geometry admits and the
straights absorb what is left.  For ``corner`` that has a closed form; for the
two-bend shapes it is a one-dimensional maximisation over the turn split.
Within a shape the gentlest bend wins, then the shortest route.

Preference alone is not enough, because a shape can fit and still be absurd.
``corner`` divides by ``sin(Delta)``, so as the ports approach antiparallel its
straights run away while both stay non-negative and the endpoint residual stays
exactly zero.  The two-bend solves have poles of their own, where the straight
and the radius blow up together.  Growing the radius removes most of this by
itself — a corner that ran to kilometres at a pinned radius becomes a modest
bend with short straights — but every candidate is still screened before the
preference is applied, see :data:`_MAX_DETOUR`.
"""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass
from typing import Callable, List, Sequence, Tuple

from .design import DesignError

# Segment kinds in a plan.
STRAIGHT = "straight"
BEND = "bend"

#: Default clothoid fraction: the pure double clothoid.
DEFAULT_P = 1.0

#: Default chord tolerance for the emitted polyline, in um.  0.001 um is the
#: database grid, so the polygon is faithful to the curve at the resolution the
#: GDS can actually express.
DEFAULT_TOLERANCE = 0.001

# An offset below this many um counts as zero: 1e-9 um is a femtometre, six
# orders below the 1 nm database grid, so it can only be transform round-off.
POS_TOL = 1e-9
# Likewise for an angle, in radians.
ANGLE_TOL = 1e-12

# ``corner`` needs a component of the exit straight across A's axis to absorb
# the lateral offset; below this the 1/sin(Delta) solve is ill conditioned.
_SIN_TOL = 1e-9
# ``sbend``'s two end straights are antiparallel as |Delta| -> pi, so they
# cannot position the route at all; |c1|^2 measures how far from that it is.
# The guard has to be explicit rather than a test for a zero denominator: at
# |Delta| = pi exactly, IEEE gives c1 = (0.0, 1.22e-16) — 1+cos(pi) is exactly
# zero but sin(pi) is not — so the denominator comes out around 1e-17 and a
# radius gets built entirely out of round-off.
_C1_TOL = 1e-12
# Guarding the *equations* against these degeneracies is not enough: the
# ill-conditioned solve stays feasible (both straights non-negative, endpoint
# residual exactly zero) while the answer runs away.  A ``corner`` a microradian
# short of 180 degrees, between ports 500 um apart at Rmin = 80 with the target
# abeam, solves to 559 metres of waveguide (a nanoradian short, 559 km).  So candidate plans are also judged on
# shape, not just on whether the equations closed.
#
# The symptom is always the same: the straight sections double back, growing
# without bound while the ports stay put.  So a plan is rejected when its
# straights add up to more than this many times the port separation.  That is
# a test on the shape itself, which keeps it stable: a legitimate route's
# straights scale with the separation, whether it runs 5 um or 5 mm.
#
# Comparing candidate shapes on relative length instead does not work, for two
# reasons.  A ratio test is relative to whatever else was found, so it cannot
# flag a runaway that is the only solution.  And a diagonal is always shorter
# than an L — an ordinary 90 degree corner is already 1.26x the shortest route
# between the same two ports — so a ratio tight enough to be useful starts
# turning plain corners into U-turns, which is not what a layout wants.
_MAX_DETOUR = 3.0
# Backstop for anything the detour test misses: this many times the port
# separation plus the circumference of a full turn at Rmin, which leaves room
# for a genuine U-turn between adjacent ports.
_LENGTH_BUDGET = 20.0
# Bend samples used only for the self-intersection screen.
_CROSS_SAMPLES = 32

# Grid for the one-dimensional maximisation of the radius over the turn split,
# how many of its local maxima get refined, and the golden-section iterations
# spent on each.  R(alpha) is not unimodal and has poles, so a scan wide enough
# to separate the peaks comes first and hill climbing only happens inside one.
# Radius ceiling, as a multiple of the port separation — see _radius_ceiling.
_MAX_RADIUS_FACTOR = 10.0
# How far a route may sweep outside the rectangle its two ports span, as a
# multiple of their separation — see _bulge.  This is not slack: over sampled
# poses the accepted corners reach 0.855 and both two-bend shapes run right up
# to the cap, which is what rejects the millimetre-scale double-back loops a
# free radius makes reachable (24.5 mm between ports 224 um apart, with the
# screen off).  It does turn 12.8% of sampled poses into a DesignError, but not
# one of those is a near miss: they would have bulged 2.53x the separation at
# the very least, and none of them route at a pinned radius either.
_MAX_BULGE = 1.5

# Machine epsilon, for tolerances that have to track a solve's conditioning.
_EPS = 2.220446049250313e-16

# The grid is only a backstop now that the exact candidates are enumerated,
# so it can be coarse.
_SPLIT_SCAN = 241
_SPLIT_PEAKS = 4
_SPLIT_REFINE = 50
# Radii offered per shape, from the gentlest the geometry allows down to Rmin.
_LADDER_RUNGS = 5

_MIN_BEND_SAMPLES = 24
_MAX_BEND_SAMPLES = 6000
# Quadrature panels per emitted bend vertex.  Even, so composite Simpson
# applies to each block.  At 16 the position error is ~1e-12 R at p = 1, where
# the curvature kink lands on a panel boundary, and a few 1e-9 R at other p,
# where it straddles one — either way orders below the chord tolerance that
# actually decides fidelity, and the endpoint is exact regardless because
# bend_points() takes it from the closed form.
_SUB_PANELS = 16

_HALF_PI = math.pi / 2.0


# ---------------------------------------------------------------------------
# The bend primitive
# ---------------------------------------------------------------------------

def _fresnel(z: float) -> Tuple[float, float]:
    """Fresnel integrals ``C(z), S(z)`` with the ``pi/2`` normalisation::

        C(z) = int_0^z cos(pi u^2 / 2) du
        S(z) = int_0^z sin(pi u^2 / 2) du

    Evaluated from the Maclaurin series.  :func:`unit_chord` only ever asks
    for ``0 <= z <= 1`` — that is what ``p <= 1`` and ``|turn| <= pi`` bound it
    to — and there the series reaches double precision in about a dozen terms,
    so no asymptotic branch is needed.
    """
    if z <= 0.0:
        return 0.0, 0.0

    z4 = z ** 4
    ratio = -(_HALF_PI ** 2) * z4

    # C: sum_k (-1)^k (pi/2)^{2k} z^{4k+1} / ((2k)! (4k+1))
    c = 0.0
    term = z
    k = 0
    while True:
        c += term / (4 * k + 1)
        k += 1
        term *= ratio / ((2 * k - 1) * (2 * k))
        if abs(term) < 1e-19 or k > 60:
            break

    # S: sum_k (-1)^k (pi/2)^{2k+1} z^{4k+3} / ((2k+1)! (4k+3))
    s = 0.0
    term = _HALF_PI * z ** 3
    k = 0
    while True:
        s += term / (4 * k + 3)
        k += 1
        term *= ratio / ((2 * k) * (2 * k + 1))
        if abs(term) < 1e-19 or k > 60:
            break

    return c, s


def unit_chord(turn_abs: float, p: float = DEFAULT_P) -> float:
    """Entry-to-exit chord length of the bend at ``R = 1``.

    Derivation.  Take the first half of the bend, which ends with heading
    ``t/2``.  Its ramp part is a Fresnel spiral: substituting
    ``s = sqrt(pi*Lc) * z`` turns ``phi = s^2/(2 Lc)`` into ``pi z^2 / 2``, so
    the ramp ends at ``sqrt(pi Lc) * (C(z1), S(z1))`` with ``z1 = sqrt(Lc/pi)``,
    and the circular part that follows integrates in closed form.  Mirroring
    the second half onto the first (its heading is ``t - phi`` of the mirror
    point) gives the full endpoint, whose components collapse to::

        chord = 2*(x_half*cos(t/2) + y_half*sin(t/2))
              = 2*(x_ramp*cos(t/2) + y_ramp*sin(t/2)) + 2*sin((1-p)*t/2)

    The same mirroring shows the endpoint direction is exactly ``t/2``, which
    is the bisector identity :func:`bend_offset` relies on.  For ``p = 0`` this
    reduces to ``2*sin(t/2)``, the chord of a unit circular arc, as it must.
    """
    t = abs(float(turn_abs))
    if t < ANGLE_TOL:
        return 0.0

    half = 0.5 * t
    lc = p * t                       # one ramp, at R = 1
    chord = 2.0 * math.sin((1.0 - p) * half)
    if lc > 0.0:
        scale = math.sqrt(math.pi * lc)
        c, s = _fresnel(lc / scale)  # lc/scale == sqrt(lc/pi)
        chord += 2.0 * scale * (c * math.cos(half) + s * math.sin(half))
    return chord


def bend_length(turn: float, radius: float, p: float = DEFAULT_P) -> float:
    """Arc length of a bend: ``R * |turn| * (1 + p)``."""
    return radius * abs(turn) * (1.0 + p)


def bend_offset(turn: float, radius: float,
                p: float = DEFAULT_P) -> Tuple[float, float]:
    """Entry-to-exit offset of a bend, in the frame it enters.

    The bend enters at the origin heading ``+x`` and leaves heading ``turn``.
    By the bisector identity the offset is the chord laid along ``turn/2``.
    """
    q = radius * unit_chord(abs(turn), p)
    half = 0.5 * turn
    return (q * math.cos(half), q * math.sin(half))


def bend_samples(turn: float, radius: float, p: float,
                 tolerance: float = DEFAULT_TOLERANCE) -> int:
    """Vertex count for one bend, from a chord-sagitta budget.

    Sampling uniformly in arc length, the largest heading step is at the peak
    curvature, ``dphi = L/(n*R)``, and a chord subtending ``dphi`` on radius
    ``R`` sits ``R*(1 - cos(dphi/2))`` inside the true curve.  Everywhere else
    the radius is larger and the step smaller, so bounding the sagitta at the
    midpoint bounds it everywhere; the count below is what holds it under
    *tolerance*, and a tighter bend gets proportionally more vertices for the
    same fidelity.

    The count is clamped to ``[24, 6000]``.  The floor only ever oversamples.
    The ceiling is reached at roughly ``n = pi*sqrt(R/(2*tolerance))``, i.e.
    never at the default tolerance for a radius up to about 5 mm — asking for
    a tolerance far below the 1 nm database grid on a very large radius is the
    one case where the emitted chord error can exceed what was requested.
    """
    t = abs(turn)
    if t < ANGLE_TOL:
        return 0
    tol = min(max(float(tolerance), 1e-9), radius)
    dphi = 2.0 * math.acos(max(-1.0, 1.0 - tol / radius))
    n = int(math.ceil(t * (1.0 + p) / max(dphi, 1e-12)))
    if n > _MAX_BEND_SAMPLES:
        step = t * (1.0 + p) / _MAX_BEND_SAMPLES
        got = radius * (1.0 - math.cos(0.5 * step))
        warnings.warn(
            f"Euler bend of {math.degrees(t):.1f} deg at radius {radius:g} um "
            f"would need {n} vertices to hold a {tol:g} um chord tolerance, "
            f"above the {_MAX_BEND_SAMPLES} cap; the polyline will sit up to "
            f"{got:.3g} um inside the true curve instead. Raise the tolerance, "
            f"or hold the radius down with Rmax — this is the radius the "
            f"solver chose, which is usually well above Rmin.",
            stacklevel=3,
        )
    return max(_MIN_BEND_SAMPLES, min(n, _MAX_BEND_SAMPLES))


def _unit_heading(s: float, t: float, lc: float, la: float, l: float) -> float:
    """Heading of the ``R = 1`` bend at arc length *s* (see module docstring)."""
    if s <= 0.0:
        return 0.0
    if s >= l:
        return t
    if lc <= 0.0:                      # pure arc: kappa = 1
        return s
    if s <= lc:
        return s * s / (2.0 * lc)
    if s <= lc + la:
        return 0.5 * lc + (s - lc)
    return t - (l - s) * (l - s) / (2.0 * lc)


def _unit_points(t: float, p: float, n: int) -> List[Tuple[float, float]]:
    """``n + 1`` points along the ``R = 1`` bend, turning ``+t`` from the origin.

    The heading is analytic, so only the position needs quadrature.  Each
    output interval is integrated with composite Simpson over ``_SUB_PANELS``
    sub-panels and accumulated, which keeps the emitted vertices and the
    endpoint on one consistent integral.
    """
    lc = p * t
    la = (1.0 - p) * t
    length = 2.0 * lc + la
    m = n * _SUB_PANELS
    h = length / m

    cs = []
    sn = []
    for i in range(m + 1):
        phi = _unit_heading(i * h, t, lc, la, length)
        cs.append(math.cos(phi))
        sn.append(math.sin(phi))

    pts = [(0.0, 0.0)]
    x = y = 0.0
    third = h / 3.0
    for i in range(n):
        b = i * _SUB_PANELS
        sx = cs[b] + cs[b + _SUB_PANELS]
        sy = sn[b] + sn[b + _SUB_PANELS]
        for j in range(1, _SUB_PANELS):
            w = 4.0 if j % 2 else 2.0
            sx += w * cs[b + j]
            sy += w * sn[b + j]
        x += third * sx
        y += third * sy
        pts.append((x, y))
    return pts


def bend_points(turn: float, radius: float, p: float,
                n: int) -> List[Tuple[float, float]]:
    """``n + 1`` points along a bend entering the origin heading ``+x``.

    The last point is replaced by the closed-form :func:`bend_offset`, so a
    composed route lands on its target to floating-point precision rather than
    to the quadrature's.
    """
    t = abs(turn)
    if t < ANGLE_TOL or n <= 0:
        return [(0.0, 0.0)]
    sgn = 1.0 if turn >= 0.0 else -1.0
    pts = [(radius * x, sgn * radius * y) for x, y in _unit_points(t, p, n)]
    pts[-1] = bend_offset(turn, radius, p)
    return pts


# ---------------------------------------------------------------------------
# Route plans
# ---------------------------------------------------------------------------

Segment = Tuple[str, float]


@dataclass
class EulerPlan:
    """A solved route: which segments, and what they cost.

    *shape* is ``"straight"``, ``"corner"``, ``"sbend"`` or ``"uturn"``.
    *radius* is the peak-curvature radius every bend actually uses, which is
    the requested ``Rmin`` — the bends are drawn as tight as the profile
    permits and the straights absorb whatever slack is left.  A bend reaches
    that radius at one point when ``p = 1`` and holds it across the circular
    middle otherwise; either way it never goes tighter.
    """

    shape: str
    segments: List[Segment]
    radius: float
    p: float
    length: float
    turn: float


def _wrap_pi(angle: float) -> float:
    """Wrap to ``(-pi, pi]``."""
    a = math.fmod(angle, 2.0 * math.pi)
    if a <= -math.pi:
        a += 2.0 * math.pi
    elif a > math.pi:
        a -= 2.0 * math.pi
    return a


def _plan_length(segments: Sequence[Segment], radius: float, p: float) -> float:
    total = 0.0
    for kind, value in segments:
        total += value if kind == STRAIGHT else bend_length(value, radius, p)
    return total


def _plan_turn(segments: Sequence[Segment]) -> float:
    return sum(v for k, v in segments if k == BEND)


def _two_bend_offset(alpha: float, total: float, radius: float,
                     p: float) -> Tuple[float, float]:
    """Offset of ``bend(alpha)`` followed immediately by ``bend(total-alpha)``."""
    d1x, d1y = bend_offset(alpha, radius, p)
    d2x, d2y = bend_offset(total - alpha, radius, p)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return (d1x + ca * d2x - sa * d2y, d1y + sa * d2x + ca * d2y)


def _roots(f: Callable[[float], float], lo: float, hi: float,
           scan: int = 361) -> List[float]:
    """Every sign-change root of *f* on ``[lo, hi]``, bisected to machine precision.

    A grid scan then bisection, because the profile pins no SciPy.  Roots
    where *f* only touches zero without changing sign are missed, which are
    exactly the marginally-feasible geometries; they show up as an
    infeasibility rather than as a bad route.

    The roots are well separated in practice: over several hundred port poses,
    scans of 181, 361, 721, 1441 and 2881 points pick the same shape and the
    same segments to within about 1e-12 um (the roots themselves differ in
    their last bits), so the grid is set for a comfortable margin over the
    coarsest that works rather than for resolution.  It dominates the solve
    cost — a route is one to two thousand evaluations of :func:`bend_offset`.
    """
    if hi - lo < ANGLE_TOL:
        return []
    step = (hi - lo) / (scan - 1)
    grid = [lo + i * step for i in range(scan)]
    vals = [f(a) for a in grid]

    out: List[float] = []
    for i in range(scan):
        if vals[i] == 0.0:
            out.append(grid[i])
    for i in range(scan - 1):
        if vals[i] * vals[i + 1] < 0.0:
            a, b, fa = grid[i], grid[i + 1], vals[i]
            for _ in range(80):
                mid = 0.5 * (a + b)
                fm = f(mid)
                if fm == 0.0:
                    break
                if (fa < 0.0) != (fm < 0.0):
                    b = mid
                else:
                    a, fa = mid, fm
            out.append(0.5 * (a + b))
    return out


def _split_bracket(total: float) -> Tuple[float, float]:
    """Range of the first turn ``alpha`` keeping both bends within +-pi."""
    return (max(-math.pi, total - math.pi), min(math.pi, total + math.pi))


def _congruent_turn(delta: float) -> float | None:
    """The net turn that reaches heading *delta* the long way round, if any.

    A two-bend shape can arrive pointing along *delta* by turning
    ``delta -+ 2*pi`` instead.  That is what makes ``|delta| = pi`` routable at
    all: with the net turn pinned to ``+pi`` both bends must turn left, the
    heading never leaves ``[0, pi]``, so the route's ``y`` only ever increases
    and every target below the start port's axis is out of reach.  The
    congruent turn ``-pi`` mirrors that and covers the other half plane.

    It costs a lot more turning, so it is only tried when nothing reaches the
    ports the short way — see :func:`plan_route`.
    """
    if abs(delta) <= ANGLE_TOL:
        return None
    alt = delta - math.copysign(2.0 * math.pi, delta)
    return alt if abs(alt) <= 2.0 * math.pi + ANGLE_TOL else None


def _cross(v, w) -> float:
    """2-D cross product, the scalar ``v.x*w.y - v.y*w.x``."""
    return v[0] * w[1] - v[1] * w[0]


def _side(a, b, c) -> float:
    """Which side of the line *a*-*b* the point *c* falls on."""
    return _cross((b[0] - a[0], b[1] - a[1]), (c[0] - a[0], c[1] - a[1]))


def _crosses(pts: Sequence[Tuple[float, float]]) -> bool:
    """True if the polyline crosses itself.

    Screened coarsely, on :data:`_CROSS_SAMPLES` vertices per bend rather than
    the emitted resolution: a route that loops back over itself would create an
    unintended waveguide crossing, and the congruent turn of
    :func:`_congruent_turn` is what makes that reachable at all.
    """
    n = len(pts) - 1
    for i in range(n):
        for j in range(i + 2, n):
            d1, d2 = _side(pts[j], pts[j + 1], pts[i]), _side(pts[j], pts[j + 1], pts[i + 1])
            d3, d4 = _side(pts[i], pts[i + 1], pts[j]), _side(pts[i], pts[i + 1], pts[j + 1])
            if (d1 > 0.0) != (d2 > 0.0) and (d3 > 0.0) != (d4 > 0.0):
                return True
    return False


def _radius_ceiling(x_rel, y_rel, r_min, r_max) -> float:
    """Largest radius worth using, whether or not the profile capped it.

    As the turn shrinks the corner's straights fall ever more slowly with
    radius (the rate goes as ``|delta|``), so the geometric maximum runs away
    like ``1/|delta|`` — a 0.001 degree turn admits a 2.9 *metre* radius.  The
    drawn shape converges long before that and the extra radius buys nothing,
    while the solve for it grows ill conditioned and the vertex count grows as
    ``sqrt(R)``.  A turn that shallow is a straight with a kink, not a bend.

    So the radius is always held to a modest multiple of the port separation,
    and *r_max* tightens that further when the profile asks.
    """
    ceiling = _MAX_RADIUS_FACTOR * max(math.hypot(x_rel, y_rel), r_min)
    if r_max is not None:
        ceiling = min(ceiling, r_max)
    return max(ceiling, r_min)


def _radius_ladder(r_min: float, top: float) -> List[float]:
    """Radii to offer for one shape, gentlest first, ending at ``r_min``.

    Geometric rungs, so the ladder is scale free.  ``_LADDER_RUNGS`` of them is
    plenty: the screens that reject a radius are monotone in it, so a couple of
    rungs is usually the difference between a route and a `DesignError`.
    """
    if top <= r_min * (1.0 + 1e-12):
        return [r_min]
    ratio = (top / r_min) ** (1.0 / (_LADDER_RUNGS - 1))
    return [min(top, r_min * ratio ** i)
            for i in range(_LADDER_RUNGS - 1, -1, -1)]


def _bulge(pts, x_rel, y_rel) -> float:
    """How far *pts* strays outside the rectangle the two ports span.

    A corner or an S-bend stays inside that rectangle only while the target is
    ahead of the start port: the route leaves the origin heading +x, so a
    target behind it puts the first straight outside an axis-aligned box
    immediately.  Over sampled poses the accepted corners bulge a median 0.14
    of the separation and reach 0.855.  What this catches is the other thing a
    free radius can produce: a pair of near-half-turn bends at a radius many times ``Rmin``,
    which closes on the ports exactly and passes every other screen while
    sweeping millimetres across the die.  Left unscreened, poses that used to
    raise a `DesignError` came back as geometry instead.
    """
    x0, x1 = min(0.0, x_rel), max(0.0, x_rel)
    y0, y1 = min(0.0, y_rel), max(0.0, y_rel)
    out = 0.0
    for px, py in pts:
        out = max(out, x0 - px, px - x1, y0 - py, py - y1)
    return out


def _corner(x_rel, y_rel, delta, r_min, r_max, p, cap, why):
    """``straight(s1) - bend(delta) - straight(s2)``, at the largest radius.

    For a fixed turn the bend offset is ``R * q * u(delta/2)``, so both
    straights are *affine* in ``R`` — and, after the half-angle identities,
    they fall at exactly the same rate::

        ds1/dR = ds2/dR = -q / (2*cos(delta/2))

    So the radius can be grown until whichever straight was shorter reaches
    zero, and that gives ``R_max`` in closed form with no search.  A pleasant
    side effect: near-antiparallel ports, where pinning ``R`` to ``Rmin`` sends
    both straights to infinity, now yield a modest radius and short straights
    instead — the runaway solves itself.
    """
    sin_d, cos_d = math.sin(delta), math.cos(delta)
    if abs(sin_d) <= _SIN_TOL:
        why.append(
            "corner: the ports face the same way, so a single bend and two "
            "straights cannot reach across the offset"
        )
        return []

    q = unit_chord(abs(delta), p)
    half = 0.5 * delta
    dhx, dhy = q * math.cos(half), q * math.sin(half)

    def straights(radius):
        s2 = (y_rel - radius * dhy) / sin_d
        return (x_rel - radius * dhx - s2 * cos_d, s2)

    # How negative a straight may be and still count as zero.  Dividing by
    # sin(delta) and then subtracting s2*cos(delta) costs about -log10|sin d|
    # digits, so near-antiparallel ports return a genuinely-zero straight as
    # a few tens of femtometres of the wrong sign.  Against a fixed tolerance
    # that reads as a real overshoot and rejects a corner that fits.
    zero_tol = (POS_TOL + 8.0 * _EPS * (abs(x_rel) + abs(y_rel))
                / max(abs(sin_d), _SIN_TOL))

    s1, s2 = straights(r_min)
    if s1 < -zero_tol or s2 < -zero_tol:
        why.append(
            f"corner: a single {math.degrees(delta):.3f} deg bend at "
            f"Rmin={r_min:g} um wants straights of {s1:.6g} um and "
            f"{s2:.6g} um, and a negative one would double back"
        )
        return []

    # Both straights fall at this rate per unit radius, so the shorter one
    # decides.  Clamp below: the gate above admits a slightly negative
    # straight, and dividing that would put the radius under Rmin — the one
    # invariant this module promises.
    # Both straights fall at this rate per unit radius, so the shorter one
    # decides.  Clamp below: the gate above admits a slightly negative
    # straight, and dividing that would put the radius under Rmin — the one
    # invariant this module promises.
    rate = q / (2.0 * math.cos(half))
    top = max(r_min, r_min + min(s1, s2) / rate)
    top = min(top, _radius_ceiling(x_rel, y_rel, r_min, r_max))

    # Offer the whole ladder from the gentlest bend down to Rmin, not just the
    # gentlest.  The screens in plan_route reject on the drawn shape, and a
    # radius that sweeps too far can be perfectly acceptable a rung lower — so
    # handing over one candidate would make a screen cost reachability that
    # pinning the radius at Rmin never lost.
    out = []
    for radius in _radius_ladder(r_min, top):
        s1, s2 = straights(radius)
        if s1 + s2 > cap:
            continue
        out.append((1, False, "corner",
                    [(STRAIGHT, max(s1, 0.0)), (BEND, delta),
                     (STRAIGHT, max(s2, 0.0))], radius))
    if not out:
        why.append(
            f"corner: reaches the ports but spends more than {cap:.3f} um of "
            f"straight doubling back, at every radius from Rmin={r_min:g} um "
            f"to {top:g} um"
        )
    return out


def _two_bend(x_rel, y_rel, delta, r_min, r_max, p, turns, cap, kind, order,
              why):
    """Two bends around one straight, at the largest radius that fits.

    ``sbend`` is ``straight(s) - bend(a) - bend(total-a) - straight(s)`` and
    ``uturn`` is ``bend(a) - straight(m) - bend(total-a)``.  Both read, for a
    *fixed* split ``a``, as two linear equations in the two unknowns (the
    straight and the radius)::

        sbend:  s * c1     + R * Dhat(a) = (x_rel, y_rel)
        uturn:  m * u(a)   + R * Dhat(a) = (x_rel, y_rel)

    Cramer on those gives ``R = N/den`` and ``straight = M/den`` with

        den(a) = cross(col, Dhat(a))
        N(a)   = cross(col, target)
        M(a)   = cross(target, Dhat(a))

    so maximising the radius is one-dimensional in the split, and needs only
    unit-radius bend evaluations.

    ``R(a)`` itself is unusable as a search objective: ``den`` vanishes at
    splits fixed by the geometry alone, so ``R`` has poles and both the radius
    and the straight run away together there.  ``den``, ``N`` and ``M`` are all
    smooth, though, so the candidate splits are found on *those*: where the
    straight reaches zero (``M = 0``), where the radius reaches a bound
    (``N - R*den = 0``), and at the ends of the bracket, where one bend
    saturates a half turn.  That last family is the one a
    "just set the straight to zero" shortcut would miss — around 40% of
    two-bend optima sit there, with a strictly positive straight.  A grid scan
    runs alongside as a backstop.
    """
    target = (x_rel, y_rel)
    c1 = (1.0 + math.cos(delta), math.sin(delta))
    if kind == "sbend" and c1[0] * c1[0] + c1[1] * c1[1] <= _C1_TOL:
        why.append(
            "sbend: the ports face the same way (180 deg turn), so the two "
            "end straights would be antiparallel and cannot position the route"
        )
        return []

    ceiling = _radius_ceiling(x_rel, y_rel, r_min, r_max)
    out = []

    for congruent, total in turns:
        lo, hi = _split_bracket(total)
        if hi - lo < ANGLE_TOL:
            continue

        def parts(alpha, total=total):
            """The three smooth quantities the solve is built from."""
            col = c1 if kind == "sbend" else (math.cos(alpha), math.sin(alpha))
            dhat = _two_bend_offset(alpha, total, 1.0, p)
            return (_cross(col, dhat), _cross(col, target),
                    _cross(target, dhat))

        def evaluate(alpha, total=total):
            """``(radius, straight)`` for this split, or ``None`` if unusable."""
            den, numer, moment = parts(alpha, total)
            if den == 0.0:
                return None
            radius, straight = numer / den, moment / den
            if not (math.isfinite(radius) and math.isfinite(straight)):
                return None
            if radius < r_min or radius > ceiling or straight < -POS_TOL:
                return None
            total_straight = 2.0 * straight if kind == "sbend" else straight
            if total_straight > cap:
                return None
            return (radius, straight)

        # Exact candidates, on the smooth functions rather than on R itself.
        splits = [lo, hi]
        splits += _roots(lambda a, t=total: parts(a, t)[2], lo, hi,
                         scan=_SPLIT_SCAN)
        for bound in (r_min, ceiling):
            splits += _roots(
                lambda a, t=total, r=bound: parts(a, t)[1] - r * parts(a, t)[0],
                lo, hi, scan=_SPLIT_SCAN)

        # Grid backstop, in case R(a) has a maximum the candidates above miss.
        step = (hi - lo) / (_SPLIT_SCAN - 1)
        grid = [(lo + i * step, evaluate(lo + i * step))
                for i in range(_SPLIT_SCAN)]
        peaks = []
        for i, (_, value) in enumerate(grid):
            if value is None:
                continue
            before = grid[i - 1][1] if i > 0 else None
            after = grid[i + 1][1] if i < _SPLIT_SCAN - 1 else None
            if ((before is None or before[0] <= value[0])
                    and (after is None or after[0] <= value[0])):
                peaks.append(i)
        peaks.sort(key=lambda i: -grid[i][1][0])
        for i in peaks[:_SPLIT_PEAKS]:
            splits.append(_refine_split(evaluate, grid, i, lo, hi, step)[0])

        # Every feasible split, gentlest first — not just the gentlest.  The
        # screens in plan_route judge the drawn shape, and a split they reject
        # at one radius is often fine at a smaller one, so offering a single
        # candidate would cost reachability a pinned radius never lost.
        found = []
        for alpha in splits:
            alpha = min(hi, max(lo, alpha))
            value = evaluate(alpha, total)
            if value is not None:
                found.append((value[0], alpha, value[1]))
        if not found:
            continue

        found.sort(key=lambda c: -c[0])
        kept = []
        for radius, alpha, straight in found:
            # Radii within a whisker of one already kept draw the same route.
            if any(radius > 0.995 * r for r in kept):
                continue
            kept.append(radius)
            if kind == "sbend":
                segments = [(STRAIGHT, max(straight, 0.0)), (BEND, alpha),
                            (BEND, total - alpha),
                            (STRAIGHT, max(straight, 0.0))]
            else:
                segments = [(BEND, alpha), (STRAIGHT, max(straight, 0.0)),
                            (BEND, total - alpha)]
            out.append((order, congruent, kind, segments, radius))
            if len(kept) >= _LADDER_RUNGS:
                break

    if not out:
        if kind == "sbend":
            why.append(
                f"sbend: no two-bend split at Rmin={r_min:g} um reaches the "
                f"target with a non-negative pair of end straights"
            )
        else:
            why.append(
                f"uturn: no two-bend split at Rmin={r_min:g} um leaves a "
                f"non-negative straight between the bends. A 180 deg turn "
                f"needs at least {unit_chord(math.pi, p) * r_min:.3f} um of "
                f"offset across the start port's axis"
            )
    return out


def _refine_split(evaluate, grid, index, lo, hi, step):
    """Golden-section refine the split around grid point *index*."""
    a0 = max(lo, grid[index][0] - step)
    a1 = min(hi, grid[index][0] + step)

    def value(alpha):
        got = evaluate(alpha)
        return -math.inf if got is None else got[0]

    for _ in range(_SPLIT_REFINE):
        m1 = a0 + 0.382 * (a1 - a0)
        m2 = a0 + 0.618 * (a1 - a0)
        if value(m1) < value(m2):
            a0 = m1
        else:
            a1 = m2
    alpha = 0.5 * (a0 + a1)
    got = evaluate(alpha)
    # The refinement can land just outside the feasible pocket; keep the grid
    # point unless the refined split is genuinely better.
    if got is None or got[0] <= grid[index][1][0]:
        return grid[index][0], grid[index][1]
    return alpha, got


def plan_route(x_rel: float, y_rel: float, delta: float, r_min: float,
               p: float = DEFAULT_P, r_max: float | None = None) -> EulerPlan:
    """Solve the segment plan joining two ports, at the gentlest radius that fits.

    Everything is in the start port's frame: the route leaves the origin
    heading ``+x`` and must arrive at ``(x_rel, y_rel)`` heading *delta*.

    *r_min* is the minimum tolerable bend radius — a floor, not the operating
    point.  Bend loss and mode conversion both fall with radius (peak curvature
    as ``1/R``, curvature rate as ``1/R^2``), so each shape is solved for the
    **largest** radius its geometry admits and the straights take what is left.
    *r_max* optionally caps that, for when a bend sweeping the whole space
    between two ports would collide with what is placed there.

    Every shape is solved, not just the first that fits, because a shape can
    fit and still be absurd.  Candidates are screened on how far their
    straights detour (:data:`_MAX_DETOUR`), on total length and on
    self-intersection; the shape preference then picks among the survivors,
    taking the largest radius within a shape.

    Raises :class:`~src.design.DesignError` when nothing fits, with what each
    shape would have needed.
    """
    # Finiteness first: every comparison against a NaN is False, so a NaN would
    # slip past the sign check, past every screen below, and end up as a
    # one-vertex path that gdstk drops — a route silently missing from the GDS.
    if not math.isfinite(r_min) or r_min <= 0.0:
        raise DesignError(f"Euler curve needs a finite Rmin > 0, got {r_min!r}.")
    if not (math.isfinite(p) and 0.0 < p <= 1.0):
        raise DesignError(
            f"Euler curve needs 0 < p <= 1, got {p!r}. p is the fraction of "
            f"the turn spent ramping curvature; p=1 is the pure clothoid and "
            f"p=0 would be a circular arc with a curvature step at each end."
        )
    if r_max is not None:
        if not math.isfinite(r_max) or r_max <= 0.0:
            raise DesignError(
                f"Euler curve needs a finite Rmax > 0, got {r_max!r}."
            )
        if r_max < r_min:
            raise DesignError(
                f"Euler curve got Rmax={r_max:g} um below Rmin={r_min:g} um; "
                f"Rmax caps how gentle a bend may become and cannot be tighter "
                f"than the minimum tolerable radius."
            )

    if not (math.isfinite(x_rel) and math.isfinite(y_rel)
            and math.isfinite(delta)):
        raise DesignError(
            f"Euler curve got a non-finite port geometry: target "
            f"({x_rel!r}, {y_rel!r}), turn {delta!r}."
        )

    delta = _wrap_pi(delta)
    separation = math.hypot(x_rel, y_rel)
    if separation < POS_TOL:
        raise DesignError(
            "Euler curve endpoints coincide: the two ports are at the same "
            "point, so there is no route to draw. Ports mated by a 'connect' "
            "step are already joined and need no route between them."
        )

    why: List[str] = []

    # -- straight ----------------------------------------------------------
    if abs(delta) < ANGLE_TOL and abs(y_rel) < POS_TOL:
        if x_rel > POS_TOL:
            return EulerPlan("straight", [(STRAIGHT, x_rel)], r_min, p,
                             x_rel, 0.0)
        why.append(
            f"straight: the ports are collinear but the target is "
            f"{x_rel:.6f} um along the start port's own axis, so a route "
            f"would have to run backwards through it"
        )

    # -- every other shape, every solution ---------------------------------
    cap = _MAX_DETOUR * separation
    # False = the short way round, True = the congruent turn.  The flag rides
    # along into the sort key: going the long way is a fallback, and without
    # it a congruent turn's larger radius would outrank the short way.
    turns = [(False, delta)]
    congruent = _congruent_turn(delta)
    if congruent is not None:
        turns.append((True, congruent))

    candidates = (
        _corner(x_rel, y_rel, delta, r_min, r_max, p, cap, why)
        + _two_bend(x_rel, y_rel, delta, r_min, r_max, p, turns, cap,
                    "sbend", 2, why)
        + _two_bend(x_rel, y_rel, delta, r_min, r_max, p, turns, cap,
                    "uturn", 3, why)
    )

    # Shape preference, then the short way round, then the gentlest bend, then
    # the shortest route.
    candidates.sort(
        key=lambda c: (c[0], c[1], -c[4], _plan_length(c[3], c[4], p))
    )

    budget = _LENGTH_BUDGET * (separation + 2.0 * math.pi * (1.0 + p) * r_min)
    bulge_cap = _MAX_BULGE * separation
    too_long = 0
    crossing = 0
    bulging = 0
    for _, _, shape, segments, radius in candidates:
        if _plan_length(segments, radius, p) > budget:
            too_long += 1
            continue
        points = _segment_points(segments, radius, p, bend_n=_CROSS_SAMPLES)
        if _bulge(points, x_rel, y_rel) > bulge_cap:
            bulging += 1
            continue
        if _crosses(points):
            crossing += 1
            continue
        # Drop no-op segments: a zero-length straight, and a zero-turn bend,
        # which is what a two-bend shape collapses to when the maximum radius
        # puts the whole turn in one of its bends.  Neither contributes offset
        # or heading, so removing them keeps the plan an honest description.
        segs = [(k, v) for k, v in segments
                if (abs(v) > ANGLE_TOL if k == BEND else v > 0.0)]
        return EulerPlan(shape, segs, radius, p,
                         _plan_length(segs, radius, p), _plan_turn(segs))

    if too_long:
        why.append(
            f"{too_long} solution(s) ran past the {budget:.0f} um length budget"
        )
    if bulging:
        why.append(
            f"{bulging} solution(s) swept more than {bulge_cap:.3f} um "
            f"({_MAX_BULGE:g}x the port separation) outside the rectangle the "
            f"two ports span"
        )
    if crossing:
        why.append(f"{crossing} solution(s) crossed themselves")

    raise DesignError(
        f"No Euler curve joins these ports at Rmin={r_min:g} um (p={p:g}"
        + (f", Rmax={r_max:g} um" if r_max is not None else "") + "). "
        f"In the start port's frame the target is at "
        f"({x_rel:.3f}, {y_rel:.3f}) um with a "
        f"{math.degrees(delta):.3f} deg turn. Tried: " + "; ".join(why) +
        ". Move the ports further apart, lower Rmin, or route via an "
        "intermediate port."
    )


def _segment_points(segments: Sequence[Segment], radius: float, p: float,
                    tolerance: float = DEFAULT_TOLERANCE,
                    bend_n: int | None = None
                    ) -> List[Tuple[float, float]]:
    """Vertices of *segments*, starting at the origin heading ``+x``.

    *bend_n* forces a fixed vertex count per bend, which the
    self-intersection screen uses; otherwise the count comes from
    :func:`bend_samples` and *tolerance*.
    """
    pts = [(0.0, 0.0)]
    x = y = 0.0
    heading = 0.0

    for kind, value in segments:
        if kind == STRAIGHT:
            if value <= POS_TOL:
                continue
            x += value * math.cos(heading)
            y += value * math.sin(heading)
            pts.append((x, y))
            continue

        n = (bend_n if bend_n is not None
             else bend_samples(value, radius, p, tolerance))
        local = bend_points(value, radius, p, n)
        ch, sh = math.cos(heading), math.sin(heading)
        for lx, ly in local[1:]:
            pts.append((x + ch * lx - sh * ly, y + sh * lx + ch * ly))
        x, y = pts[-1]
        heading += value

    # Drop vertices that coincide: gdstk would emit degenerate polygon corners.
    out = [pts[0]]
    for px, py in pts[1:]:
        qx, qy = out[-1]
        if abs(px - qx) > 1e-12 or abs(py - qy) > 1e-12:
            out.append((px, py))
    return out


def plan_points(plan: EulerPlan,
                tolerance: float = DEFAULT_TOLERANCE
                ) -> List[Tuple[float, float]]:
    """Vertices of *plan* in the start port's frame, beginning at the origin."""
    return _segment_points(plan.segments, plan.radius, plan.p, tolerance)
