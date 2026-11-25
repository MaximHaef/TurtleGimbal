#!/usr/bin/env python3
"""Generate an excitation sequence for training-data collection (F9, WP1).

    python3 tools/excitation/make_sequence.py --box 1.5 --seed 1 --out seq_s1.yaml

Writes a YAML file in the format of base_teleop/sequence.py, to be played by
the excitation node (data_collection.launch.py sequence_file:=...). The same
arguments always give the same file.

Each round contains, in this order (amplitudes as fractions of --vmax/--wmax):

  rest              5 s at (0, 0): gyro bias, floor-vibration baseline
  straight          v = +-0.3, +-0.6, +-1.0 vmax, back and forth
  turn in place     w = +-0.3, +-0.6, +-1.0 wmax
  turn while drive  v = 0.5 vmax with w = +-0.3, +-0.6 wmax; each arc is driven
                    back along itself, so the robot returns to where it started
  steps             the steps of base_teleop/config/sysid_steps.yaml (tau_v and
                    tau_omega stay identifiable from the same bag)
  ramps             staircases of 0.25 s steps up to the limit and back down
  random            0.5-2.0 s segments, (v, w) uniform within the limits, with a
                    0.5-1.0 s rest after a segment with probability 0.3
  return            turn to face the start, drive to it, turn back to the start
                    heading

Constraints enforced on every segment (checked again on the whole sequence):
  - duration is a multiple of 0.05 s (excitation node at 20 Hz) and >= 0.25 s
  - outer wheel speed |v| + |w| b/2 <= 0.9 x wheel speed limit
  - the commanded unicycle path stays inside the square of half-width --box
    around the start pose (x forward, y left at the start)
  - the first and the last segment are rests

The box check uses the commanded path. The real robot reaches only part of
the commanded speed (tools/sysid measured K = 0.66-0.74) and slips, so the
real path drifts; give --box with a safety margin to the obstacles.
"""
import argparse
import math
import random
import sys

DT = 0.05                  # s, period of the excitation node (publish_rate 20 Hz)
MIN_DURATION = 0.25        # s
TRACK_WIDTH = 0.23         # m, b, Kobuki spec, unverified (base_command.yaml)
WHEEL_SPEED_LIMIT = 0.7    # m/s, Kobuki spec, unverified (base_command.yaml)
WHEEL_MARGIN = 0.9         # fraction of the wheel-speed limit the sequence may use
DEFAULT_VMAX = 0.2         # m/s, = largest v in sysid_steps.yaml
DEFAULT_WMAX = 1.0         # rad/s, = largest |w| in sysid_steps.yaml

FIRST_REST = 5.0           # s
REST = 2.5                 # s, between designed segments
LEVELS = (0.3, 0.6, 1.0)   # fractions of vmax / wmax
STRAIGHT_MAX = 10.0        # s
STRAIGHT_MIN = 6.0         # s, shorter only if the box is too small (with a note)
TURN_DURATION = 4.0        # s
ARC_SPEED = 0.5            # fraction of vmax
ARC_LEVELS = (0.3, 0.6)    # fractions of wmax
ARC_DURATION = 8.0         # s
RAMP_STEP = 0.25           # s
RAMP_DURATION = 10.0       # s, up and down
RANDOM_DURATION = 120.0    # s per round
RANDOM_SEGMENT = (0.5, 2.0)  # s
RANDOM_REST = (0.5, 1.0)   # s
RANDOM_REST_PROBABILITY = 0.3
RANDOM_ATTEMPTS = 200
RETURN_FRACTION = 0.5      # return-to-start speeds as fractions of vmax / wmax
RETURN_TOLERANCE = 0.02    # m and rad, smaller errors are not corrected

# Copy of base_teleop/config/sysid_steps.yaml without its first rest
# (test_make_sequence.py checks that the two agree).
SYSID_STEPS = [
    (3.0, 0.0, 0.3), (2.5, 0.0, 0.0), (3.0, 0.0, -0.3), (2.5, 0.0, 0.0),
    (3.0, 0.0, 0.6), (2.5, 0.0, 0.0), (3.0, 0.0, -0.6), (2.5, 0.0, 0.0),
    (3.0, 0.0, 1.0), (2.5, 0.0, 0.0), (3.0, 0.0, -1.0), (2.5, 0.0, 0.0),
    (3.0, 0.1, 0.0), (2.5, 0.0, 0.0), (3.0, -0.1, 0.0), (2.5, 0.0, 0.0),
    (2.0, 0.2, 0.0), (2.5, 0.0, 0.0), (2.0, -0.2, 0.0), (2.5, 0.0, 0.0),
    (3.0, 0.1, 0.5), (2.5, 0.0, 0.0), (3.0, -0.1, -0.5), (3.0, 0.0, 0.0),
]

PATH_STEP = 0.01           # s, resolution of the box check


def ticks(duration):
    return int(round(duration / DT))


def snap(duration, mode='round'):
    """Duration on the 0.05 s grid, at least MIN_DURATION."""
    x = duration / DT
    n = {'round': round(x), 'floor': math.floor(x + 1e-9), 'ceil': math.ceil(x - 1e-9)}[mode]
    return round(max(ticks(MIN_DURATION), int(n)) * DT, 2)


def clean(x):
    x = round(float(x), 4)
    return 0.0 if x == 0.0 else x


def wheel_speed(v, w, track_width):
    return abs(v) + abs(w) * track_width / 2.0


def drive(pose, duration, v, w):
    """Exact unicycle motion. Returns the end pose (x, y, heading)."""
    x, y, th = pose
    if abs(w) < 1e-12:
        return x + v * duration * math.cos(th), y + v * duration * math.sin(th), th
    th1 = th + w * duration
    return (x + v / w * (math.sin(th1) - math.sin(th)),
            y - v / w * (math.cos(th1) - math.cos(th)), th1)


def path_extent(pose, duration, v, w):
    """Largest max(|x|, |y|) along a segment, and its end pose."""
    n = max(1, int(math.ceil(duration / PATH_STEP)))
    extent = max(abs(pose[0]), abs(pose[1]))
    for i in range(1, n + 1):
        x, y, _ = drive(pose, duration * i / n, v, w)
        extent = max(extent, abs(x), abs(y))
    return extent, drive(pose, duration, v, w)


def validate(segments, box, track_width=TRACK_WIDTH, wheel_speed_limit=WHEEL_SPEED_LIMIT):
    """Return a list of problems (empty if the sequence obeys all constraints)."""
    problems = []
    if not segments:
        return ['no segments']
    limit = WHEEL_MARGIN * wheel_speed_limit
    pose = (0.0, 0.0, 0.0)
    for i, (d, v, w) in enumerate(segments):
        if abs(d / DT - ticks(d)) > 1e-6:
            problems.append(f'segment {i}: duration {d} is not a multiple of {DT} s')
        if d < MIN_DURATION - 1e-9:
            problems.append(f'segment {i}: duration {d} below {MIN_DURATION} s')
        if wheel_speed(v, w, track_width) > limit + 1e-9:
            problems.append(f'segment {i}: wheel speed {wheel_speed(v, w, track_width):.3f} m/s above {limit:.3f}')
        extent, pose = path_extent(pose, d, v, w)
        if extent > box + 1e-6:
            problems.append(f'segment {i}: leaves the box ({extent:.3f} m > {box} m)')
    for i in (0, len(segments) - 1):
        if segments[i][1] != 0.0 or segments[i][2] != 0.0:
            problems.append(f'segment {i}: first and last segment must be rests')
    return problems


class Builder:
    """Collects segments, tracks the commanded pose, refuses unsafe segments."""

    def __init__(self, box, track_width, wheel_speed_limit):
        self.box = box
        self.track_width = track_width
        self.limit = WHEEL_MARGIN * wheel_speed_limit
        self.pose = (0.0, 0.0, 0.0)
        self.segments = []
        self.blocks = []   # (index of first segment, label)
        self.max_extent = 0.0

    def block(self, label):
        self.blocks.append((len(self.segments), label))

    def fits(self, d, v, w):
        if wheel_speed(v, w, self.track_width) > self.limit + 1e-9:
            return False
        extent, _ = path_extent(self.pose, d, v, w)
        return extent <= self.box + 1e-6

    def add(self, d, v, w):
        d, v, w = snap(d), clean(v), clean(w)
        if wheel_speed(v, w, self.track_width) > self.limit + 1e-9:
            raise ValueError(f'(v={v}, w={w}) needs wheel speed {wheel_speed(v, w, self.track_width):.3f} m/s, '
                             f'above {self.limit:.3f} m/s (0.9 x limit): lower --vmax/--wmax')
        extent, pose = path_extent(self.pose, d, v, w)
        if extent > self.box + 1e-6:
            raise ValueError(f'segment (d={d}, v={v}, w={w}) leaves the {self.box} m box '
                             f'({extent:.2f} m): enlarge --box or lower --vmax')
        self.pose = pose
        self.max_extent = max(self.max_extent, extent)
        self.segments.append((d, v, w))

    def rest(self, d=REST):
        self.add(d, 0.0, 0.0)

    def room(self):
        """Free distance [m] from the current pose to the box edge, with a small margin."""
        return self.box - max(abs(self.pose[0]), abs(self.pose[1])) - 2 * RETURN_TOLERANCE


def add_straight(b, vmax, notes):
    for k in LEVELS:
        v = k * vmax
        d = min(STRAIGHT_MAX, snap(b.room() / v, 'floor'))
        if d < STRAIGHT_MIN:
            notes.add(f'straight segments at {v:.3f} m/s shortened to {d} s to stay in the box')
        for sign in (1, -1):
            b.add(d, sign * v, 0.0)
            b.rest()


def add_turns(b, wmax):
    for k in LEVELS:
        for sign in (1, -1):
            b.add(TURN_DURATION, 0.0, sign * k * wmax)
            b.rest()


def add_arcs(b, vmax, wmax):
    v = ARC_SPEED * vmax
    for k in ARC_LEVELS:
        for sign in (1, -1):
            w = sign * k * wmax
            b.add(ARC_DURATION, v, w)
            b.rest()
            b.add(ARC_DURATION, -v, -w)   # back along the same arc
            b.rest()


def add_steps(b, vmax, wmax, notes):
    for d, v, w in SYSID_STEPS:
        vc, wc = max(-vmax, min(vmax, v)), max(-wmax, min(wmax, w))
        if (vc, wc) != (v, w):
            notes.add('sysid steps clipped to --vmax/--wmax')
        b.add(d, vc, wc)


def staircase(amplitude):
    n = int(round(RAMP_DURATION / RAMP_STEP / 2))
    up = [amplitude * i / n for i in range(1, n + 1)]
    return up + up[-2::-1]


def add_ramps(b, vmax, wmax, notes):
    # Scale the speed ramp down if its travel does not fit in the box.
    travel = sum(staircase(1.0)) * RAMP_STEP
    va = min(vmax, b.room() / travel)
    if va < vmax:
        notes.add(f'speed ramp limited to {va:.3f} m/s to stay in the box')
    for sign in (1, -1):
        for v in staircase(sign * va):
            b.add(RAMP_STEP, v, 0.0)
        b.rest()
    for sign in (1, -1):
        for w in staircase(sign * wmax):
            b.add(RAMP_STEP, 0.0, w)
        b.rest()


def random_duration(rng, low, high):
    return round(rng.randint(ticks(low), ticks(high)) * DT, 2)


def add_random(b, rng, vmax, wmax):
    t = 0.0
    while t < RANDOM_DURATION - 1e-9:
        d = random_duration(rng, *RANDOM_SEGMENT)
        for _ in range(RANDOM_ATTEMPTS):
            v, w = clean(rng.uniform(-vmax, vmax)), clean(rng.uniform(-wmax, wmax))
            if b.fits(d, v, w):
                break
        else:
            v, w = 0.0, clean(rng.uniform(-wmax, wmax))   # turning in place always fits
        b.add(d, v, w)
        t += d
        if rng.random() < RANDOM_REST_PROBABILITY:
            r = random_duration(rng, *RANDOM_REST)
            b.rest(r)
            t += r


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def turn_to(b, heading, w_ret):
    a = wrap(heading - b.pose[2])
    if abs(a) < RETURN_TOLERANCE:
        return
    d = snap(abs(a) / w_ret, 'ceil')
    b.add(d, 0.0, a / d)
    b.rest(1.0)


def add_return(b, vmax, wmax):
    """Drive back to the start pose of the sequence."""
    w_ret, v_ret = RETURN_FRACTION * wmax, RETURN_FRACTION * vmax
    x, y, _ = b.pose
    dist = math.hypot(x, y)
    if dist > RETURN_TOLERANCE:
        turn_to(b, math.atan2(-y, -x), w_ret)
        d = snap(dist / v_ret, 'ceil')
        b.add(d, dist / d, 0.0)
        b.rest(1.0)
    turn_to(b, 0.0, w_ret)


def generate(vmax=DEFAULT_VMAX, wmax=DEFAULT_WMAX, rounds=1, seed=0, box=1.5,
             track_width=TRACK_WIDTH, wheel_speed_limit=WHEEL_SPEED_LIMIT):
    """Return (segments, blocks, builder, notes). Raises ValueError if the limits cannot be met."""
    if vmax <= 0 or wmax <= 0 or box <= 0 or rounds < 1:
        raise ValueError('vmax, wmax, box must be positive and rounds >= 1')
    rng = random.Random(seed)
    b = Builder(box, track_width, wheel_speed_limit)
    notes = set()
    for r in range(1, rounds + 1):
        b.block(f'round {r}: rest')
        b.rest(FIRST_REST)
        b.block(f'round {r}: straight')
        add_straight(b, vmax, notes)
        b.block(f'round {r}: turn in place')
        add_turns(b, wmax)
        b.block(f'round {r}: turn while driving (each arc driven back)')
        add_arcs(b, vmax, wmax)
        b.block(f'round {r}: sysid steps')
        add_steps(b, vmax, wmax, notes)
        b.block(f'round {r}: ramps')
        add_ramps(b, vmax, wmax, notes)
        b.block(f'round {r}: random')
        add_random(b, rng, vmax, wmax)
        b.block(f'round {r}: return to start')
        add_return(b, vmax, wmax)
    b.block('end')
    b.rest(FIRST_REST)

    problems = validate(b.segments, box, track_width, wheel_speed_limit)
    if problems:
        raise ValueError('generated sequence breaks the constraints:\n  ' + '\n  '.join(problems))
    return b.segments, b.blocks, b, sorted(notes)


def to_yaml(segments, blocks, header):
    lines = [f'# {h}' if h else '#' for h in header]
    lines.append('segments:')
    starts = dict(blocks)
    for i, (d, v, w) in enumerate(segments):
        if i in starts:
            lines.append(f'  # {starts[i]}')
        lines.append(f'  - {{duration: {d:.2f}, v: {v}, w: {w}}}')
    return '\n'.join(lines) + '\n'


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--vmax', type=float, default=DEFAULT_VMAX, help='largest |v| [m/s] (default %(default)s)')
    p.add_argument('--wmax', type=float, default=DEFAULT_WMAX, help='largest |w| [rad/s] (default %(default)s)')
    p.add_argument('--rounds', type=int, default=1, help='number of rounds (default %(default)s)')
    p.add_argument('--seed', type=int, default=0, help='seed of the random block (default %(default)s)')
    p.add_argument('--box', type=float, required=True,
                   help='half-width [m] of the free square around the start pose, minus a safety margin')
    p.add_argument('--track-width', type=float, default=TRACK_WIDTH, help='b [m] (default %(default)s, unverified)')
    p.add_argument('--wheel-speed-limit', type=float, default=WHEEL_SPEED_LIMIT,
                   help='Kobuki wheel-speed limit [m/s] (default %(default)s, unverified)')
    p.add_argument('--out', required=True, help='output YAML file')
    a = p.parse_args(argv)

    if a.vmax > DEFAULT_VMAX or a.wmax > DEFAULT_WMAX:
        print(f'WARNING: above the sysid_steps.yaml amplitudes (v {DEFAULT_VMAX} m/s, w {DEFAULT_WMAX} rad/s). '
              'Raise them only after the Kobuki wheel-speed limit is measured (ARCHITECTURE.md 4.5).',
              file=sys.stderr)
    try:
        segments, blocks, b, notes = generate(a.vmax, a.wmax, a.rounds, a.seed, a.box,
                                              a.track_width, a.wheel_speed_limit)
    except ValueError as e:
        print(f'error: {e}', file=sys.stderr)
        return 1

    total = sum(d for d, _, _ in segments)
    header = [
        'Excitation sequence for training-data collection (F9).',
        'Generated by tools/excitation/make_sequence.py, do not edit by hand:',
        f'  make_sequence.py --vmax {a.vmax} --wmax {a.wmax} --rounds {a.rounds} --seed {a.seed} '
        f'--box {a.box} --track-width {a.track_width} --wheel-speed-limit {a.wheel_speed_limit}',
        f'{len(segments)} segments, {total:.2f} s. Commanded path stays within {b.max_extent:.2f} m '
        f'of the start (box {a.box} m), ends at ({b.pose[0]:.3f}, {b.pose[1]:.3f}) m.',
        'Each segment: duration [s], v [m/s], w [rad/s].',
    ] + [f'Note: {n}' for n in notes]
    with open(a.out, 'w', newline='\n') as f:
        f.write(to_yaml(segments, blocks, header))

    for n in notes:
        print(f'note: {n}')
    print(f'{a.out}: {len(segments)} segments, {total:.2f} s ({total / 60:.1f} min), '
          f'seed {a.seed}, path within {b.max_extent:.2f} m of the start')
    return 0


if __name__ == '__main__':
    sys.exit(main())
