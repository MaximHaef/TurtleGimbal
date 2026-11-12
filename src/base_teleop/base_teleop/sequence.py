"""Command sequences for excitation runs (F9 data collection)."""
import yaml


def load_sequence(path):
    """Load a list of segments from a YAML file.

    Format:
        segments:
          - {duration: 2.0, v: 0.0, w: 0.0}
          - {duration: 3.0, v: 0.2, w: 0.0}

    Returns a list of (duration [s], v [m/s], w [rad/s]).
    """
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    segments = []
    for i, seg in enumerate(data['segments']):
        duration = float(seg['duration'])
        if duration <= 0.0:
            raise ValueError(f'segment {i}: duration must be positive')
        segments.append((duration, float(seg.get('v', 0.0)), float(seg.get('w', 0.0))))
    if not segments:
        raise ValueError('sequence has no segments')
    return segments


def command_at(segments, t):
    """Return (index, v, w) of the segment active at time t since start, or None after the end."""
    if t < 0.0:
        return None
    end = 0.0
    for i, (duration, v, w) in enumerate(segments):
        end += duration
        if t < end:
            return i, v, w
    return None
