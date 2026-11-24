"""Static calibration of IMU A: accelerometer scale/misalignment + bias, gyro bias.

The IMU is held still in many orientations. In each still window the
accelerometer measures only gravity, so the calibrated mean must have length g:

    a     = K (a_raw - b_a)        K: 3x3 upper triangular, b_a: accel bias
    omega = omega_raw - b_g        b_g: gyro bias (mean of all still windows)

Why K is upper triangular: ||K d|| = ||R K d|| for every rotation R, so gravity
magnitude only determines K^T K (6 numbers), not a full K (9). Fixing K upper
triangular removes that rotation: the calibrated x axis stays along the raw x
axis, y stays in the raw x-y plane. Scale (diagonal) and axis
non-orthogonality (off-diagonal) remain. The mounting rotation sensor -> base
frame B is a separate calibration.

Fitting needs at least 9 poses (9 parameters); 12-20 poses spread over the
sphere are recommended. The gyro scale cannot be found at rest and is not
fitted.

numpy only, no ROS, so the fit also runs off the Pi.
"""
from dataclasses import dataclass, field

import numpy as np

STANDARD_GRAVITY = 9.80665  # m/s^2

_TRIU = np.triu_indices(3)   # order of the 6 K entries in the parameter vector


@dataclass
class ImuCalibration:
    accel_matrix: np.ndarray = field(default_factory=lambda: np.eye(3))  # K
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))   # b_a, m/s^2
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))    # b_g, rad/s
    info: dict = field(default_factory=dict)   # fit statistics, metadata (saved, not used)

    def accel(self, a_raw):
        """Calibrated acceleration; a_raw is (3,) or (N, 3)."""
        d = np.asarray(a_raw, dtype=float) - self.accel_bias
        return d @ self.accel_matrix.T

    def gyro(self, w_raw):
        return np.asarray(w_raw, dtype=float) - self.gyro_bias

    def apply(self, sample):
        """Calibrated copy of an ImuSample (accel stays None if None)."""
        gyro = tuple(float(x) for x in self.gyro(sample.gyro))
        accel = None if sample.accel is None else tuple(float(x) for x in self.accel(sample.accel))
        return sample._replace(gyro=gyro, accel=accel)

    # --- YAML -------------------------------------------------------------

    def to_dict(self):
        return {
            'model': 'accel = accel_matrix @ (accel_raw - accel_bias); gyro = gyro_raw - gyro_bias',
            'accel_matrix': [[float(x) for x in row] for row in self.accel_matrix],
            'accel_bias': [float(x) for x in self.accel_bias],
            'gyro_bias': [float(x) for x in self.gyro_bias],
            'info': self.info,
        }

    @classmethod
    def from_dict(cls, d):
        K = np.array(d['accel_matrix'], dtype=float)
        ba = np.array(d['accel_bias'], dtype=float)
        bg = np.array(d['gyro_bias'], dtype=float)
        if K.shape != (3, 3) or ba.shape != (3,) or bg.shape != (3,):
            raise ValueError('calibration file: accel_matrix must be 3x3, biases length 3')
        return cls(K, ba, bg, dict(d.get('info') or {}))

    def save(self, path):
        import yaml
        with open(path, 'w') as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False, default_flow_style=None)

    @classmethod
    def load(cls, path):
        import yaml
        with open(path, 'r') as f:
            return cls.from_dict(yaml.safe_load(f))


# --- still windows -------------------------------------------------------------

@dataclass
class Window:
    """Statistics of one still pose."""
    accel_mean: np.ndarray
    gyro_mean: np.ndarray
    accel_std: np.ndarray
    gyro_std: np.ndarray
    n: int


def window_stats(accel, gyro):
    accel = np.asarray(accel, dtype=float).reshape(-1, 3)
    gyro = np.asarray(gyro, dtype=float).reshape(-1, 3)
    if len(accel) < 2 or len(gyro) < 2:
        raise ValueError('window needs at least 2 samples')
    return Window(accel.mean(0), gyro.mean(0), accel.std(0), gyro.std(0), len(accel))


def is_still(w, max_accel_std, max_gyro_std):
    return bool(np.all(w.accel_std <= max_accel_std) and np.all(w.gyro_std <= max_gyro_std))


def min_angle_deg(direction, others):
    """Smallest angle between `direction` and the rows of `others` (inf if none)."""
    if len(others) == 0:
        return float('inf')
    u = direction / np.linalg.norm(direction)
    V = np.asarray(others, dtype=float)
    V = V / np.linalg.norm(V, axis=1, keepdims=True)
    return float(np.degrees(np.arccos(np.clip(V @ u, -1.0, 1.0)).min()))


def coverage(accel_means):
    """Smallest eigenvalue of mean(u u^T) over the gravity directions u.

    1/3 for a perfectly even spread over the sphere, 0 if some axis is never
    excited (e.g. all poses rotated only about one axis).
    """
    U = np.asarray(accel_means, dtype=float)
    U = U / np.linalg.norm(U, axis=1, keepdims=True)
    return float(np.linalg.eigvalsh(U.T @ U / len(U))[0])


# --- fits ----------------------------------------------------------------------

def _unpack(p):
    K = np.zeros((3, 3))
    K[_TRIU] = p[:6]
    return K, p[6:9]


def _residuals_jacobian(p, A, g):
    K, b = _unpack(p)
    D = A - b                       # (N, 3)
    Y = D @ K.T                     # (N, 3)
    norm = np.linalg.norm(Y, axis=1)
    r = norm - g
    Nrm = Y / norm[:, None]         # unit vectors n_i
    J = np.empty((len(A), 9))
    for col, (j, k) in enumerate(zip(*_TRIU)):
        J[:, col] = Nrm[:, j] * D[:, k]      # d r / d K_jk = n_j d_k
    J[:, 6:9] = -Nrm @ K                      # d r / d b = -K^T n
    return r, J


def fit_accel(accel_means, g=STANDARD_GRAVITY, max_iter=200, tol=1e-12):
    """Fit K (upper triangular) and b_a so that ||K (a_i - b_a)|| = g.

    accel_means: (N, 3) mean raw acceleration of each still pose, N >= 9.
    Returns (K, b_a, stats). Raises ValueError if the poses do not determine
    all 9 parameters.
    """
    A = np.asarray(accel_means, dtype=float).reshape(-1, 3)
    if len(A) < 9:
        raise ValueError(f'need at least 9 poses for 9 parameters, got {len(A)}')

    s = g / np.linalg.norm(A, axis=1).mean()
    p = np.concatenate([(s * np.eye(3))[_TRIU], np.zeros(3)])
    r, J = _residuals_jacobian(p, A, g)
    cost = r @ r
    lam = 1e-3
    for _ in range(max_iter):
        H = J.T @ J
        step = np.linalg.solve(H + lam * np.diag(np.diag(H)), -J.T @ r)
        p_new = p + step
        r_new, J_new = _residuals_jacobian(p_new, A, g)
        cost_new = r_new @ r_new
        if cost_new < cost:
            converged = cost - cost_new <= tol * max(cost, 1e-30) or np.linalg.norm(step) < 1e-12
            p, r, J, cost = p_new, r_new, J_new, cost_new
            lam = max(lam / 10, 1e-12)
            if converged:
                break
        else:
            lam *= 10
            if lam > 1e12:
                break

    K, b = _unpack(p)
    if np.any(np.diag(K) < 0):     # same ||K d|| with a row's sign flipped; keep scales positive
        K = np.diag(np.sign(np.diag(K))) @ K

    sv = np.linalg.svd(J, compute_uv=False)
    cond = float(sv[0] / sv[-1]) if sv[-1] > 0 else float('inf')
    if cond > 1e6:
        raise ValueError(f'poses do not determine all parameters (condition {cond:.3g}); '
                         'add poses in more different orientations')

    dof = len(A) - 9
    rms = float(np.sqrt(cost / len(A)))
    std = None
    if dof > 0:
        cov = (cost / dof) * np.linalg.inv(J.T @ J)
        std = np.sqrt(np.diag(cov))
    raw_err = np.linalg.norm(A, axis=1) - g
    stats = {
        'poses': int(len(A)),
        'gravity': float(g),
        'norm_error_rms_raw': float(np.sqrt(np.mean(raw_err ** 2))),      # m/s^2, before
        'norm_error_rms': rms,                                             # m/s^2, after
        'norm_error_max': float(np.abs(r).max()),
        'condition': cond,
        'coverage': coverage(A),
    }
    if std is not None:
        Kstd, bstd = _unpack(std)
        stats['accel_matrix_std'] = [[float(x) for x in row] for row in Kstd]
        stats['accel_bias_std'] = [float(x) for x in bstd]
    return K, b, stats


def fit_gyro_bias(gyro_means):
    """Gyro bias = mean over the still windows; also their spread per axis.

    The spread shows how stable the bias is across the session (and includes
    the Earth rate, < 7.3e-5 rad/s, projected differently in each pose).
    """
    G = np.asarray(gyro_means, dtype=float).reshape(-1, 3)
    if len(G) == 0:
        raise ValueError('no still windows')
    spread = G.std(0, ddof=1) if len(G) > 1 else np.zeros(3)
    return G.mean(0), {'gyro_bias_spread': [float(x) for x in spread]}


def calibrate(windows, g=STANDARD_GRAVITY):
    """Full calibration from a list of still Windows."""
    K, ba, astats = fit_accel([w.accel_mean for w in windows], g)
    bg, gstats = fit_gyro_bias([w.gyro_mean for w in windows])
    return ImuCalibration(K, ba, bg, {**astats, **gstats})
