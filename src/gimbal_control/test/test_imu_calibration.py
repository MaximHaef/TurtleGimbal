import numpy as np
import pytest

from gimbal_control.imu import ImuSample
from gimbal_control.imu_calibrate import load_raw, save_raw
from gimbal_control.imu_calibration import (
    ImuCalibration, calibrate, coverage, fit_accel, fit_gyro_bias, is_still, min_angle_deg,
    window_stats)

G = 9.80665
K_TRUE = np.array([[1.02, 0.015, -0.01],
                   [0.0, 0.98, 0.02],
                   [0.0, 0.0, 1.01]])
B_TRUE = np.array([0.12, -0.08, 0.25])
BG_TRUE = np.array([0.004, -0.002, 0.0015])


def random_dirs(n, rng):
    u = rng.normal(size=(n, 3))
    return u / np.linalg.norm(u, axis=1, keepdims=True)


def raw_from_true(a_true):
    """Invert a = K (a_raw - b)."""
    return np.linalg.solve(K_TRUE, a_true.T).T + B_TRUE


def test_fit_recovers_exact_parameters():
    rng = np.random.default_rng(0)
    A = raw_from_true(G * random_dirs(15, rng))
    K, b, stats = fit_accel(A, G)
    np.testing.assert_allclose(K, K_TRUE, atol=1e-8)
    np.testing.assert_allclose(b, B_TRUE, atol=1e-8)
    assert stats['norm_error_rms'] < 1e-8
    assert stats['norm_error_rms_raw'] > 0.05


def test_fit_with_noise_close():
    rng = np.random.default_rng(1)
    A = raw_from_true(G * random_dirs(20, rng)) + rng.normal(scale=0.003, size=(20, 3))
    K, b, stats = fit_accel(A, G)
    np.testing.assert_allclose(K, K_TRUE, atol=2e-3)
    np.testing.assert_allclose(b, B_TRUE, atol=1e-2)
    assert stats['norm_error_rms'] < 0.01
    assert 'accel_bias_std' in stats


def test_calibrated_norm_is_g_for_new_poses():
    rng = np.random.default_rng(2)
    K, b, _ = fit_accel(raw_from_true(G * random_dirs(12, rng)), G)
    cal = ImuCalibration(K, b)
    a = cal.accel(raw_from_true(G * random_dirs(50, rng)))
    np.testing.assert_allclose(np.linalg.norm(a, axis=1), G, atol=1e-7)


def test_too_few_poses_rejected():
    rng = np.random.default_rng(3)
    with pytest.raises(ValueError, match='at least 9'):
        fit_accel(raw_from_true(G * random_dirs(8, rng)), G)


def test_poses_about_one_axis_rejected():
    # Rotations about raw x only: gravity stays in the y-z plane, x is never excited.
    t = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    dirs = np.stack([np.zeros_like(t), np.cos(t), np.sin(t)], axis=1)
    assert coverage(dirs) < 1e-12
    with pytest.raises(ValueError, match='do not determine'):
        fit_accel(raw_from_true(G * dirs), G)


def test_gyro_bias_is_mean():
    G_means = BG_TRUE + np.array([[1e-4, 0, 0], [-1e-4, 0, 0], [0, 2e-4, -2e-4], [0, -2e-4, 2e-4]])
    bg, stats = fit_gyro_bias(G_means)
    np.testing.assert_allclose(bg, BG_TRUE, atol=1e-12)
    assert stats['gyro_bias_spread'][0] > 0


def test_calibrate_from_windows_and_apply():
    rng = np.random.default_rng(4)
    windows = []
    for a in raw_from_true(G * random_dirs(14, rng)):
        acc = a + rng.normal(scale=0.01, size=(300, 3))
        gyr = BG_TRUE + rng.normal(scale=0.002, size=(300, 3))
        windows.append(window_stats(acc, gyr))
    cal = calibrate(windows, G)
    np.testing.assert_allclose(cal.gyro_bias, BG_TRUE, atol=5e-4)
    s = cal.apply(ImuSample(tuple(BG_TRUE), tuple(raw_from_true(np.array([0.0, 0.0, G])))))
    np.testing.assert_allclose(s.gyro, 0.0, atol=5e-4)
    np.testing.assert_allclose(s.accel, [0.0, 0.0, G], atol=0.02)
    assert cal.apply(ImuSample((0.0, 0.0, 0.0), None)).accel is None


def test_yaml_roundtrip(tmp_path):
    cal = ImuCalibration(K_TRUE.copy(), B_TRUE.copy(), BG_TRUE.copy(), {'poses': 12})
    path = tmp_path / 'cal.yaml'
    cal.save(path)
    back = ImuCalibration.load(path)
    np.testing.assert_allclose(back.accel_matrix, K_TRUE)
    np.testing.assert_allclose(back.accel_bias, B_TRUE)
    np.testing.assert_allclose(back.gyro_bias, BG_TRUE)
    assert back.info['poses'] == 12


def test_raw_npz_roundtrip(tmp_path):
    rng = np.random.default_rng(5)
    samples = [(rng.normal(size=(10, 3)), rng.normal(size=(10, 3))) for _ in range(3)]
    windows = [window_stats(a, g) for a, g in samples]
    path = tmp_path / 'raw.npz'
    save_raw(path, windows, samples)
    back = load_raw(path)
    assert len(back) == 3
    np.testing.assert_allclose(back[1].accel_mean, windows[1].accel_mean)
    assert back[2].n == 10


def test_stillness_and_angle_checks():
    rng = np.random.default_rng(6)
    still = window_stats(G * np.array([0, 0, 1.0]) + rng.normal(scale=0.01, size=(100, 3)),
                         rng.normal(scale=0.002, size=(100, 3)))
    moving = window_stats(G * np.array([0, 0, 1.0]) + rng.normal(scale=0.5, size=(100, 3)),
                          rng.normal(scale=0.002, size=(100, 3)))
    assert is_still(still, 0.05, 0.01)
    assert not is_still(moving, 0.05, 0.01)
    assert min_angle_deg(np.array([0, 0, 1.0]), []) == float('inf')
    assert min_angle_deg(np.array([0, 0, 1.0]), [[0, 1.0, 0], [0, 0.1, 1.0]]) == pytest.approx(5.71, abs=0.01)
