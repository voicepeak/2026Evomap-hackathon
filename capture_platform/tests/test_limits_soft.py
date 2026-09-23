"""回归：限位排斥减弱 + 安全盒软边界（消除"死区 / 反弹"）。

- limit_repulse：最大回推速度 ≤0.15 rad/s，作用带 3°；离限位 4° 以外不再回推
- soft_box_velocity：接近边界按剩余空间成比例减速（可慢慢贴边，不越界）；
  半径上限只削"向外"的径向分量，切向保留（能沿弧线滑动，不卡死）
"""
import numpy as np


def test_limit_repulsion_is_gentle_and_narrow(core_env):
    import teleop_core as tc
    lo = np.zeros(6)
    hi = np.full(6, np.pi)
    assert tc.LIMIT_K <= 0.15, "回推速度应已减弱"
    assert tc.LIMIT_SOFT <= np.radians(3.5), "作用带应已收窄"

    z = tc.limit_repulse(np.zeros(6), lo, hi)       # 正好在限位上
    assert np.allclose(z, tc.LIMIT_K, atol=1e-9)
    assert np.allclose(tc.limit_repulse(np.full(6, np.radians(4.0)), lo, hi), 0.0)


def test_soft_box_scales_velocity_before_wall(core_env):
    import teleop_core as tc
    v = np.array([0.08, 0.0, 0.0])
    near = np.array([tc.BX[1] - 0.02, 0.0, 0.30])   # 距 +X 边界 2cm（缓冲带 4cm）
    out = tc.soft_box_velocity(v, near, band=0.04)
    assert np.isclose(out[0], 0.08 * 0.02 / 0.04)
    assert 0.0 < out[0] < v[0]

    far = np.array([tc.BX[0] + 0.05, 0.0, 0.30])    # 缓冲带以外
    assert np.allclose(tc.soft_box_velocity(v, far, band=0.04), v)

    outside = np.array([tc.BX[1] + 0.01, 0.0, 0.30])   # 已在界外
    assert tc.soft_box_velocity(np.array([0.08, 0.0, 0.0]), outside)[0] == 0.0
    assert np.isclose(tc.soft_box_velocity(np.array([-0.08, 0.0, 0.0]), outside)[0], -0.08)


def test_soft_box_radius_keeps_tangential_motion(core_env):
    import teleop_core as tc
    p = np.array([0.42, 0.16, 0.30])                # r ≈ 0.449 → 进入 RMAX 缓冲带
    v = np.array([0.05, 0.05, 0.0])
    out = tc.soft_box_velocity(v, p, band=0.04)
    r = float(np.hypot(p[0], p[1]))
    tangent = np.array([-p[1], p[0]]) / r
    assert np.linalg.norm(out[:2]) < np.linalg.norm(v[:2])
    assert np.isclose(float(np.dot(out[:2], tangent)), float(np.dot(v[:2], tangent)))
