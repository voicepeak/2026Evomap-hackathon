"""回归：GUI 相机挑选（机械臂上的相机 = 配置索引，默认 0）。

现场问题：脚本把 index 1 命名成 "wrist"，但机械臂上的那路其实是 cam0；
于是面板大画面/腕部小窗显示的是别处的画面。现在：
- 机械臂相机 **优先按配置索引**（默认 0），别名/老默认 1 只当兜底；
- 面板大画面默认固定显示机械臂那一路，用户点过别的相机才跟着换。
"""
from rebot_capture.gui.cams import cam_key, pick_arm_camera, pick_other_camera, pick_preview_camera


def _cam(i, alias="", opened=True, name=None, fps=25.0):
    return {"index": i, "alias": alias, "opened": opened, "name": name or f"cam{i}",
            "fps_actual": fps}


def test_pick_arm_camera_prefers_configured_index():
    cams = [_cam(1, "wrist"), _cam(0, "cam0"), _cam(2, "operator")]
    # 现场：别名写着 wrist=1，但机械臂上的相机其实是 0
    assert cam_key(pick_arm_camera(cams, preferred_index=0)) == 0
    # 换台机器：配置成 1 就以 1 为准
    assert cam_key(pick_arm_camera(cams, preferred_index=1)) == 1


def test_pick_arm_camera_fallbacks():
    # 配置的索引没开 → 退到别名 wrist*
    cams = [_cam(1, "wrist"), _cam(2, "scene")]
    assert cam_key(pick_arm_camera(cams, preferred_index=0)) == 1
    # 也没别名 → 退到老默认 index 1
    cams2 = [_cam(1, ""), _cam(2, "scene")]
    assert cam_key(pick_arm_camera(cams2, preferred_index=0)) == 1
    # 都没有 → None
    assert pick_arm_camera([], preferred_index=0) is None
    # 只挑"已打开"的
    assert pick_arm_camera([_cam(0, opened=False)], preferred_index=0) is None


def test_pick_other_camera_prefers_gesture_index():
    cams = [_cam(0), _cam(2, "operator"), _cam(1, "scene")]
    assert cam_key(pick_other_camera(cams, exclude_key=0, gesture_index=2)) == 2
    assert cam_key(pick_other_camera(cams, exclude_key=0, gesture_index=None)) == 2
    assert cam_key(pick_other_camera(cams, exclude_key=2, gesture_index=2)) == 0


def test_pick_preview_defaults_to_arm_camera():
    cams = [_cam(1, "wrist"), _cam(0, "cam0")]
    # 用户没点过 → 显示机械臂那一路（0）
    assert cam_key(pick_preview_camera(cams, preview_key=None, arm_key=0)) == 0
    # 用户点过 1 → 跟着 1
    assert cam_key(pick_preview_camera(cams, preview_key=1, arm_key=0)) == 1
    # 点过的那路被关掉了 → 回到机械臂那一路
    cams2 = [_cam(1, "wrist", opened=False), _cam(0, "cam0")]
    assert cam_key(pick_preview_camera(cams2, preview_key=1, arm_key=0)) == 0
    # 机械臂相机不在了 → None（面板显示提示）
    assert pick_preview_camera([_cam(1, opened=False)], preview_key=None, arm_key=0) is None
