"""Safety gate for camera-based palm control. No motor or GUI dependencies."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class PalmSafetyGate:
    """Arm only after an open palm holds still near the image center."""

    dwell_s: float = 0.6
    min_frames: int = 10
    still_radius: float = 0.05
    max_frame_jump: float = 0.12
    min_scale: float = 0.08
    armed: bool = False
    candidate: tuple[float, float] | None = None
    candidate_since: float = 0.0
    candidate_frames: int = 0
    previous: tuple[float, float] | None = None
    previous_frame_time: float = 0.0

    def reset(self) -> None:
        self.armed = False
        self.candidate = None
        self.candidate_since = 0.0
        self.candidate_frames = 0
        self.previous = None
        self.previous_frame_time = 0.0

    def update(
        self, *, detected: bool, gesture: str, px: float, py: float,
        scale: float, frame_time: float, frozen: bool, now: float,
    ) -> bool:
        valid = (
            detected and not frozen and gesture == "open"
            and all(math.isfinite(v) for v in (px, py, scale, frame_time))
            and 0.08 <= px <= 0.92 and 0.08 <= py <= 0.92
            and scale >= self.min_scale
        )
        if not valid:
            self.reset()
            return False

        pos = (px, py)
        fresh_frame = frame_time != self.previous_frame_time
        if (self.previous is not None and fresh_frame
                and math.dist(pos, self.previous) > self.max_frame_jump):
            self.reset()
            return False
        self.previous = pos
        self.previous_frame_time = frame_time

        if self.armed:
            return True
        if not (0.25 <= px <= 0.75 and 0.20 <= py <= 0.80):
            self.candidate = None
            self.candidate_frames = 0
            return False
        if self.candidate is None or math.dist(pos, self.candidate) > self.still_radius:
            self.candidate = pos
            self.candidate_since = now
            self.candidate_frames = 1
            return False
        if fresh_frame:
            self.candidate_frames += 1
        if (fresh_frame and self.candidate_frames >= self.min_frames
                and now - self.candidate_since >= self.dwell_s):
            self.armed = True
        return self.armed
