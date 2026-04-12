#!/usr/bin/env python3
"""
Scalar simulation of PathSmoother to validate the asymmetric attack/release
rate fix.  No build required – pure Python / NumPy.

The C++ algorithm is replicated faithfully for the 1-D, single-mesh-point case
(which is what happens with the Homography subsystem and a 2x2 motion grid
when the camera motion is purely translational).

Usage:
    python3 test_smoother_sim.py
    python3 test_smoother_sim.py --plot      # also show a matplotlib chart
"""

import argparse
import math
import numpy as np
from collections import deque


# ---------------------------------------------------------------------------
# Core algorithm (mirrors PathSmoother.cpp)
# ---------------------------------------------------------------------------

def gaussian_kernel(size: int, sigma: float) -> np.ndarray:
    """Replicate cv::getGaussianKernel (normalized)."""
    if sigma <= 0:
        sigma = 0.3 * ((size - 1) * 0.5 - 1) + 0.8   # OpenCV default formula
    x = np.arange(size) - (size - 1) / 2.0
    kernel = np.exp(-0.5 * (x / sigma) ** 2)
    return kernel / kernel.sum()


def hysteresis(state, thresh_lower, state_lower, thresh_upper, state_upper):
    if state >= thresh_upper:
        return state_upper
    if state <= thresh_lower:
        return state_lower
    return state


def exp_moving_average(average, new_sample, rate):
    return average + rate * (new_sample - average)


class PathSmootherSim:
    """1-D scalar simulation of lvk::PathSmoother."""

    def __init__(
        self,
        predictive_samples: int = 10,
        corrective_limit: float = 0.1,
        smoothing_steps: float = 20.0,
        response_rate: float = 0.04,
        release_rate: float = 0.04,
    ):
        self.predictive_samples = predictive_samples
        self.corrective_limit = corrective_limit
        self.smoothing_steps = smoothing_steps
        self.response_rate = response_rate
        self.release_rate = release_rate

        window = 2 * predictive_samples + 1
        self.base_sigma = window / 12.0
        self.trajectory = deque([0.0] * window, maxlen=window)
        self.position = 0.0   # cumulative sum up to centre
        self.trace = 0.0
        self.smoothing_factor = 0.0

        # Re-compute position = sum of elements [0..centre_index]
        centre_index = predictive_samples
        self.position = sum(list(self.trajectory)[: centre_index + 1])

    def next(self, motion: float) -> float:
        traj = list(self.trajectory)
        centre_index = self.predictive_samples

        # Remove oldest, push new, update position with new centre
        self.position -= traj[0]
        self.trajectory.append(motion)
        traj = list(self.trajectory)
        self.position += traj[centre_index]

        # Build Gaussian filter
        window = len(traj)
        sigma = self.base_sigma + self.smoothing_factor
        filt = gaussian_kernel(window, sigma)

        # Compute m_Trace (cumulative Gaussian tail weighting)
        weight = 1.0
        self.trace = traj[0]
        for i in range(1, window):
            weight -= filt[i - 1]
            self.trace += weight * traj[i]

        path_correction = self.trace - self.position

        # Measure drift as fraction of corrective limit
        drift = abs(path_correction) / self.corrective_limit
        if drift > 1.0:
            path_correction = math.copysign(self.corrective_limit, path_correction)
            drift = 1.0

        # Asymmetric EMA: release_rate when returning to rest, response_rate when engaging
        target = hysteresis(drift, 0.3, self.smoothing_steps, 0.7, 0.0)
        rate = self.release_rate if target > self.smoothing_factor else self.response_rate
        self.smoothing_factor = exp_moving_average(self.smoothing_factor, target, rate)

        return path_correction


# ---------------------------------------------------------------------------
# Test scenarios
# ---------------------------------------------------------------------------

def run_scenario(label: str, smoother: PathSmootherSim, motions: list[float]):
    corrections = []
    smoothing_factors = []
    for m in motions:
        c = smoother.next(m)
        corrections.append(c)
        smoothing_factors.append(smoother.smoothing_factor)
    return corrections, smoothing_factors


def frames_until_settled(corrections, threshold=0.005, window=5):
    """Return the frame index at which |correction| stays below threshold for `window` frames."""
    for i in range(len(corrections) - window):
        if all(abs(corrections[i + j]) < threshold for j in range(window)):
            return i
    return len(corrections)  # never settled


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--plot", action="store_true", help="Show matplotlib chart")
    args = parser.parse_args()

    FPS = 30
    BUMP_SIZE = 0.15        # jerk magnitude (fraction of frame – 15 % travel)
    BUMP_DURATION = 3       # frames of motion (a short, sharp bump)
    PRE_FRAMES = 15         # stable frames before the bump
    POST_FRAMES = 90        # frames to observe recovery after bump

    total = PRE_FRAMES + BUMP_DURATION + POST_FRAMES

    # Motion signal: still → bump → still
    motions = [0.0] * PRE_FRAMES + [BUMP_SIZE] * BUMP_DURATION + [0.0] * POST_FRAMES

    configs = {
        "Original  (response=0.04, release=0.04)": dict(
            response_rate=0.04, release_rate=0.04
        ),
        "Transient (response=0.15, release=0.40)": dict(
            response_rate=0.15, release_rate=0.40
        ),
    }

    print(f"\n{'='*65}")
    print(f"  PathSmoother simulation – momentary bump test")
    print(f"  FPS={FPS}, bump size={BUMP_SIZE*100:.0f}% of frame, "
          f"duration={BUMP_DURATION} frames ({BUMP_DURATION*1000//FPS} ms)")
    print(f"{'='*65}\n")

    all_corrections = {}
    all_factors = {}

    for label, cfg in configs.items():
        sm = PathSmootherSim(
            predictive_samples=10,
            corrective_limit=0.1,
            smoothing_steps=20.0,
            **cfg,
        )
        corr, factors = run_scenario(label, sm, motions)
        all_corrections[label] = corr
        all_factors[label] = factors

        settle_frame = frames_until_settled(corr[PRE_FRAMES:]) + PRE_FRAMES
        settle_ms = (settle_frame - PRE_FRAMES) * 1000 // FPS

        peak = max(abs(c) for c in corr)

        print(f"  {label}")
        print(f"    Peak correction magnitude : {peak*100:.1f}% of frame")
        print(f"    Settled after bump        : {settle_frame - PRE_FRAMES} frames "
              f"({settle_ms} ms)")
        print()

    # -----------------------------------------------------------------------
    # Regression check
    # -----------------------------------------------------------------------
    orig_settle  = frames_until_settled(all_corrections["Original  (response=0.04, release=0.04)"][PRE_FRAMES:])
    trans_settle = frames_until_settled(all_corrections["Transient (response=0.15, release=0.40)"][PRE_FRAMES:])

    print(f"{'='*65}")
    if trans_settle < orig_settle:
        print(f"  PASS  Transient settles in {trans_settle} frames vs "
              f"{orig_settle} for original ({orig_settle - trans_settle} frames faster)")
    else:
        print(f"  FAIL  Transient did NOT settle faster ({trans_settle} vs {orig_settle})")
    print(f"{'='*65}\n")

    # -----------------------------------------------------------------------
    # Optional plot
    # -----------------------------------------------------------------------
    if args.plot:
        try:
            import matplotlib.pyplot as plt

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

            t = [i / FPS for i in range(total)]
            bump_start = PRE_FRAMES / FPS
            bump_end   = (PRE_FRAMES + BUMP_DURATION) / FPS

            colors = ["#e74c3c", "#2ecc71"]
            for (label, corr), color in zip(all_corrections.items(), colors):
                ax1.plot(t, [c * 100 for c in corr], label=label, color=color, lw=1.8)
            ax1.axvline(bump_start, color="gray", ls="--", lw=0.9, label="Bump start/end")
            ax1.axvline(bump_end,   color="gray", ls="--", lw=0.9)
            ax1.axhline(0, color="black", lw=0.5)
            ax1.set_ylabel("Path correction (% of frame)")
            ax1.set_title("PathSmoother response to a momentary camera bump")
            ax1.legend(fontsize=9)
            ax1.grid(True, alpha=0.3)

            for (label, factors), color in zip(all_factors.items(), colors):
                ax2.plot(t, factors, label=label, color=color, lw=1.8)
            ax2.axvline(bump_start, color="gray", ls="--", lw=0.9)
            ax2.axvline(bump_end,   color="gray", ls="--", lw=0.9)
            ax2.set_ylabel("m_SmoothingFactor (σ)")
            ax2.set_xlabel("Time (s)")
            ax2.legend(fontsize=9)
            ax2.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig("smoother_sim.png", dpi=150)
            print("  Chart saved to smoother_sim.png")
            plt.show()
        except ImportError:
            print("  matplotlib not available – skipping plot. "
                  "Install with: pip install matplotlib")


if __name__ == "__main__":
    main()
