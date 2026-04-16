//     *************************** LiveVisionKit ****************************
//     Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//     **********************************************************************

#pragma once

#include <atomic>
#include <opencv2/opencv.hpp>

#include "Math/WarpMesh.hpp"
#include "Data/StreamBuffer.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

    struct PathSmootherSettings
    {
        // NOTE: introduces time delay.
        size_t predictive_samples = 10;
        cv::Size motion_resolution = {2, 2};
        cv::Size2f corrective_limits = {0.1f, 0.1f};

        // Smoothing Characteristics
        float smoothing_steps = 20.0f;
        float response_rate = 0.04f;
        float release_rate = 0.04f;

        // Anchor mode: zero-latency EMA anchor suited for fixed/PTZ cameras.
        // All motion is treated as vibration and corrected back to a slowly-
        // drifting anchor point.  anchor_decay controls how fast the anchor
        // follows sustained (intentional) camera moves: lower = more stable
        // but slower to accept a deliberate PTZ repositioning.
        // anchor_snap_threshold: if per-frame motion (as a fraction of frame
        // size) exceeds this value, the anchor snaps to the current position
        // immediately — treating the move as intentional PTZ rather than
        // vibration.  Set lower to accept pans more eagerly; higher to
        // require a larger motion before snapping.
        bool  anchor_mode           = false;
        float anchor_decay          = 0.003f;
        float anchor_snap_threshold = 0.010f;
    };

    class PathSmoother final : public Configurable<PathSmootherSettings>
    {
    public:

        explicit PathSmoother(const PathSmootherSettings& settings = {});

        void configure(const PathSmootherSettings& settings) override;

        WarpMesh next(const WarpMesh& motion);

        // Signal that a deliberate PTZ move is in progress.  While active the
        // anchor snaps to the current position every frame, so no correction is
        // applied and the stabilizer does not fight the intentional reframe.
        // Safe to call from any thread (hotkey / UI thread).
        void set_ptz_active(bool active) noexcept;

        void restart();

        size_t time_delay() const;

        const WarpMesh& scene_crop() const;

        const cv::Rect2f& scene_margins() const;

    private:
        std::atomic<bool> m_PtzActive{false};

        double m_SmoothingFactor = 0.0f;
        double m_BaseSmoothingFactor = 0.0f;
        StreamBuffer<WarpMesh> m_Trajectory{1};
        WarpMesh m_Trace{WarpMesh::MinimumSize};
        WarpMesh m_Position{WarpMesh::MinimumSize};
        WarpMesh m_Anchor{WarpMesh::MinimumSize};

        cv::Rect2f m_SceneMargins{0,0,0,0};
        WarpMesh m_SceneCrop{WarpMesh::MinimumSize};
    };


}