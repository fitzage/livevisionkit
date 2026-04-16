//    *************************** LiveVisionKit ****************************
//    Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
// 	  **********************************************************************

#pragma once

#include <atomic>
#include <LiveVisionKit.hpp>

#include "Interop/VisionFilter.hpp"

namespace lvk
{

	class VSFilter : public VisionFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		explicit VSFilter(obs_source_t* context);

		~VSFilter();

		void configure(obs_data_t* settings);

		bool validate() const;

	private:

        void filter(OBSFrame& frame) override;

		void draw_debug_hud(OBSFrame& frame);

        static bool on_crop_split(obs_properties_t* props, obs_property_t* property, obs_data_t* settings);

        static bool on_motion_profile_changed(obs_properties_t* props, obs_property_t* property, obs_data_t* settings);

        static bool on_delay_update(void* data, obs_properties_t* props, obs_property_t* property, obs_data_t* settings);

        // Fires on the main thread when the filter's parent source becomes visible
        // in the current scene.  Starts the scene-change PTZ cooldown.
        static void on_source_activate(void* data, calldata_t* cd);

	private:
		obs_source_t* m_Context = nullptr;

		StabilizationFilter m_Filter;
		bool m_TestMode = false;

        // PTZ override sources — combined in filter() each frame
        obs_hotkey_id          m_PtzHotkeyId         = OBS_INVALID_HOTKEY_ID;
        std::atomic<bool>      m_HotkeyPtzActive{false};
        std::atomic<uint32_t>  m_SceneChangeCooldown{0};  // frames remaining
        uint32_t               m_SceneChangeDelayFrames = 0;
        bool                   m_SignalConnected = false;
	};

}
