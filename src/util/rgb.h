/*
 * Copyright © 2015-2023 Synthstrom Audible Limited
 *
 * This file is part of The Synthstrom Audible Deluge Firmware.
 *
 * The Synthstrom Audible Deluge Firmware is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 */

// Minimal standalone RGB class extracted from src/deluge/gui/colour/rgb.h.
// Only the subset of methods needed by the controller firmware is included.

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

class RGB {
public:
	using channel_type = uint8_t;

	static constexpr auto channel_min = std::numeric_limits<channel_type>::min();
	static constexpr auto channel_max = std::numeric_limits<channel_type>::max();

	channel_type r = 0;
	channel_type g = 0;
	channel_type b = 0;

	constexpr RGB() = default;
	constexpr RGB(channel_type r_, channel_type g_, channel_type b_) : r(r_), g(g_), b(b_) {}
	constexpr RGB& operator=(const RGB& other) = default;
	bool operator==(RGB const&) const = default;

	static constexpr RGB monochrome(uint8_t brightness) {
		return RGB{brightness, brightness, brightness};
	}

	[[nodiscard]] constexpr RGB dim(uint8_t level = 1) const {
		return RGB{(channel_type)(r >> level), (channel_type)(g >> level), (channel_type)(b >> level)};
	}

	constexpr channel_type& operator[](size_t idx) {
		switch (idx) {
		case 0:
			return r;
		case 1:
			return g;
		case 2:
			return b;
		default:
			__builtin_unreachable();
		}
	}

	static constexpr size_t size() { return 3; }
	constexpr channel_type* begin() { return &this->r; }
	constexpr channel_type* end() { return (&this->b + 1); }
};

static_assert(std::is_trivially_copyable_v<RGB>, "RGB must be trivially copyable");
