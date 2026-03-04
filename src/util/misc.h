// Extracted from src/deluge/util/misc.h

#pragma once
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace util {

template <auto Start, auto End, auto Inc, class F>
constexpr void constexpr_for(F&& f) {
	if constexpr (Start < End) {
		f.template operator()<Start>();
		constexpr_for<Start + Inc, End, Inc>(f);
	}
}

template <class Enum>
concept enumeration = std::is_enum_v<Enum>;

template <enumeration Enum>
constexpr std::underlying_type_t<Enum> to_underlying(Enum e) noexcept {
	return static_cast<std::underlying_type_t<Enum>>(e);
}

template <std::size_t n, typename T = int32_t>
constexpr T bit = T{1} << n;

template <std::size_t width, typename T = int32_t>
constexpr T bit_value = bit<width - 1, T>;

template <std::size_t width, typename T = int32_t>
constexpr T bitmask = bit<width, T> - 1;

template <typename T>
constexpr T bit_from_top(const std::size_t n) {
	return T{1} << (std::numeric_limits<T>::digits - (n + 1));
}

template <typename T>
constexpr T top_n_bits(const std::size_t n) {
	T output{0};
	for (std::size_t i = 0; i < n; i++) {
		output |= bit_from_top<T>(i);
	}
	return output;
}

template <typename T>
constexpr T top_bit = T{1} << (std::numeric_limits<T>::digits - 1);

template <std::integral T>
constexpr T median_value = (std::numeric_limits<T>::max() / 2) + (std::numeric_limits<T>::min() / 2) + 1;

template <typename T>
constexpr T set_top_bit(const T v) {
	return v | top_bit<T>;
}

template <typename T, std::size_t n>
constexpr T set_top_n_bits(const T v) {
	T output = v;
	for (std::size_t i = 0; i < n; i++) {
		output |= bit_from_top<T>(i);
	}
	return output;
}

template <typename T>
[[nodiscard]] constexpr T map(T x, T in_min, T in_max, T out_min, T out_max) {
	return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

template <std::integral T>
[[nodiscard]] constexpr T div_ceil(T a, T b) {
	return (a + (b - 1)) / b;
}

[[nodiscard]] constexpr bool infinite_a_lt_b(uint32_t a, uint32_t b) {
	return ((int32_t)(a - b) < 0);
}

} // namespace util

consteval uint_least64_t operator""_u64(unsigned long long arg) { return arg; }
consteval uint_least32_t operator""_u32(unsigned long long arg) { return arg; }
consteval uint_least16_t operator""_u16(unsigned long long arg) { return arg; }
consteval uint_least8_t  operator""_u8 (unsigned long long arg) { return arg; }

consteval int_least64_t operator""_i64(unsigned long long arg) { return arg; }
consteval int_least32_t operator""_i32(unsigned long long arg) { return arg; }
consteval int_least16_t operator""_i16(unsigned long long arg) { return arg; }
consteval int_least8_t  operator""_i8 (unsigned long long arg) { return arg; }

consteval std::byte operator""_b(unsigned long long arg) { return std::byte(arg); }
