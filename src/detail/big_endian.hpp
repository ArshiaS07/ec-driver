#pragma once
// Sequential big-endian reader over a byte span (private to the library).
//
// The controller sends every field in network (big-endian) byte order. Each
// value is copied into an unsigned integer of the same size, byte-swapped on
// little-endian hosts (std::byteswap, C++23), and reinterpreted with
// std::bit_cast. No pointer casts into the buffer, so no alignment or aliasing
// issues.

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

namespace ec::detail {

template <std::size_t N>
struct uint_of_size;
template <>
struct uint_of_size<1> {
    using type = std::uint8_t;
};
template <>
struct uint_of_size<2> {
    using type = std::uint16_t;
};
template <>
struct uint_of_size<4> {
    using type = std::uint32_t;
};
template <>
struct uint_of_size<8> {
    using type = std::uint64_t;
};

class BigEndianReader {
public:
    explicit BigEndianReader(std::span<const std::uint8_t> bytes) noexcept : bytes_(bytes) {}

    // Precondition: remaining() >= sizeof(T). Callers check the total size once
    // up front instead of paying for a check per field.
    template <typename T>
        requires std::is_arithmetic_v<T>
    [[nodiscard]] T read() noexcept {
        using U = typename uint_of_size<sizeof(T)>::type;
        U raw{};
        std::memcpy(&raw, bytes_.data() + pos_, sizeof(T));
        if constexpr (std::endian::native == std::endian::little) raw = std::byteswap(raw);
        pos_ += sizeof(T);
        return std::bit_cast<T>(raw);
    }

    template <typename T, std::size_t N>
    [[nodiscard]] std::array<T, N> read_array() noexcept {
        std::array<T, N> out{};
        for (auto& v : out) v = read<T>();
        return out;
    }

    [[nodiscard]] std::size_t position() const noexcept { return pos_; }
    [[nodiscard]] std::size_t remaining() const noexcept { return bytes_.size() - pos_; }

private:
    std::span<const std::uint8_t> bytes_;
    std::size_t pos_ = 0;
};

}  // namespace ec::detail
