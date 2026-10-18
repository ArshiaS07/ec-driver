#pragma once
/// @file
/// Decoding a call's result into the command's reply type.

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "ec/sdk/error.hpp"
#include "ec/sdk/json.hpp"

namespace ec::sdk {

/// Reply of calls documented as "success: true; failure: false".
struct Ok {
    bool operator==(const Ok&) const = default;
};

/// A yes/no answer that the controller sends as the number 0 or 1, not as
/// true/false (e.g. "collision occurred: 1; no collision: 0").
enum class Flag : std::uint8_t {
    Off = 0,  ///< 0: no, not enabled, not present
    On = 1,   ///< 1: yes, enabled, present
};

[[nodiscard]] constexpr bool is_valid(Flag f) noexcept { return f == Flag::Off || f == Flag::On; }
[[nodiscard]] constexpr bool is_on(Flag f) noexcept { return f == Flag::On; }

/// Decodes a call's result (already unwrapped from the controller's JSON string)
/// into T. Every check happens before any conversion, so decoding never throws: a
/// result of the wrong type or range is Error::Kind::BadReply.
///
/// Specialized below for Ok, bool, integers, floating point, std::string, enums
/// with an is_valid() overload, and std::array / std::vector of any of these.
/// A command whose reply has another shape defines its own decode(); one whose
/// shape the manual leaves unclear uses Json and passes the result on as it came.
template <typename T>
struct ReplyDecoder;

/// Reply types ReplyDecoder can decode.
template <typename T>
concept DecodableReply = requires(const Json& result) {
    { ReplyDecoder<T>::decode(result) } -> std::same_as<std::expected<T, Error>>;
};

/// true is Ok; false is Error::Kind::Refused.
template <>
struct ReplyDecoder<Ok> {
    static std::expected<Ok, Error> decode(const Json& r) {
        if (!r.is_boolean()) return bad_reply(std::format("expected true or false, got {}", r.dump()));
        if (!r.get<bool>()) return std::unexpected(Error{Error::Kind::Refused, 0, "controller returned false"});
        return Ok{};
    }
};

/// A true/false value, where false is a valid answer (e.g. "synchronized").
template <>
struct ReplyDecoder<bool> {
    static std::expected<bool, Error> decode(const Json& r) {
        if (!r.is_boolean()) return bad_reply(std::format("expected true or false, got {}", r.dump()));
        return r.get<bool>();
    }
};

/// An integer that fits T.
template <std::integral T>
struct ReplyDecoder<T> {
    static std::expected<T, Error> decode(const Json& r) {
        if (r.is_number_unsigned()) return checked(r.get<std::uint64_t>(), r);
        if (r.is_number_integer()) return checked(r.get<std::int64_t>(), r);
        return bad_reply(std::format("expected an integer, got {}", r.dump()));
    }

private:
    template <typename V>
    static std::expected<T, Error> checked(V v, const Json& r) {
        if (!std::in_range<T>(v)) return bad_reply(std::format("{} is out of range", r.dump()));
        return static_cast<T>(v);
    }
};

/// A number; integers are accepted and converted.
template <std::floating_point T>
struct ReplyDecoder<T> {
    static std::expected<T, Error> decode(const Json& r) {
        if (!r.is_number()) return bad_reply(std::format("expected a number, got {}", r.dump()));
        return r.get<T>();
    }
};

/// The result as it came: for replies whose shape the manual leaves unclear.
template <>
struct ReplyDecoder<Json> {
    static std::expected<Json, Error> decode(const Json& r) { return r; }
};

template <>
struct ReplyDecoder<std::string> {
    static std::expected<std::string, Error> decode(const Json& r) {
        if (!r.is_string()) return bad_reply(std::format("expected a string, got {}", r.dump()));
        return r.get<std::string>();
    }
};

/// An enum with an is_valid() overload (found by argument-dependent lookup);
/// integers it does not define are BadReply.
template <typename E>
    requires std::is_enum_v<E> && requires(E e) {
        { is_valid(e) } -> std::same_as<bool>;
    }
struct ReplyDecoder<E> {
    static std::expected<E, Error> decode(const Json& r) {
        const auto v = ReplyDecoder<std::underlying_type_t<E>>::decode(r);
        if (!v) return std::unexpected(v.error());
        const auto e = static_cast<E>(*v);
        if (!is_valid(e)) return bad_reply(std::format("{} is not a defined value", r.dump()));
        return e;
    }
};

/// An array of exactly N elements, e.g. a double[6] pose.
template <DecodableReply T, std::size_t N>
struct ReplyDecoder<std::array<T, N>> {
    static std::expected<std::array<T, N>, Error> decode(const Json& r) {
        if (!r.is_array() || r.size() != N)
            return bad_reply(std::format("expected an array of {} elements, got {}", N, r.dump()));
        std::array<T, N> out{};
        for (std::size_t i = 0; i < N; ++i) {
            auto v = ReplyDecoder<T>::decode(r[i]);
            if (!v) return bad_reply(std::format("element {}: {}", i, v.error().message));
            out[i] = std::move(*v);
        }
        return out;
    }
};

/// An array of any length, e.g. an int[length] list of register values.
template <DecodableReply T>
struct ReplyDecoder<std::vector<T>> {
    static std::expected<std::vector<T>, Error> decode(const Json& r) {
        if (!r.is_array()) return bad_reply(std::format("expected an array, got {}", r.dump()));
        std::vector<T> out;
        out.reserve(r.size());
        for (std::size_t i = 0; i < r.size(); ++i) {
            auto v = ReplyDecoder<T>::decode(r[i]);
            if (!v) return bad_reply(std::format("element {}: {}", i, v.error().message));
            out.push_back(std::move(*v));
        }
        return out;
    }
};

}  // namespace ec::sdk
