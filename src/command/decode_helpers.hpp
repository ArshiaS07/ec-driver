#pragma once
// Private helpers for the commands' decode() functions: turn an unwrapped
// result into a typed value, or a BadReply error that says what was expected.
// None of them throws: every type is checked before nlohmann's get<>().

#include <array>
#include <cstddef>
#include <expected>
#include <format>
#include <string>
#include <type_traits>
#include <utility>

#include "ec/command/request.hpp"
#include "ec/status.hpp"

namespace ec::command::detail {

inline std::unexpected<Error> bad_reply(std::string what) {
    return std::unexpected(Error{Error::Kind::BadReply, 0, std::move(what)});
}

// "true" → Ok, "false" → Error::Kind::Refused. Calls documented as
// "Success: true; failure: false" use this, so failure has one channel.
inline std::expected<Ok, Error> as_ok(const Json& r) {
    if (!r.is_boolean()) return bad_reply(std::format("expected true/false, got {}", r.dump()));
    if (!r.get<bool>()) return std::unexpected(Error{Error::Kind::Refused, 0, "controller returned false"});
    return Ok{};
}

inline std::expected<double, Error> as_double(const Json& r) {
    if (!r.is_number()) return bad_reply(std::format("expected a number, got {}", r.dump()));
    return r.get<double>();
}

inline std::expected<long long, Error> as_integer(const Json& r) {
    if (!r.is_number_integer()) return bad_reply(std::format("expected an integer, got {}", r.dump()));
    return r.get<long long>();
}

inline std::expected<std::string, Error> as_string(const Json& r) {
    if (!r.is_string()) return bad_reply(std::format("expected a string, got {}", r.dump()));
    return r.get<std::string>();
}

template <std::size_t N>
std::expected<std::array<double, N>, Error> as_doubles(const Json& r) {
    if (!r.is_array() || r.size() != N)
        return bad_reply(std::format("expected an array of {} numbers, got {}", N, r.dump()));
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
        if (!r[i].is_number()) return bad_reply(std::format("element {} is not a number: {}", i, r.dump()));
        out[i] = r[i].get<double>();
    }
    return out;
}

// Integer → enum, rejecting values the enum does not define.
template <typename E>
    requires std::is_enum_v<E>
std::expected<E, Error> as_enum(const Json& r) {
    const auto v = as_integer(r);
    if (!v) return std::unexpected(v.error());
    if (!std::in_range<std::underlying_type_t<E>>(*v)) return bad_reply(std::format("value {} is out of range", *v));
    const auto e = static_cast<E>(*v);
    if (!is_valid(e)) return bad_reply(std::format("value {} is out of range", *v));
    return e;
}

}  // namespace ec::command::detail
