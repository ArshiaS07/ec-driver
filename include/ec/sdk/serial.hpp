#pragma once
/// @file
/// The controller's serial ports, SDK manual sections 2.2.14 (the hardware
/// port, RS232 or RS485) and 2.2.15 (the tool connector's TCI port). Each is a
/// session: open, configure, send and receive, close. Firmware 3.6.2 or later.

#include <expected>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "ec/sdk/error.hpp"
#include "ec/sdk/json.hpp"
#include "ec/sdk/reply.hpp"
#include "ec/sdk/request.hpp"

namespace ec::sdk {

/// What a receive returned: {"result", "size", "buf"}.
struct PortRead {
    bool ok = false;     ///< result
    int size = 0;        ///< bytes read
    std::string data{};  ///< buf: text, or hexadecimal digits in hex mode
    bool operator==(const PortRead&) const = default;
};

/// What a send returned: {"result", "size"}.
struct PortWritten {
    bool ok = false;  ///< result
    int size = 0;     ///< bytes sent
    bool operator==(const PortWritten&) const = default;
};

/// The settings both ports share; each left out keeps the controller's default.
struct PortSettings {
    std::optional<int> baud_rate{};  ///< baud_rate; default 9600
    std::optional<int> data_bits{};  ///< bits: 7 or 8 (TCI: 8); default 8
    std::optional<char> parity{};    ///< event: 'N', 'O' or 'E'; default 'N'
    std::optional<int> stop_bits{};  ///< stop: 1 or 2; default 1

    [[nodiscard]] Json json() const {
        Json p = Json::object();
        if (baud_rate) p["baud_rate"] = *baud_rate;
        if (data_bits) p["bits"] = *data_bits;
        if (parity) p["event"] = std::string(1, *parity);
        if (stop_bits) p["stop"] = *stop_bits;
        return p;
    }
};

namespace detail {
inline std::expected<PortRead, Error> decode_read(const Json& r) {
    if (!r.is_object() || !r.contains("result"))
        return bad_reply(std::format("expected result, size and buf, got {}", r.dump()));
    PortRead out;
    const auto ok = ReplyDecoder<bool>::decode(r["result"]);
    if (!ok) return std::unexpected(ok.error());
    out.ok = *ok;
    if (r.contains("size")) {
        const auto size = ReplyDecoder<int>::decode(r["size"]);
        if (!size) return std::unexpected(size.error());
        out.size = *size;
    }
    if (r.contains("buf")) {
        auto buf = ReplyDecoder<std::string>::decode(r["buf"]);
        if (!buf) return std::unexpected(buf.error());
        out.data = std::move(*buf);
    }
    return out;
}

inline std::expected<PortWritten, Error> decode_written(const Json& r) {
    const auto read = decode_read(r);
    if (!read) return std::unexpected(read.error());
    return PortWritten{read->ok, read->size};
}

inline Json receive_params(int count, std::optional<bool> hex, std::optional<int> timeout_ms) {
    Json p{{"count", count}};
    if (hex) p["hex"] = *hex ? 1 : 0;
    if (timeout_ms) p["timeout"] = *timeout_ms;
    return p;
}

inline Json send_params(const std::string& data, std::optional<bool> hex) {
    Json p{{"send_buf", data}};
    if (hex) p["hex"] = *hex ? 1 : 0;
    return p;
}
}  // namespace detail

/// §2.2.14.1 Opens the hardware serial port.
struct OpenSerialPort {
    static constexpr std::string_view method = "open_serial_port";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
    bool rs485 = false;  ///< device_type: 0 RS232, 1 RS485

    [[nodiscard]] Json params() const { return Json{{"device_type", rs485 ? 1 : 0}}; }
};

/// §2.2.14.2 Configures it.
struct SetoptSerialPort {
    static constexpr std::string_view method = "setopt_serial_port";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
    PortSettings settings{};

    [[nodiscard]] Json params() const { return settings.json(); }
};

/// §2.2.14.3 Receives up to `count` bytes.
struct RecvSerialPort {
    static constexpr std::string_view method = "recv_serial_port";
    static constexpr Kind kind = Kind::Session;
    using Reply = PortRead;
    int count = 0;                    ///< count, 0..256
    std::optional<bool> hex{};        ///< hex: hexadecimal instead of text
    std::optional<int> timeout_ms{};  ///< timeout; default 100

    [[nodiscard]] Json params() const { return detail::receive_params(count, hex, timeout_ms); }
    static std::expected<PortRead, Error> decode(const Json& r) { return detail::decode_read(r); }
};

/// §2.2.14.4 Sends up to 256 bytes.
struct SendSerialPort {
    static constexpr std::string_view method = "send_serial_port";
    static constexpr Kind kind = Kind::Session;
    using Reply = PortWritten;
    std::string data{};         ///< send_buf
    std::optional<bool> hex{};  ///< hex: `data` is hexadecimal digits

    [[nodiscard]] Json params() const { return detail::send_params(data, hex); }
    static std::expected<PortWritten, Error> decode(const Json& r) { return detail::decode_written(r); }
};

/// §2.2.14.5 Clears its buffer.
struct FlushSerialPort {
    static constexpr std::string_view method = "flush_serial_port";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
};

/// §2.2.14.6 Closes it.
struct CloseSerialPort {
    static constexpr std::string_view method = "close_serial_port";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
};

/// §2.2.15.1 Opens the TCI port.
struct OpenTci {
    static constexpr std::string_view method = "open_tci";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
};

/// §2.2.15.2 Configures it (8 data bits only).
struct SetoptTci {
    static constexpr std::string_view method = "setopt_tci";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
    PortSettings settings{};

    [[nodiscard]] Json params() const { return settings.json(); }
};

/// §2.2.15.3 Receives up to `count` bytes.
struct RecvTci {
    static constexpr std::string_view method = "recv_tci";
    static constexpr Kind kind = Kind::Session;
    using Reply = PortRead;
    int count = 0;                    ///< count, 0..256
    std::optional<bool> hex{};        ///< hex
    std::optional<int> timeout_ms{};  ///< timeout; default 100

    [[nodiscard]] Json params() const { return detail::receive_params(count, hex, timeout_ms); }
    static std::expected<PortRead, Error> decode(const Json& r) { return detail::decode_read(r); }
};

/// §2.2.15.4 Sends up to 256 bytes.
struct SendTci {
    static constexpr std::string_view method = "send_tci";
    static constexpr Kind kind = Kind::Session;
    using Reply = PortWritten;
    std::string data{};         ///< send_buf
    std::optional<bool> hex{};  ///< hex

    [[nodiscard]] Json params() const { return detail::send_params(data, hex); }
    static std::expected<PortWritten, Error> decode(const Json& r) { return detail::decode_written(r); }
};

/// §2.2.15.5 Clears its buffer.
struct FlushTci {
    static constexpr std::string_view method = "flush_tci";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
};

/// §2.2.15.6 Closes it.
struct CloseTci {
    static constexpr std::string_view method = "close_tci";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
};

static_assert(Command<OpenSerialPort>);
static_assert(Command<SetoptSerialPort>);
static_assert(Command<RecvSerialPort>);
static_assert(Command<SendSerialPort>);
static_assert(Command<FlushSerialPort>);
static_assert(Command<CloseSerialPort>);
static_assert(Command<OpenTci>);
static_assert(Command<SetoptTci>);
static_assert(Command<RecvTci>);
static_assert(Command<SendTci>);
static_assert(Command<FlushTci>);
static_assert(Command<CloseTci>);

}  // namespace ec::sdk
