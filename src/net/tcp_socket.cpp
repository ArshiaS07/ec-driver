#include "ec/net/tcp_socket.hpp"

#include <algorithm>
#include <climits>
#include <memory>
#include <optional>
#include <system_error>
#include <utility>

#include "detail/deadline.hpp"

namespace {

enum class Direction { Read, Write };
enum class Wait { Ready, Timeout, Error };

}  // namespace

// ---------------------------------------------------------------- platform shim
// Everything that differs between Winsock and POSIX sockets lives in this block.
// The rest of the file is written once against these names.
#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

namespace {
using socket_t = SOCKET;
constexpr socket_t kInvalid = INVALID_SOCKET;
using io_size = int;  // send/recv lengths are int on Windows
constexpr int kTimedOut = WSAETIMEDOUT;
constexpr int kInvalidArgument = WSAEINVAL;
constexpr int kSendFlags = 0;

int last_error() { return WSAGetLastError(); }
bool in_progress(int e) { return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS; }
bool would_block(int e) { return e == WSAEWOULDBLOCK; }
bool interrupted(int e) { return e == WSAEINTR; }
bool connection_lost(int e) { return e == WSAECONNRESET || e == WSAECONNABORTED || e == WSAESHUTDOWN; }
void close_socket(socket_t s) { ::closesocket(s); }
bool set_nonblocking(socket_t s) {
    u_long on = 1;
    return ::ioctlsocket(s, static_cast<long>(FIONBIO), &on) == 0;  // FIONBIO is unsigned, the parameter long
}
std::string resolve_message(int code) { return std::system_category().message(code); }

// Winsock must be initialised once per process before any socket call.
// Returns 0, or the WSAStartup error.
int network_error() {
    struct Init {
        int error;
        Init() {
            WSADATA data;
            error = ::WSAStartup(MAKEWORD(2, 2), &data);
        }
        ~Init() {
            if (error == 0) ::WSACleanup();
        }
    };
    static const Init init;
    return init.error;
}

// select(), not WSAPoll(): WSAPoll does not report a failed connect on Windows
// before 10 version 2004. Winsock's fd_set holds handles rather than bits, so
// select() has no descriptor-number limit here.
Wait wait_for(socket_t s, Direction d, int timeout_ms, int& error) {
    fd_set ready;
    fd_set failed;
    FD_ZERO(&ready);
    FD_ZERO(&failed);
    FD_SET(s, &ready);
    FD_SET(s, &failed);  // Winsock reports a failed connect here
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    const bool write = d == Direction::Write;
    const int n = ::select(0, write ? nullptr : &ready, write ? &ready : nullptr, &failed, &tv);
    if (n > 0) return Wait::Ready;
    if (n == 0) return Wait::Timeout;
    error = last_error();
    return Wait::Error;
}
}  // namespace

#else
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>

namespace {
using socket_t = int;
constexpr socket_t kInvalid = -1;
using io_size = std::size_t;
constexpr int kTimedOut = ETIMEDOUT;
constexpr int kInvalidArgument = EINVAL;
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;  // a closed peer must be an error, not SIGPIPE
#else
constexpr int kSendFlags = 0;
#endif

int last_error() { return errno; }
bool in_progress(int e) { return e == EINPROGRESS; }
bool would_block(int e) { return e == EAGAIN || e == EWOULDBLOCK; }
bool interrupted(int e) { return e == EINTR; }
bool connection_lost(int e) { return e == ECONNRESET || e == EPIPE || e == ECONNABORTED; }
void close_socket(socket_t s) { ::close(s); }
bool set_nonblocking(socket_t s) {
    const int flags = ::fcntl(s, F_GETFL, 0);
    return flags >= 0 && ::fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
}
std::string resolve_message(int code) { return ::gai_strerror(code); }
int network_error() { return 0; }

// poll(), not select(): select() cannot watch a descriptor numbered FD_SETSIZE
// (1024) or above, which a process with many open files reaches.
Wait wait_for(socket_t s, Direction d, int timeout_ms, int& error) {
    pollfd p{};
    p.fd = s;
    p.events = d == Direction::Write ? POLLOUT : POLLIN;
    const int n = ::poll(&p, 1, timeout_ms);
    if (n > 0) return Wait::Ready;  // also on POLLERR/POLLHUP: the following call reports the error
    if (n == 0) return Wait::Timeout;
    error = last_error();
    return Wait::Error;
}
}  // namespace
#endif

// ---------------------------------------------------------------- portable part
namespace ec::net {
namespace {

using detail::Clock;
using detail::deadline_after;

constexpr std::size_t kMaxIo = std::size_t{1} << 30;  // bytes per send/recv call; larger transfers loop

constexpr socket_t to_native(std::uintptr_t h) noexcept { return static_cast<socket_t>(h); }
constexpr std::uintptr_t to_handle(socket_t s) noexcept { return static_cast<std::uintptr_t>(s); }
constexpr std::uintptr_t kInvalidHandle = to_handle(kInvalid);

struct FreeAddrinfo {
    void operator()(addrinfo* a) const noexcept { ::freeaddrinfo(a); }
};

std::unexpected<NetError> fail(NetError::Kind kind, int code = 0) { return std::unexpected(NetError{kind, code}); }

// Waits until the socket is ready or `deadline` passes. Interrupted and early
// returns are retried, so Timeout means the deadline has passed.
Wait wait_until(socket_t s, Direction d, Clock::time_point deadline, int& error) {
    for (;;) {
        const auto left = detail::time_left(deadline).count();
        const int ms = static_cast<int>(std::min<decltype(left)>(left, INT_MAX));
        const Wait w = wait_for(s, d, ms, error);
        if (w == Wait::Ready) return w;
        if (w == Wait::Error && !interrupted(error)) return w;
        if (Clock::now() >= deadline) return Wait::Timeout;
    }
}

// Makes `s` non-blocking, enables TCP_NODELAY and connects it to `a` before
// `deadline`. Returns the error, or nothing once connected.
std::optional<NetError> establish(socket_t s, const addrinfo& a, Clock::time_point deadline) {
    if (!set_nonblocking(s)) return NetError{NetError::Kind::Io, last_error()};
    const int one = 1;
    ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof one);

    if (::connect(s, a.ai_addr, static_cast<socklen_t>(a.ai_addrlen)) == 0) return std::nullopt;
    if (const int e = last_error(); !in_progress(e)) return NetError{NetError::Kind::Connect, e};

    int wait_error = 0;
    switch (wait_until(s, Direction::Write, deadline, wait_error)) {
        case Wait::Ready: break;
        case Wait::Timeout: return NetError{NetError::Kind::Connect, kTimedOut};
        case Wait::Error: return NetError{NetError::Kind::Connect, wait_error};
    }
    int so_error = 0;
    socklen_t len = sizeof so_error;
    if (::getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_error), &len) != 0)
        return NetError{NetError::Kind::Connect, last_error()};
    if (so_error != 0) return NetError{NetError::Kind::Connect, so_error};
    return std::nullopt;
}

}  // namespace

std::string_view to_string(NetError::Kind k) noexcept {
    switch (k) {
        case NetError::Kind::Resolve: return "resolve";
        case NetError::Kind::Connect: return "connect";
        case NetError::Kind::Timeout: return "timeout";
        case NetError::Kind::Closed: return "connection closed";
        case NetError::Kind::Io: return "socket error";
    }
    return "unknown";
}

std::string NetError::message() const {
    std::string text(to_string(kind));
    if (code != 0) {
        text += ": ";
        text += kind == Kind::Resolve ? resolve_message(code) : std::system_category().message(code);
    }
    return text;
}

std::expected<TcpSocket, NetError> TcpSocket::connect(const std::string& host, std::uint16_t port, Timeout timeout) {
    if (const int e = network_error(); e != 0) return fail(NetError::Kind::Io, e);
    const auto deadline = deadline_after(timeout);

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;  // IPv4 or IPv6, whatever the host resolves to
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    addrinfo* found = nullptr;
    const std::string service = std::to_string(port);
    if (const int rc = ::getaddrinfo(host.c_str(), service.c_str(), &hints, &found); rc != 0)
        return fail(NetError::Kind::Resolve, rc);
    const std::unique_ptr<addrinfo, FreeAddrinfo> addresses(found);

    NetError last{NetError::Kind::Connect, 0};
    for (const addrinfo* a = addresses.get(); a != nullptr; a = a->ai_next) {
        const socket_t s = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (s == kInvalid) {
            last = NetError{NetError::Kind::Io, last_error()};
            continue;
        }
        TcpSocket sock(to_handle(s));  // closes s on every path that does not return it
        const auto error = establish(s, *a, deadline);
        if (!error) return sock;
        last = *error;
        if (Clock::now() >= deadline) break;
    }
    return std::unexpected(last);
}

TcpSocket::TcpSocket(TcpSocket&& other) noexcept : handle_(std::exchange(other.handle_, kInvalidHandle)) {}

TcpSocket& TcpSocket::operator=(TcpSocket&& other) noexcept {
    if (this != &other) {
        close();
        handle_ = std::exchange(other.handle_, kInvalidHandle);
    }
    return *this;
}

TcpSocket::~TcpSocket() { close(); }

bool TcpSocket::is_open() const noexcept { return handle_ != kInvalidHandle; }

void TcpSocket::close() noexcept {
    if (is_open()) {
        close_socket(to_native(handle_));
        handle_ = kInvalidHandle;
    }
}

std::expected<std::size_t, NetError> TcpSocket::receive(std::span<std::uint8_t> buffer, Timeout timeout) {
    if (!is_open()) return fail(NetError::Kind::Closed);
    if (buffer.empty()) return fail(NetError::Kind::Io, kInvalidArgument);
    const socket_t s = to_native(handle_);
    const auto deadline = deadline_after(timeout);
    const auto size = static_cast<io_size>(std::min(buffer.size(), kMaxIo));

    for (;;) {
        int wait_error = 0;
        switch (wait_until(s, Direction::Read, deadline, wait_error)) {
            case Wait::Ready: break;
            case Wait::Timeout: return fail(NetError::Kind::Timeout);
            case Wait::Error: return fail(NetError::Kind::Io, wait_error);
        }
        const auto n = ::recv(s, reinterpret_cast<char*>(buffer.data()), size, 0);
        if (n > 0) return static_cast<std::size_t>(n);
        if (n == 0) return fail(NetError::Kind::Closed);
        const int e = last_error();
        if (connection_lost(e)) return fail(NetError::Kind::Closed, e);
        if (!would_block(e) && !interrupted(e)) return fail(NetError::Kind::Io, e);
        // Woken without data after all: wait again, up to the same deadline.
    }
}

std::expected<void, NetError> TcpSocket::send_all(std::span<const std::uint8_t> data, Timeout timeout) {
    if (!is_open()) return fail(NetError::Kind::Closed);
    const socket_t s = to_native(handle_);
    const auto deadline = deadline_after(timeout);

    while (!data.empty()) {
        const auto size = static_cast<io_size>(std::min(data.size(), kMaxIo));
        const auto n = ::send(s, reinterpret_cast<const char*>(data.data()), size, kSendFlags);
        if (n > 0) {
            data = data.subspan(static_cast<std::size_t>(n));
            continue;
        }
        const int e = last_error();
        if (connection_lost(e)) return fail(NetError::Kind::Closed, e);
        if (!would_block(e) && !interrupted(e)) return fail(NetError::Kind::Io, e);

        int wait_error = 0;
        switch (wait_until(s, Direction::Write, deadline, wait_error)) {
            case Wait::Ready: break;
            case Wait::Timeout: return fail(NetError::Kind::Timeout);
            case Wait::Error: return fail(NetError::Kind::Io, wait_error);
        }
    }
    return {};
}

}  // namespace ec::net
