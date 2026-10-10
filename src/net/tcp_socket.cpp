#include "ec/net/tcp_socket.hpp"

#include <system_error>
#include <utility>

// ---------------------------------------------------------------- platform shim
// Everything that differs between Winsock and BSD sockets lives in this block.
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

int last_error() { return WSAGetLastError(); }
bool in_progress(int e) { return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS; }
bool would_block(int e) { return e == WSAEWOULDBLOCK; }
bool interrupted(int e) { return e == WSAEINTR; }
void close_socket(socket_t s) { ::closesocket(s); }
bool set_nonblocking(socket_t s) {
    u_long on = 1;
    return ::ioctlsocket(s, static_cast<long>(FIONBIO), &on) == 0;  // FIONBIO is unsigned, the parameter long
}
int select_nfds(socket_t) { return 0; }  // ignored by Winsock
constexpr int kSendFlags = 0;

// Winsock must be initialised once per process before any socket call.
bool ensure_network() {
    struct Init {
        bool ok = false;
        Init() {
            WSADATA data;
            ok = ::WSAStartup(MAKEWORD(2, 2), &data) == 0;
        }
        ~Init() {
            if (ok) ::WSACleanup();
        }
    };
    static const Init init;
    return init.ok;
}
std::string resolve_message(int code) { return std::system_category().message(code); }
}  // namespace

#else
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>

namespace {
using socket_t = int;
constexpr socket_t kInvalid = -1;
using io_size = std::size_t;

int last_error() { return errno; }
bool in_progress(int e) { return e == EINPROGRESS; }
bool would_block(int e) { return e == EAGAIN || e == EWOULDBLOCK; }
bool interrupted(int e) { return e == EINTR; }
void close_socket(socket_t s) { ::close(s); }
bool set_nonblocking(socket_t s) {
    const int flags = ::fcntl(s, F_GETFL, 0);
    return flags >= 0 && ::fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
}
int select_nfds(socket_t s) { return s + 1; }
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;  // a closed peer must be an error, not SIGPIPE
#else
constexpr int kSendFlags = 0;
#endif
bool ensure_network() { return true; }
std::string resolve_message(int code) { return ::gai_strerror(code); }
}  // namespace
#endif

// ---------------------------------------------------------------- portable part
namespace ec::net {
namespace {

using Clock = std::chrono::steady_clock;

socket_t to_native(std::uintptr_t h) { return static_cast<socket_t>(h); }
std::uintptr_t to_handle(socket_t s) { return static_cast<std::uintptr_t>(s); }
const std::uintptr_t kInvalidHandle = to_handle(kInvalid);

std::unexpected<NetError> fail(NetError::Kind kind, int code = 0) { return std::unexpected(NetError{kind, code}); }

enum class Wait { Ready, Timeout, Error };

// Waits until the socket is readable (or writable), at most `timeout`.
// The socket is non-blocking, so every wait goes through here: one place
// decides how timeouts work on every platform.
Wait wait_for(socket_t s, bool for_write, std::chrono::milliseconds timeout, int& error) {
    fd_set set;
    fd_set except;
    FD_ZERO(&set);
    FD_ZERO(&except);
    FD_SET(s, &set);
    FD_SET(s, &except);  // Winsock reports a failed connect here
    timeval tv{};
    const auto ms = timeout.count() < 0 ? 0 : timeout.count();
    tv.tv_sec = static_cast<decltype(tv.tv_sec)>(ms / 1000);
    tv.tv_usec = static_cast<decltype(tv.tv_usec)>((ms % 1000) * 1000);

    const int n = ::select(select_nfds(s), for_write ? nullptr : &set, for_write ? &set : nullptr, &except, &tv);
    if (n > 0) return Wait::Ready;
    if (n == 0) return Wait::Timeout;
    error = last_error();
    // A signal (e.g. Ctrl+C) interrupts select on POSIX: report a timeout so the
    // caller's loop gets a chance to check its stop flag.
    return interrupted(error) ? Wait::Timeout : Wait::Error;
}

std::chrono::milliseconds remaining(Clock::time_point deadline) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
}

}  // namespace

std::string_view to_string(NetError::Kind k) noexcept {
    switch (k) {
        case NetError::Kind::Resolve: return "resolve";
        case NetError::Kind::Connect: return "connect";
        case NetError::Kind::Timeout: return "timeout";
        case NetError::Kind::Closed: return "closed by peer";
        case NetError::Kind::Io: return "socket error";
    }
    return "unknown";
}

std::string NetError::message() const {
    std::string text(to_string(kind));
    if (kind == Kind::Connect && code == 0) text += ": timed out";
    if (code != 0) {
        text += ": ";
        text += kind == Kind::Resolve ? resolve_message(code) : std::system_category().message(code);
    }
    return text;
}

std::expected<TcpSocket, NetError> TcpSocket::connect(const std::string& host, std::uint16_t port,
                                                      Timeout timeout) {
    if (!ensure_network()) return fail(NetError::Kind::Io);

    addrinfo hints{};
    hints.ai_family = AF_INET;  // the controller is IPv4
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    addrinfo* found = nullptr;
    const std::string service = std::to_string(port);
    if (const int rc = ::getaddrinfo(host.c_str(), service.c_str(), &hints, &found); rc != 0)
        return fail(NetError::Kind::Resolve, rc);

    const socket_t s = ::socket(found->ai_family, found->ai_socktype, found->ai_protocol);
    if (s == kInvalid) {
        const int e = last_error();
        ::freeaddrinfo(found);
        return fail(NetError::Kind::Io, e);
    }
    TcpSocket sock(to_handle(s));  // owns s from here: closed on every early return

    if (!set_nonblocking(s)) {
        ::freeaddrinfo(found);
        return fail(NetError::Kind::Io, last_error());
    }
    const int one = 1;
    ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof one);

    const int rc = ::connect(s, found->ai_addr, static_cast<socklen_t>(found->ai_addrlen));
    const int connect_error = rc == 0 ? 0 : last_error();
    ::freeaddrinfo(found);

    if (rc != 0) {
        if (!in_progress(connect_error)) return fail(NetError::Kind::Connect, connect_error);
        int wait_error = 0;
        switch (wait_for(s, /*for_write=*/true, timeout, wait_error)) {
            case Wait::Ready: break;
            case Wait::Timeout: return fail(NetError::Kind::Connect, 0);
            case Wait::Error: return fail(NetError::Kind::Connect, wait_error);
        }
        int so_error = 0;
        socklen_t len = sizeof so_error;
        ::getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_error), &len);
        if (so_error != 0) return fail(NetError::Kind::Connect, so_error);
    }
    return sock;
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
    const socket_t s = to_native(handle_);

    int wait_error = 0;
    switch (wait_for(s, /*for_write=*/false, timeout, wait_error)) {
        case Wait::Ready: break;
        case Wait::Timeout: return fail(NetError::Kind::Timeout);
        case Wait::Error: return fail(NetError::Kind::Io, wait_error);
    }

    const auto n = ::recv(s, reinterpret_cast<char*>(buffer.data()), static_cast<io_size>(buffer.size()), 0);
    if (n > 0) return static_cast<std::size_t>(n);
    if (n == 0) return fail(NetError::Kind::Closed);
    const int e = last_error();
    if (would_block(e) || interrupted(e)) return fail(NetError::Kind::Timeout);  // spurious wake-up
    return fail(NetError::Kind::Io, e);
}

std::expected<void, NetError> TcpSocket::send_all(std::span<const std::uint8_t> data, Timeout timeout) {
    if (!is_open()) return fail(NetError::Kind::Closed);
    const socket_t s = to_native(handle_);
    const auto deadline = Clock::now() + timeout;

    while (!data.empty()) {
        const auto n = ::send(s, reinterpret_cast<const char*>(data.data()), static_cast<io_size>(data.size()),
                              kSendFlags);
        if (n > 0) {
            data = data.subspan(static_cast<std::size_t>(n));
            continue;
        }
        const int e = last_error();
        if (!would_block(e) && !interrupted(e)) return fail(NetError::Kind::Io, e);

        int wait_error = 0;
        switch (wait_for(s, /*for_write=*/true, remaining(deadline), wait_error)) {
            case Wait::Ready: break;
            case Wait::Timeout: return fail(NetError::Kind::Timeout);
            case Wait::Error: return fail(NetError::Kind::Io, wait_error);
        }
    }
    return {};
}

}  // namespace ec::net
