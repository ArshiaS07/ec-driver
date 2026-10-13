#include "ec/command/all.hpp"

#include <algorithm>
#include <array>

namespace ec::command {
namespace {

template <Command C>
std::expected<void, Error> check(const Json& result) {
    const auto reply = decode_reply<C>(result);
    if (!reply) return std::unexpected(reply.error());
    return {};
}

template <Command C>
Request example() {
    return to_request(C{});
}

template <Command... Cs>
constexpr std::array<CommandInfo, sizeof...(Cs)> table(CommandList<Cs...> /*unused*/) {
    return {CommandInfo{Cs::method, Cs::kind, &check<Cs>, &example<Cs>}...};
}

}  // namespace

std::span<const CommandInfo> all_commands() noexcept {
    static constexpr auto kTable = table(AllCommands{});
    return kTable;
}

const CommandInfo* find_command(std::string_view method) noexcept {
    const auto commands = all_commands();
    const auto it = std::ranges::find(commands, method, &CommandInfo::method);
    return it == commands.end() ? nullptr : &*it;
}

}  // namespace ec::command
