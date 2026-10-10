// LineFramer: however TCP chops the stream, whole lines come out in order.

#include "ec/command/line_framer.hpp"

#include <gtest/gtest.h>

#include <string>
#include <string_view>
#include <vector>

namespace {

using ec::command::LineFramer;

std::span<const std::uint8_t> bytes(std::string_view s) {
    return {reinterpret_cast<const std::uint8_t*>(s.data()), s.size()};
}

std::vector<std::string> drain(LineFramer& f) {
    std::vector<std::string> out;
    while (auto l = f.pop()) out.push_back(*l);
    return out;
}

TEST(LineFramer, NeedsTheNewline) {
    LineFramer f;
    f.push(bytes(R"({"id":1})"));
    EXPECT_FALSE(f.pop());
    f.push(bytes("\n"));
    EXPECT_EQ(drain(f), (std::vector<std::string>{R"({"id":1})"}));
}

TEST(LineFramer, SeveralLinesInOnePush) {
    LineFramer f;
    f.push(bytes("a\nb\nc\npartial"));
    EXPECT_EQ(drain(f), (std::vector<std::string>{"a", "b", "c"}));
    f.push(bytes("\n"));
    EXPECT_EQ(drain(f), (std::vector<std::string>{"partial"}));
}

TEST(LineFramer, ByteByByte) {
    LineFramer f;
    std::vector<std::string> seen;
    for (char c : std::string_view("one\ntwo\n")) {
        f.push(bytes(std::string_view(&c, 1)));
        for (auto& l : drain(f)) seen.push_back(l);
    }
    EXPECT_EQ(seen, (std::vector<std::string>{"one", "two"}));
}

TEST(LineFramer, StripsCarriageReturn) {
    LineFramer f;
    f.push(bytes("x\r\n"));
    EXPECT_EQ(drain(f), (std::vector<std::string>{"x"}));
}

TEST(LineFramer, EmptyLinesAreLines) {
    LineFramer f;
    f.push(bytes("\n\n"));
    EXPECT_EQ(drain(f), (std::vector<std::string>{"", ""}));
}

TEST(LineFramer, DropsOversizedLineAndRecovers) {
    LineFramer f;
    const std::string huge(LineFramer::kMaxLine + 100, 'x');
    f.push(bytes(huge));
    f.push(bytes("still the huge line\nnext\n"));
    EXPECT_EQ(drain(f), (std::vector<std::string>{"next"}));
    EXPECT_EQ(f.oversized_lines(), 1u);
}

TEST(LineFramer, OversizedLineDoesNotTakeCompleteLinesWithIt) {
    LineFramer f;
    f.push(bytes("before\n"));
    f.push(bytes(std::string(LineFramer::kMaxLine + 1, 'x')));
    f.push(bytes("\nafter\n"));
    EXPECT_EQ(drain(f), (std::vector<std::string>{"before", "after"}));
    EXPECT_EQ(f.oversized_lines(), 1u);
}

TEST(LineFramer, LineOfExactlyMaxLengthIsKept) {
    LineFramer f;
    const std::string longest(LineFramer::kMaxLine, 'x');
    f.push(bytes(longest + "\n"));
    EXPECT_EQ(drain(f), (std::vector<std::string>{longest}));
    EXPECT_EQ(f.oversized_lines(), 0u);
}

TEST(LineFramer, ResetDropsPartialLine) {
    LineFramer f;
    f.push(bytes("half"));
    f.reset();
    f.push(bytes("whole\n"));
    EXPECT_EQ(drain(f), (std::vector<std::string>{"whole"}));
}

}  // namespace
