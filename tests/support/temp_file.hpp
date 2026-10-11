#pragma once
// Test helper: a file in the current directory (the build tree, when ctest runs
// the tests), removed when the test ends.

#include <filesystem>
#include <string>
#include <system_error>

namespace ec::test {

class TempFile {
public:
    explicit TempFile(const std::string& name) : path_(std::filesystem::current_path() / ("ec_test_" + name)) {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }
    ~TempFile() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;

    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

}  // namespace ec::test
