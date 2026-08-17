#pragma once

#include <filesystem>
#include <string>

namespace srp::editor
{

// File-backed script text editor state. Keeps the current file path, the text
// buffer, and a dirty flag; saving clears the dirty flag.
class ScriptEditor
{
public:
    // Reads a script file into the buffer. Returns false and sets error when
    // the file cannot be read.
    bool load(const std::filesystem::path& path, std::string& error);

    // Writes the buffer to the given path. Returns false and sets error on
    // failure. Success clears the dirty flag and updates the current path.
    bool save(const std::filesystem::path& path, std::string& error) const;

    // Saves to the current path and clears the dirty flag. Returns false when
    // no path is set yet or the write fails.
    bool saveCurrent(std::string& error);

    // Replaces the buffer and marks it dirty when the content changed.
    void setText(std::string text);

    const std::string& text() const;
    const std::filesystem::path& path() const;

    bool dirty() const;

    void clear();

private:
    std::filesystem::path path_;
    std::string text_;
    bool dirty_{false};
};

}  // namespace srp::editor
