#include "editor/script_editor.hpp"

#include <fstream>
#include <sstream>
#include <utility>

namespace srp::editor
{

bool ScriptEditor::load(const std::filesystem::path& path, std::string& error)
{
    error.clear();
    std::ifstream stream(path);
    if (!stream.is_open())
    {
        error = "cannot open script file: " + path.string();
        return false;
    }

    std::ostringstream buffer;
    buffer << stream.rdbuf();
    text_ = buffer.str();
    path_ = path;
    dirty_ = false;
    return true;
}

bool ScriptEditor::save(const std::filesystem::path& path, std::string& error) const
{
    error.clear();
    std::ofstream stream(path);
    if (!stream.is_open())
    {
        error = "cannot write script file: " + path.string();
        return false;
    }

    stream << text_;
    stream.flush();
    if (!stream.good())
    {
        error = "failed while writing script file: " + path.string();
        return false;
    }

    return true;
}

bool ScriptEditor::saveCurrent(std::string& error)
{
    if (path_.empty())
    {
        error = "no script file path is set";
        return false;
    }
    if (!save(path_, error))
    {
        return false;
    }
    dirty_ = false;
    return true;
}

void ScriptEditor::setText(std::string text)
{
    if (text != text_)
    {
        text_ = std::move(text);
        dirty_ = true;
    }
}

const std::string& ScriptEditor::text() const
{
    return text_;
}

const std::filesystem::path& ScriptEditor::path() const
{
    return path_;
}

bool ScriptEditor::dirty() const
{
    return dirty_;
}

void ScriptEditor::clear()
{
    path_.clear();
    text_.clear();
    dirty_ = false;
}

}  // namespace srp::editor
