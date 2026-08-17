#include "editor/script_editor.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

namespace srp::editor
{
namespace
{

std::filesystem::path tempScriptPath(const char* name)
{
    return std::filesystem::temp_directory_path() / name;
}

TEST(ScriptEditorTest, LoadReadsFileContent)
{
    const std::filesystem::path path = tempScriptPath("srp_script_editor_load.lua");
    {
        std::ofstream stream(path);
        stream << "function update(dt)\n    set_motor(1, 1.0)\nend\n";
    }

    ScriptEditor editor;
    std::string error;
    ASSERT_TRUE(editor.load(path, error)) << error;
    EXPECT_EQ(editor.path(), path);
    EXPECT_FALSE(editor.dirty());
    EXPECT_EQ(editor.text(), "function update(dt)\n    set_motor(1, 1.0)\nend\n");
    std::filesystem::remove(path);
}

TEST(ScriptEditorTest, LoadMissingFileFails)
{
    ScriptEditor editor;
    std::string error;
    EXPECT_FALSE(editor.load(
        std::filesystem::temp_directory_path() / "srp_missing_script.lua",
        error));
    EXPECT_FALSE(error.empty());
}

TEST(ScriptEditorTest, SetTextMarksDirty)
{
    const std::filesystem::path path = tempScriptPath("srp_script_editor_save.lua");
    ScriptEditor editor;
    std::string error;

    editor.setText("elapsed = 0");
    EXPECT_TRUE(editor.dirty());

    // Editing to the same text keeps the dirty flag stable.
    editor.setText("elapsed = 0");
    EXPECT_TRUE(editor.dirty());

    ASSERT_TRUE(editor.save(path, error)) << error;
    EXPECT_TRUE(editor.dirty());  // const save does not claim the buffer saved

    std::string loaded;
    {
        std::ifstream stream(path);
        std::getline(stream, loaded);
    }
    EXPECT_EQ(loaded, "elapsed = 0");
    std::filesystem::remove(path);
}

TEST(ScriptEditorTest, SaveCurrentUpdatesPathAndDirty)
{
    const std::filesystem::path path = tempScriptPath("srp_script_editor_current.lua");
    {
        std::ofstream stream(path);
        stream << "print('initial')\n";
    }

    ScriptEditor editor;
    std::string error;

    EXPECT_FALSE(editor.saveCurrent(error));

    ASSERT_TRUE(editor.load(path, error)) << error;
    editor.setText("print('hello')");
    EXPECT_TRUE(editor.dirty());

    ASSERT_TRUE(editor.saveCurrent(error)) << error;
    EXPECT_FALSE(editor.dirty());
    EXPECT_EQ(editor.path(), path);
    std::filesystem::remove(path);
}

TEST(ScriptEditorTest, ClearResetsState)
{
    ScriptEditor editor;
    editor.setText("x = 1");
    editor.clear();

    EXPECT_TRUE(editor.text().empty());
    EXPECT_TRUE(editor.path().empty());
    EXPECT_FALSE(editor.dirty());
}

}  // namespace
}  // namespace srp::editor
