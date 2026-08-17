#include "editor/scene_editor.hpp"

#include <gtest/gtest.h>

namespace srp::editor
{
namespace
{

TEST(SceneEditorTest, AddBodyCreatesEntryAndWorldBody)
{
    SceneEditor editor;

    const srp::physics::BodyId id = editor.addBody(
        ShapeKind::kBox,
        srp::math::Vec3(1.0, 0.5, 2.0));

    ASSERT_NE(id, srp::physics::kInvalidBodyId);
    const BodyEntry* entry = editor.entry(id);
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->kind, ShapeKind::kBox);
    EXPECT_EQ(entry->name, "box_1");

    const srp::physics::RigidBodyState* body = editor.world().body(id);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->position, srp::math::Vec3(1.0, 0.5, 2.0));
    EXPECT_EQ(body->type, srp::physics::RigidBodyType::kDynamic);

    const srp::physics::CollisionShape* shape = editor.world().shape(id);
    ASSERT_NE(shape, nullptr);
    EXPECT_TRUE(std::holds_alternative<srp::physics::BoxShape>(*shape));
}

TEST(SceneEditorTest, AutoNamesAreUnique)
{
    SceneEditor editor;

    const srp::physics::BodyId box = editor.addBody(ShapeKind::kBox);
    const srp::physics::BodyId sphere = editor.addBody(ShapeKind::kSphere);
    const srp::physics::BodyId cylinder = editor.addBody(ShapeKind::kCylinder);

    EXPECT_EQ(editor.entry(box)->name, "box_1");
    EXPECT_EQ(editor.entry(sphere)->name, "sphere_2");
    EXPECT_EQ(editor.entry(cylinder)->name, "cylinder_3");
}

TEST(SceneEditorTest, RemoveBodyCleansWorldAndKeepsOtherIds)
{
    SceneEditor editor;

    const srp::physics::BodyId first = editor.addBody(ShapeKind::kBox);
    const srp::physics::BodyId second = editor.addBody(ShapeKind::kSphere);
    const srp::physics::BodyId third = editor.addBody(ShapeKind::kCylinder);

    EXPECT_TRUE(editor.removeBody(second));
    EXPECT_EQ(editor.entry(second), nullptr);
    EXPECT_EQ(editor.world().body(second), nullptr);
    EXPECT_NE(editor.world().body(first), nullptr);
    EXPECT_NE(editor.world().body(third), nullptr);
    EXPECT_EQ(editor.entries().size(), 2u);

    // Removing again fails.
    EXPECT_FALSE(editor.removeBody(second));
    // Ground cannot be removed.
    EXPECT_FALSE(editor.removeBody(editor.groundBody()));
}

TEST(SceneEditorTest, SelectionAndMove)
{
    SceneEditor editor;

    const srp::physics::BodyId id = editor.addBody(ShapeKind::kBox);
    EXPECT_FALSE(editor.selected().has_value());

    EXPECT_TRUE(editor.select(id));
    EXPECT_EQ(editor.selected(), id);

    const srp::math::Vec3 target(3.0, 1.2, -4.0);
    EXPECT_TRUE(editor.moveTo(id, target));
    EXPECT_EQ(editor.world().body(id)->position, target);
    EXPECT_EQ(editor.world().body(id)->linear_velocity, srp::math::Vec3(0.0));

    editor.deselect();
    EXPECT_FALSE(editor.selected().has_value());

    // Selecting the ground is rejected.
    EXPECT_FALSE(editor.select(editor.groundBody()));
}

TEST(SceneEditorTest, ClearObjectsKeepsGround)
{
    SceneEditor editor;
    editor.addBody(ShapeKind::kBox);
    editor.addBody(ShapeKind::kSphere);
    editor.addBody(ShapeKind::kCylinder);

    editor.clearObjects();

    EXPECT_TRUE(editor.entries().empty());
    EXPECT_EQ(editor.world().bodyIds().size(), 1u);
    EXPECT_EQ(editor.world().bodyIds().front(), editor.groundBody());
}

TEST(SceneEditorTest, PickHitsNearestBody)
{
    SceneEditor editor;

    const srp::physics::BodyId top_box = editor.addBody(
        ShapeKind::kBox,
        srp::math::Vec3(0.0, 5.0, 0.0));
    const srp::physics::BodyId bottom_box = editor.addBody(
        ShapeKind::kBox,
        srp::math::Vec3(0.0, 2.0, 0.0));

    const srp::math::Vec3 origin(0.0, 8.0, 0.0);
    const srp::math::Vec3 direction(0.0, -1.0, 0.0);

    const auto hit = SceneEditor::pick(editor.world(), origin, direction);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(*hit, top_box);

    // A ray to the side of both bodies misses.
    EXPECT_FALSE(SceneEditor::pick(
        editor.world(),
        srp::math::Vec3(4.0, 8.0, 0.0),
        srp::math::Vec3(0.0, -1.0, 0.0)).has_value());

    // The lower box becomes the nearest hit once the top box is removed.
    editor.removeBody(top_box);
    const auto hit_bottom = SceneEditor::pick(editor.world(), origin, direction);
    ASSERT_TRUE(hit_bottom.has_value());
    EXPECT_EQ(*hit_bottom, bottom_box);
}

TEST(SceneEditorTest, PickGroundIntersectsYZero)
{
    const auto hit = SceneEditor::pickGround(
        srp::math::Vec3(0.0, 3.0, 0.0),
        srp::math::Vec3(0.0, -1.0, 0.0));
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->y, 0.0, 1e-12);

    // A ray parallel to the ground never intersects it.
    EXPECT_FALSE(SceneEditor::pickGround(
        srp::math::Vec3(0.0, 3.0, 0.0),
        srp::math::Vec3(1.0, 0.0, 0.0)).has_value());
}

TEST(SceneEditorTest, PhysicsWorldStepsAfterRemoval)
{
    SceneEditor editor;
    const srp::physics::BodyId dropped = editor.addBody(
        ShapeKind::kBox,
        srp::math::Vec3(0.0, 3.0, 0.0));
    editor.addBody(ShapeKind::kSphere, srp::math::Vec3(1.0, 3.0, 1.0));

    editor.world().step(1.0 / 60.0);
    EXPECT_TRUE(editor.removeBody(dropped));
    editor.world().step(1.0 / 60.0);
    EXPECT_EQ(editor.world().bodyIds().size(), 2u);
    EXPECT_EQ(editor.entries().size(), 1u);
}

}  // namespace
}  // namespace srp::editor
