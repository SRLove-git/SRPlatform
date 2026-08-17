#include "rendering/debug_draw.hpp"

#include <GL/gl.h>
#include <GL/glu.h>

#include <cmath>
#include <algorithm>
#include <type_traits>

#include <glm/gtc/quaternion.hpp>

namespace srp::rendering
{

namespace
{

void applyTransform(const srp::math::Vec3& position, const srp::math::Quat& orientation)
{
    glTranslated(position.x, position.y, position.z);

    const double w = std::clamp(orientation.w, -1.0, 1.0);
    const double angle = 2.0 * std::acos(w);
    const double half_angle_sine = std::sin(angle * 0.5);
    if (angle > 1e-9 && std::abs(half_angle_sine) > 1e-9)
    {
        const srp::math::Vec3 axis(
            orientation.x / half_angle_sine,
            orientation.y / half_angle_sine,
            orientation.z / half_angle_sine);

        glRotated(
            srp::math::degrees(angle),
            axis.x,
            axis.y,
            axis.z);
    }
}

void drawBoxEdges(const srp::math::Vec3& half_extents)
{
    const double x = half_extents.x;
    const double y = half_extents.y;
    const double z = half_extents.z;

    const srp::math::Vec3 corners[8] = {
        {-x, -y, -z},
        {x, -y, -z},
        {x, y, -z},
        {-x, y, -z},
        {-x, -y, z},
        {x, -y, z},
        {x, y, z},
        {-x, y, z}};

    const int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}};

    glBegin(GL_LINES);
    for (const auto& edge : edges)
    {
        glVertex3d(corners[edge[0]].x, corners[edge[0]].y, corners[edge[0]].z);
        glVertex3d(corners[edge[1]].x, corners[edge[1]].y, corners[edge[1]].z);
    }
    glEnd();
}

void drawSphereWireframe(double radius)
{
    GLUquadric* quadric = gluNewQuadric();
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    gluSphere(quadric, radius, 16, 12);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    gluDeleteQuadric(quadric);
}

void drawPlaneGrid(double half_size)
{
    glBegin(GL_LINES);
    for (int i = -10; i <= 10; ++i)
    {
        const double position = static_cast<double>(i) * 0.5;
        glVertex3d(position, 0.0, -half_size);
        glVertex3d(position, 0.0, half_size);
        glVertex3d(-half_size, 0.0, position);
        glVertex3d(half_size, 0.0, position);
    }
    glEnd();
}

void drawConvexHullPoints(const srp::physics::ConvexHullShape& hull)
{
    glBegin(GL_POINTS);
    for (const srp::math::Vec3& point : hull.points)
    {
        glVertex3d(point.x, point.y, point.z);
    }
    glEnd();
}

}  // namespace

void drawCollisionShape(
    const srp::physics::CollisionShape& shape,
    const srp::math::Vec3& position,
    const srp::math::Quat& orientation)
{
    glPushMatrix();
    applyTransform(position, orientation);

    std::visit(
        [](const auto& value)
        {
            using T = std::decay_t<decltype(value)>;

            if constexpr (std::is_same_v<T, srp::physics::BoxShape>)
            {
                drawBoxEdges(value.half_extents);
            }
            else if constexpr (std::is_same_v<T, srp::physics::SphereShape>)
            {
                drawSphereWireframe(value.radius);
            }
            else if constexpr (std::is_same_v<T, srp::physics::PlaneShape>)
            {
                drawPlaneGrid(5.0);
            }
            else if constexpr (std::is_same_v<T, srp::physics::CylinderShape>)
            {
                GLUquadric* quadric = gluNewQuadric();
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glTranslated(0.0, -value.half_height, 0.0);
                gluCylinder(quadric, value.radius, value.radius, value.half_height * 2.0, 16, 4);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                gluDeleteQuadric(quadric);
            }
            else
            {
                drawConvexHullPoints(value);
            }
        },
        shape);

    glPopMatrix();
}

void drawContactPoint(const srp::physics::ContactPoint& contact_point)
{
    glBegin(GL_LINES);
    glColor3d(1.0, 0.2, 0.2);
    glVertex3d(
        contact_point.point.x - 0.05,
        contact_point.point.y,
        contact_point.point.z);
    glVertex3d(
        contact_point.point.x + 0.05,
        contact_point.point.y,
        contact_point.point.z);
    glVertex3d(
        contact_point.point.x,
        contact_point.point.y - 0.05,
        contact_point.point.z);
    glVertex3d(
        contact_point.point.x,
        contact_point.point.y + 0.05,
        contact_point.point.z);
    glVertex3d(
        contact_point.point.x,
        contact_point.point.y,
        contact_point.point.z - 0.05);
    glVertex3d(
        contact_point.point.x,
        contact_point.point.y,
        contact_point.point.z + 0.05);

    glVertex3d(
        contact_point.point.x,
        contact_point.point.y,
        contact_point.point.z);
    glVertex3d(
        contact_point.point.x + contact_point.normal.x * 0.2,
        contact_point.point.y + contact_point.normal.y * 0.2,
        contact_point.point.z + contact_point.normal.z * 0.2);
    glEnd();
}

}  // namespace srp::rendering
