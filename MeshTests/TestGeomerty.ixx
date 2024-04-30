
#pragma once
#include "assimp/scene.h"
#include <optional>
#include <vector>
#include <thread>
#include <algorithm>
#include <execution>
#include <ranges>
#include <array>
#include <cassert>
#include "assimp/vector3.h"
#include "assimp/Importer.hpp"
export module TestGeometry;

export namespace geometry
{
    export void foo()
    {

    }

    export float dotProduct(aiVector3D first, aiVector3D second)
    {
        return first.x * second.x + first.y * second.y + first.z * second.z;
    }

    export aiVector3D crossProduct(aiVector3D first, aiVector3D second)
    {
        return aiVector3D(
            first.y * second.z - second.y * first.z,
            first.z * second.x - second.z * first.x,
            first.x * second.y - second.x * first.y);
    }


    export struct Triangle
    {
        aiVector3D v0, v1, v2;
        aiVector3D normal;

        bool isInsideBox(aiAABB const& aabb) const
        {
            return (v0.x >= aabb.mMin.x && v0.x <= aabb.mMax.x &&
                v0.y >= aabb.mMin.y && v0.y <= aabb.mMax.y &&
                v0.z >= aabb.mMin.z && v0.z <= aabb.mMax.z &&
                v1.x >= aabb.mMin.x && v1.x <= aabb.mMax.x &&
                v1.y >= aabb.mMin.y && v1.y <= aabb.mMax.y &&
                v1.z >= aabb.mMin.z && v1.z <= aabb.mMax.z &&
                v2.x >= aabb.mMin.x && v2.x <= aabb.mMax.x &&
                v2.y >= aabb.mMin.y && v2.y <= aabb.mMax.y &&
                v2.z >= aabb.mMin.z && v2.z <= aabb.mMax.z);
        }

        bool isPointInTriangle(aiVector3D const& p) const
        {
            auto w1 = (v0.x * (v2.y - v0.y) + (p.y - v0.y) * (v2.x - v0.x) - p.x * (v2.y - v0.y)) / ((v1.y - v0.y) * (v2.x - v0.x) - (v1.x - v0.x) * (v2.y - v0.y));
            auto w2 = (p.y - v0.y - w1 * (v1.y - v0.y)) / (v2.y - v0.y);
            return (w1 >= 0) && (w2 >= 0) && (w1 + w2 <= 1);
        }

        bool isInsideTriangle(Triangle const& other) const
        {
            return other.isPointInTriangle(v0) && other.isPointInTriangle(v1) && other.isPointInTriangle(v2);
        }

        bool doesIntersectBox(aiAABB const& box) const
        {
            // Check if any of the AABB edges intersect the triangle
            aiVector3D boxMin = box.mMin;
            aiVector3D boxMax = box.mMax;

            aiVector3D boxVertices[8] = {
                aiVector3D(boxMin.x, boxMin.y, boxMin.z),
                aiVector3D(boxMin.x, boxMin.y, boxMax.z),
                aiVector3D(boxMin.x, boxMax.y, boxMin.z),
                aiVector3D(boxMin.x, boxMax.y, boxMax.z),
                aiVector3D(boxMax.x, boxMin.y, boxMin.z),
                aiVector3D(boxMax.x, boxMin.y, boxMax.z),
                aiVector3D(boxMax.x, boxMax.y, boxMin.z),
                aiVector3D(boxMax.x, boxMax.y, boxMax.z)
            };

            aiVector3D triangleEdges[3] = {
                v1 - v0,
                v2 - v1,
                v0 - v2
            };

            aiVector3D triangleNormal = crossProduct(triangleEdges[0], triangleEdges[1]);

            for (int i = 0; i < 3; i++) {
                aiVector3D axis = crossProduct(triangleEdges[i], triangleNormal);

                float triangleMin = v0 * axis;
                float triangleMax = triangleMin;

                triangleMin = std::min(triangleMin, v1 * axis);
                triangleMax = std::max(triangleMax, v1 * axis);

                triangleMin = std::min(triangleMin, v2 * axis);
                triangleMax = std::max(triangleMax, v2 * axis);

                float boxMin = boxVertices[0] * axis;
                float boxMax = boxMin;

                for (int j = 1; j < 8; j++) {
                    float projection = boxVertices[j] * axis;

                    if (projection < boxMin) {
                        boxMin = projection;
                    }

                    if (projection > boxMax) {
                        boxMax = projection;
                    }
                }

                if (triangleMax < boxMin || triangleMin > boxMax) {
                    return false;
                }
            }
            return true;
        }
    };

    export bool isPointInsideBox(aiVector3D const& point, aiAABB const& aabb)
    {
        return point.x >= aabb.mMin.x && point.x <= aabb.mMax.x &&
            point.y >= aabb.mMin.y && point.y <= aabb.mMax.y &&
            point.z >= aabb.mMin.z && point.z <= aabb.mMax.z;
    }


    export float getDistance(aiVector3D const& origin, Triangle const& triangle, aiVector3D const& direction)
    {
        aiVector3D v0 = triangle.v0;
        aiVector3D v1 = triangle.v1;
        aiVector3D v2 = triangle.v2;
        aiVector3D N = crossProduct(v1 - v0, v2 - v0);
        // Step 1: finding intersection point P

        // check if the ray and plane are parallel.

        float NdotRayDirection = dotProduct(N, direction);
        if (fabs(NdotRayDirection) < std::numeric_limits<float>::epsilon()) // almost 0
            return std::numeric_limits<float>::max(); // they are parallel, so they don't intersect!

        float t = (dotProduct(N, v0) - dotProduct(N, origin)) / NdotRayDirection;

        //check if the triangle is behind the ray
        if (t < 0) return std::numeric_limits<float>::max(); // the triangle is behind

        // compute the intersection point
        aiVector3D P = origin + t * direction;
        aiVector3D C;

        // Step 2: inside-outside test

        // edge 1
        aiVector3D edge0 = v1 - v0;
        aiVector3D vp0 = P - v0;

        C = crossProduct(edge0, vp0); // vector perpendicular to triangle's plane
        if (dotProduct(N, C) < 0) return std::numeric_limits<float>::max(); // P is on the right side

        // edge 2
        aiVector3D edge1 = v2 - v1;
        aiVector3D vp1 = P - v1;

        C = crossProduct(edge1, vp1); // vector perpendicular to triangle's plane
        if (dotProduct(N, C) < 0) return std::numeric_limits<float>::max(); // P is on the right side

        // edge 3
        aiVector3D edge2 = v0 - v2;
        aiVector3D vp2 = P - v2;

        C = crossProduct(edge2, vp2); // vector perpendicular to triangle's plane
        if (dotProduct(N, C) < 0) return std::numeric_limits<float>::max(); // P is on the right side;

        return t;
    }


    export float getDistance(aiVector3D const& origin, std::vector<Triangle> const& triangles, aiVector3D const& direction)
    {
        if (true)
        {
            float min = std::numeric_limits<float>::max();
            for (auto const& t : triangles)
            {
                min = std::min(min, getDistance(origin, t, direction));
            }
            return min;
        }
    }





    export bool getDistanceToAABB(aiVector3D const& orig, aiVector3D const& dir, aiAABB const& aabb)
    {
        auto min = aabb.mMin;
        auto max = aabb.mMax;

        float tmin = (min.x - orig.x) / dir.x;
        float tmax = (max.x - orig.x) / dir.x;

        if (tmin > tmax) std::swap(tmin, tmax);

        float tymin = (min.y - orig.y) / dir.y;
        float tymax = (max.y - orig.y) / dir.y;

        if (tymin > tymax) std::swap(tymin, tymax);

        if ((tmin > tymax) || (tymin > tmax))
            return false;

        if (tymin > tmin)
            tmin = tymin;

        if (tymax < tmax)
            tmax = tymax;

        float tzmin = (min.z - orig.z) / dir.z;
        float tzmax = (max.z - orig.z) / dir.z;

        if (tzmin > tzmax) std::swap(tzmin, tzmax);

        if ((tmin > tzmax) || (tzmin > tmax))
            return false;

        if (tzmin > tmin)
            tmin = tzmin;

        if (tzmax < tmax)
            tmax = tzmax;

        return true;

    }

    /* export float getBoxToBoxDistance(aiAABB const& first, aiAABB const& second, aiVector3D const& rayDir)
     {
         const float a_center_x = (first.mMin.x + first.mMax.x) / 2.0f;
         const float a_center_y = (first.mMin.y + first.mMax.y) / 2.0f;
         const float a_center_z = (first.mMin.z + first.mMax.z) / 2.0f;
         const float b_center_x = (second.mMin.x + second.mMax.x) / 2.0f;
         const float b_center_y = (second.mMin.y + second.mMax.y) / 2.0f;
         const float b_center_z = (second.mMin.z + second.mMax.z) / 2.0f;
         // Project the centers onto the given direction
         const float a_proj = a_center_x * rayDir.x + a_center_y * rayDir.y + a_center_z * rayDir.z;
         const float b_proj = b_center_x * rayDir.x + b_center_y * rayDir.y + b_center_z * rayDir.z;

         // Calculate the distance between the projected centers
         const float dist = std::abs(a_proj - b_proj);

         // Calculate the half-sizes of the AABBs along the given direction
         const float a_half_size = (first.mMax.x - first.mMin.x) * std::abs(rayDir.x) +
             (first.mMax.y - first.mMin.y) * std::abs(rayDir.y) +
             (first.mMax.z - first.mMin.z) * std::abs(rayDir.z);
         const float b_half_size = (second.mMax.x - second.mMin.x) * std::abs(rayDir.x) +
             (second.mMax.y - second.mMin.y) * std::abs(rayDir.y) +
             (second.mMax.z - second.mMin.z) * std::abs(rayDir.z);

         // Calculate the directed distance
         const float directed_dist = a_half_size + b_half_size - dist;

         return directed_dist;
         
     }*/

     aiVector3D get_local_coordinates(const aiAABB& aabb, const aiVector3D& point)
     {
         // Find the center of the AABB
         aiVector3D center = aabb.mMin + (aabb.mMax - aabb.mMin) / 2.0f;

         // Translate the coordinates of the point to the coordinate system associated with the center of the AABB
         aiVector3D translated_point = point - center;

         // Get the half-sizes of the AABB along each axis
         aiVector3D half_sizes = (aabb.mMax - aabb.mMin) * 0.5f;

         // Divide the translated point coordinates by the half-sizes along each axis to get the coordinates of the point in the local coordinate system of the AABB
         aiVector3D local_coordinates(
             translated_point.x / half_sizes.x,
             translated_point.y / half_sizes.y,
             translated_point.z / half_sizes.z
         );

         return local_coordinates;
     }

    export aiVector3D line_intersection(aiVector3D const& p1, aiVector3D const& p2, aiVector3D const& q1, aiVector3D const& q2) 
    {
        aiVector3D intersectionPoint;
        aiVector3D dir1 = p2 - p1;
        aiVector3D dir2 = q2 - q1;
        aiVector3D crossProduct = dir1 ^ dir2;
        if (crossProduct.Length() < 1e-8) {
            // Lines are parallel, no intersection point
            intersectionPoint.x = NAN;
            intersectionPoint.y = NAN;
            intersectionPoint.z = NAN;
        }
        else {
            aiVector3D vecBetween = q1 - p1;
            aiVector3D crossProduct2 = vecBetween ^ dir1;
            float t = crossProduct2.Length() / crossProduct.Length();
            intersectionPoint = p1 + t*dir1;
        }
        return intersectionPoint;
    }

    export float getTriangleToTriangleDist(Triangle const& first, Triangle const& second, aiVector3D const& rayDir)
    {

        aiVector3D v0 = second.v0;
        aiVector3D v1 = second.v1;
        aiVector3D v2 = second.v2;
        aiVector3D N = crossProduct(v1 - v0, v2 - v0);

        float NdotRayDirection = dotProduct(N, rayDir);
        if (fabs(NdotRayDirection) < std::numeric_limits<float>::epsilon())
            return std::numeric_limits<float>::max();

        float t0 = (dotProduct(N, v0) - dotProduct(N, first.v0)) / NdotRayDirection;
        float t1 = (dotProduct(N, v0) - dotProduct(N, first.v1)) / NdotRayDirection;
        float t2 = (dotProduct(N, v0) - dotProduct(N, first.v2)) / NdotRayDirection;

        aiVector3D projection0 = first.v0 + t0 * rayDir;
        aiVector3D projection1 = first.v1 + t1 * rayDir;
        aiVector3D projection2 = first.v2 + t2 * rayDir;

        Triangle projectTriangle{ projection0, projection1, projection2 };

        std::vector<aiVector3D> intersection;

        if (projectTriangle.isInsideTriangle(second))
        {
            intersection.push_back(projectTriangle.v0);
            intersection.push_back(projectTriangle.v1);
            intersection.push_back(projectTriangle.v2);
        }
        else if (second.isInsideTriangle(projectTriangle))
        {
            intersection.push_back(second.v0);
            intersection.push_back(second.v1);
            intersection.push_back(second.v2);
        }
        else
        {
            auto edge00 = v1 - v0;
            auto edge01 = v2 - v0;
            auto edge02 = v2 - v1;

            auto edge10 = projection1 - projection0;
            auto edge11 = projection2 - projection0;
            auto edge12 = projection2 - projection1;


        }


        if (intersection.size() == 0)
        {
            return std::numeric_limits<float>::max();
        }

        std::vector<float> distances;
        distances.resize(intersection.size());
        v0 = first.v0;
        v1 = first.v1;
        v2 = first.v2;
        N = crossProduct(v1 - v0, v2 - v0);
        NdotRayDirection = dotProduct(N, -rayDir);

        if (fabs(NdotRayDirection) < std::numeric_limits<float>::epsilon())
            return std::numeric_limits<float>::max();

        for (int i = 0; i < intersection.size(); i++)
        {
            distances[i] = (dotProduct(N, v0) - dotProduct(N, intersection[i])) / NdotRayDirection;
        }

        return *std::min_element(distances.begin(), distances.end());
    }
   
    export std::array<aiAABB, 8> getOctants(aiAABB const& aabb)
    {
        aiVector3D BboxMin = aabb.mMin;
        aiVector3D BboxMax = aabb.mMax;

        aiVector3D center = BboxMin + (BboxMax - BboxMin) / 2.0f;

        std::array<aiAABB, 8> octants
        {
            aiAABB{ BboxMin, center },
            { { center.x, BboxMin.y, BboxMin.z }, { BboxMax.x, center.y, center.z } },
            { { center.x, BboxMin.y, center.z }, { BboxMax.x, center.y, BboxMax.z } },
            { { BboxMin.x, BboxMin.y, center.z }, { center.x, center.y, BboxMax.z } },
            { { BboxMin.x, center.y, BboxMin.z }, { center.x, BboxMax.y, center.z } },
            { { center.x, center.y, BboxMin.z }, { BboxMax.x, BboxMax.y, center.z } },
            { center, BboxMax },
            { { BboxMin.x, center.y, center.z }, { center.x, BboxMax.y, BboxMax.z } }
        };

        return octants;
    }

    export std::array<aiVector3D, 8> getAABBVertices(aiAABB const& aabb)
    {
        std::array<aiVector3D, 8> vertices;

        vertices[0] = aiVector3D(aabb.mMin.x, aabb.mMin.y, aabb.mMin.z);
        vertices[1] = aiVector3D(aabb.mMin.x, aabb.mMin.y, aabb.mMax.z);
        vertices[2] = aiVector3D(aabb.mMin.x, aabb.mMax.y, aabb.mMin.z);
        vertices[3] = aiVector3D(aabb.mMin.x, aabb.mMax.y, aabb.mMax.z);
        vertices[4] = aiVector3D(aabb.mMax.x, aabb.mMin.y, aabb.mMin.z);
        vertices[5] = aiVector3D(aabb.mMax.x, aabb.mMin.y, aabb.mMax.z);
        vertices[6] = aiVector3D(aabb.mMax.x, aabb.mMax.y, aabb.mMin.z);
        vertices[7] = aiVector3D(aabb.mMax.x, aabb.mMax.y, aabb.mMax.z);

        return vertices;
    }

    struct QueueBoxElement
    {
        int boxIndex;
        float distanceToPlane;
        QueueBoxElement(const int& index, float const& distance) : boxIndex(index), distanceToPlane(distance) {};
    };

    inline bool operator > (QueueBoxElement const& a, QueueBoxElement const& b) { return a.distanceToPlane > b.distanceToPlane; }

    export std::array<int, 8 > getBoxesIndices(aiAABB const& parentAABB, aiVector3D const& secondCenter, aiVector3D const& rayDirection)
    {
        std::priority_queue<QueueBoxElement, std::vector<QueueBoxElement>, std::greater<QueueBoxElement>> queue;

        auto octants = getOctants(parentAABB);

        for (int i = 0; i < 8; i++)
        {
            auto distance = std::numeric_limits<float>::max();

            for (auto const& vertex : getAABBVertices(octants[i]))
            {
                distance = std::min(distance, (dotProduct(rayDirection, vertex - secondCenter)) / dotProduct(rayDirection, rayDirection));
            }
            queue.push(QueueBoxElement(i, distance));
        }

        std::array<int, 8> indices;

        for (int i = 0; i < 8; i++)
        {
            indices[i] = queue.top().boxIndex;
            queue.pop();

        }
        return indices;
    }

    export float getBoxToBoxDistance(aiAABB const& from, aiAABB const& to, aiVector3D const& rayDirection)
    {
        auto fromPolygon = getAABBSection(from, rayDirection);
        auto toPolygon = getAABBSection(to, rayDirection);
        return getPolygonToPolygonDistance(fromPolygon, toPolygon, rayDirection);
    }

    export std::vector<aiVector3D> getAABBSection(aiAABB const& aabb, aiVector3D const& rayDirection)
    {
        std::vector<aiVector3D> points;
        auto aabbCenter = aabb.mMin + (aabb.mMax - aabb.mMin) / 2.0f;
        auto rayDirDotRayDir = dotProduct(rayDirection, rayDirection);
        auto rayDirDotCenter = dotProduct(rayDirection, aabbCenter);
        aiVector3D dir;
        aiVector3D aabbMin = aabb.mMin;
        aiVector3D aabbMax = aabb.mMax;

        // Test edges along X axis, pointing right.
        dir = { aabbMax.x - aabbMin.x, 0.f, 0.f };
        for (auto const& vertex : geometry::getYOZFace(aabbMin, aabbMax))
        {
            auto rayDirDotDir = dotProduct(rayDirection, dir);
            if (rayDirDotDir < std::numeric_limits<float>::epsilon())
                continue;
            float t = (rayDirDotCenter - dotProduct(rayDirection, vertex)) / rayDirDotDir;
            points.push_back(vertex + t * dir);
        }

        // Test edges along Y axis, pointing up.
        dir = { 0.f, aabbMax.y - aabbMin.y, 0.f };
        for (auto const& vertex : geometry::getXOZFace(aabbMin, aabbMax))
        {
            auto rayDirDotDir = dotProduct(rayDirection, dir);
            if (rayDirDotDir < std::numeric_limits<float>::epsilon())
                continue;
            float t = (rayDirDotCenter - dotProduct(rayDirection, vertex)) / rayDirDotDir;
            points.push_back(vertex + t * dir);
        }

        // Test edges along Z axis, pointing forward.
        dir = { 0.f, 0.f, aabbMax.z - aabbMin.z };
        for (auto const& vertex : geometry::getXOYFace(aabbMin, aabbMax))
        {
            auto rayDirDotDir = dotProduct(rayDirection, dir);
            if (rayDirDotDir < std::numeric_limits<float>::epsilon())
                continue;
            float t = (rayDirDotCenter - dotProduct(rayDirection, vertex)) / rayDirDotDir;
            points.push_back(vertex + t * dir);
        }


        if (points.size() != 0)
        {
            auto origin = points[0];
            std::sort(points.begin(), points.end(), [&](aiVector3D const& lhs, aiVector3D const& rhs) -> bool {
                aiVector3D v;
                v = crossProduct(lhs - origin, rhs - origin);
                return dotProduct(v, rayDirection) < 0;
                });
        }

        return points;
    }

    export std::array<aiVector3D, 4> getYOZFace(aiVector3D const& aabbMin, aiVector3D const& aabbMax)
    {
        return std::array<aiVector3D, 4>
        {
            aabbMin,
            { aabbMin.x, aabbMax.y, aabbMin.z },
            { aabbMin.x, aabbMin.y, aabbMax.z },
            { aabbMin.x, aabbMax.y, aabbMax.z }
        };
    }

    export std::array<aiVector3D, 4> getXOZFace(aiVector3D const& aabbMin, aiVector3D const& aabbMax)
    {
        return std::array<aiVector3D, 4>
        {
            aabbMin,
            { aabbMax.x, aabbMin.y, aabbMin.z },
            { aabbMin.x, aabbMin.y, aabbMax.z },
            { aabbMax.x, aabbMin.y, aabbMax.z }
        };
    }

    export std::array<aiVector3D, 4> getXOYFace(aiVector3D const& aabbMin, aiVector3D const& aabbMax)
    {
        return std::array<aiVector3D, 4>
        {
            aabbMin,
            { aabbMax.x, aabbMin.y, aabbMin.z },
            { aabbMin.x, aabbMax.y, aabbMin.z },
            { aabbMax.x, aabbMax.y, aabbMin.z }
        };
    }

    export float getPolygonToPolygonDistance(std::vector<aiVector3D> const& fromPoints, std::vector<aiVector3D> const& toPoints, aiVector3D const& rayDirection)
    {
        aiVector3D toPlaneNormal = crossProduct(rayDirection, toPoints[0]);
        auto toPlane = Plane(toPlaneNormal, toPoints[0]);

        float maxFloat = std::numeric_limits<float>::max();
        float distance = maxFloat;

        for (auto const& point : fromPoints)
        {
            float t = getPointToPlaneDistance(point, toPlane, rayDirection);
            if (t == maxFloat)
                continue;
            if (isPointInsidePolygon(point, toPoints))
                distance = std::min(distance, t);
        }

        return distance;
    }

    export struct Plane
    {
        aiVector3D normal;
        aiVector3D pointOnPlane;
        Plane(aiVector3D const& n, aiVector3D const& p) : normal(n), pointOnPlane(p) {};
    };

    export float getPointToPlaneDistance(aiVector3D const& origin, Plane const& plane, aiVector3D const& rayDir)
    {
        auto N = plane.normal;
        auto p = plane.pointOnPlane;
        auto NdotRayDirection = dotProduct(N, rayDir);
        auto epsilon = std::numeric_limits<float>::epsilon();
        auto maxFloat = std::numeric_limits<float>::max();

        return (NdotRayDirection < epsilon) ? maxFloat : (dotProduct(N, p) - dotProduct(N, origin)) / NdotRayDirection;
    }

    export bool isPointInsidePolygon(aiVector3D const& point, std::vector<aiVector3D> const& polygon)
    {
        int n = polygon.size();
        int count = 0;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            if ((polygon[i].y > point.y) != (polygon[j].y > point.y)) {
                float x = (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x;
                if (x > point.x) {
                    count++;
                }
            }
        }
        return count % 2 == 1;
    }

}


