#include "assimp/scene.h"
#include <optional>

export module Geometry;
namespace geometry
{
    export float DotProduct(aiVector3D first, aiVector3D second)
    {
        return first.x * second.x + first.y * second.y + first.z * second.z;
    }

    export aiVector3D CrossProduct(aiVector3D first, aiVector3D second)
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

        bool IsInsideBox(aiAABB const& aabb) const
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
    };




    export float getDistance(aiVector3D const& origin, Triangle const& triangle, aiVector3D const& direction)
    {
        aiVector3D v0 = triangle.v0;
        aiVector3D v1 = triangle.v1;
        aiVector3D v2 = triangle.v2;
        aiVector3D N = CrossProduct(v1 - v0, v2 - v0);
        // Step 1: finding intersection point P

        // check if the ray and plane are parallel.

        float NdotRayDirection = DotProduct(N, direction);
        if (fabs(NdotRayDirection) < std::numeric_limits<float>::epsilon()) // almost 0
            return std::numeric_limits<float>::max(); // they are parallel, so they don't intersect!

        float t = (DotProduct(N, v0) - DotProduct(N, origin)) / NdotRayDirection;

        //check if the triangle is behind the ray
        if (t < 0) return std::numeric_limits<float>::max(); // the triangle is behind

        // compute the intersection point
        aiVector3D P = origin + t * direction;
        aiVector3D C;

        // Step 2: inside-outside test

        // edge 1
        aiVector3D edge0 = v1 - v0;
        aiVector3D vp0 = P - v0;

        C = CrossProduct(edge0, vp0); // vector perpendicular to triangle's plane
        if (DotProduct(N, C) < 0) return std::numeric_limits<float>::max(); // P is on the right side

        // edge 2
        aiVector3D edge1 = v2 - v1;
        aiVector3D vp1 = P - v1;

        C = CrossProduct(edge1, vp1); // vector perpendicular to triangle's plane
        if (DotProduct(N, C) < 0) return std::numeric_limits<float>::max(); // P is on the right side

        // edge 3
        aiVector3D edge2 = v0 - v2;
        aiVector3D vp2 = P - v2;

        C = CrossProduct(edge2, vp2); // vector perpendicular to triangle's plane
        if (DotProduct(N, C) < 0) return std::numeric_limits<float>::max(); // P is on the right side;

        return t;
    }

    export std::pair<bool, float> rayIntersectBox(aiVector3D const& origin, aiVector3D const& direction, aiAABB const& aabb)
    {
        float tMin = 0.0f;
        float tMax = std::numeric_limits<float>::max();

        for (int i = 0; i < 3; i++)
        {
            if (std::abs(direction[i]) < std::numeric_limits<float>::epsilon())
            {
                // Ray is parallel to slab, no intersection if origin is outside slab
                if (origin[i] < aabb.mMin[i] || origin[i] > aabb.mMax[i])
                {
                    return std::make_pair(false, std::numeric_limits<float>::max());
                }
            }
            else
            {
                // Compute intersection t value of ray with near and far plane of slab
                float tNear = (aabb.mMin[i] - origin[i]) / direction[i];
                float tFar = (aabb.mMax[i] - origin[i]) / direction[i];

                // Swap if tNear is greater than tFar
                if (tNear > tFar)
                {
                    std::swap(tNear, tFar);
                }

                // Update tMin and tMax
                tMin = std::max(tMin, tNear);
                tMax = std::min(tMax, tFar);

                // Exit with no collision if tMin is greater than tMax
                if (tMin > tMax)
                {
                    return std::make_pair(false, std::numeric_limits<float>::max());
                }
            }
        }

        // Ray intersects AABB
        return std::make_pair(true, tMin);
    
    }
}


