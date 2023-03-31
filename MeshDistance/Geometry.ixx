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
    };


    export std::optional<float> getDistance(aiVector3D const& origin, Triangle const& triangle, aiVector3D const& direction)
    {
        aiVector3D v0 = triangle.v0;
        aiVector3D v1 = triangle.v1;
        aiVector3D v2 = triangle.v2;
        aiVector3D N = CrossProduct(v1 - v0, v2 - v0);
        // Step 1: finding P

        // check if the ray and plane are parallel.

        float NdotRayDirection = DotProduct(N, direction);
        if (fabs(NdotRayDirection) < std::numeric_limits<float>::epsilon()) // almost 0
            return {}; // they are parallel, so they don't intersect!

        float t = (DotProduct(N, v0) - DotProduct(N, origin)) / NdotRayDirection;

        //check if the triangle is behind the ray
        if (t < 0) return {}; // the triangle is behind

        // compute the intersection point
        aiVector3D P = origin + t * direction;
        aiVector3D C;

        // Step 2: inside-outside test

        // edge 1
        aiVector3D edge0 = v1 - v0;
        aiVector3D vp0 = P - v0;

        C = CrossProduct(edge0, vp0); // vector perpendicular to triangle's plane
        if (DotProduct(N, C) < 0) return {}; // P is on the right side

        // edge 2
        aiVector3D edge1 = v2 - v1;
        aiVector3D vp1 = P - v1;

        C = CrossProduct(edge1, vp1); // vector perpendicular to triangle's plane
        if (DotProduct(N, C) < 0) return {}; // P is on the right side

        // edge 3
        aiVector3D edge2 = v0 - v2;
        aiVector3D vp2 = P - v2;

        C = CrossProduct(edge2, vp2); // vector perpendicular to triangle's plane
        if (DotProduct(N, C) < 0) return {}; // P is on the right side;

        return t;
    }
}


