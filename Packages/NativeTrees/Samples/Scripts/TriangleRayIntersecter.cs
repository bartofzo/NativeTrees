using System;
using NativeTrees;
using Unity.Mathematics;
using UnityEngine;

namespace NativeTrees.Samples
{
    public struct TriangleRayIntersecter : IOctreeRayIntersecter<Triangle>
    {
        public bool IntersectRay(in PrecomputedRay ray, Triangle triangle, AABB objBounds, out float distance)
        {
            if (!objBounds.IntersectsRay(ray))
            {
                distance = float.PositiveInfinity;
                return false;
            }

            return triangle.IntersectsRay(ray.origin, ray.dir, out distance);
        }
    }
}