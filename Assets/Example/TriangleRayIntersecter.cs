using System;
using AnyPath.Graphs.Extra;
using NativeTrees;
using Unity.Mathematics;
using UnityEngine;

namespace Example
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