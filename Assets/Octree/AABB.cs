using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;
using static Unity.Mathematics.math;

namespace NativeTrees
{
    /// <summary>
    /// 3D axis aligned bounding box with support for fast ray intersection checking.
    /// Optimized for burst compilation.
    /// </summary>
    /// <remarks>Differs from Unity's <see cref="Bounds"/> as this stores the min and max.
    /// Which is faster for overlap and ray intersection checking</remarks>
    public struct AABB
    {
        public float3 min;
        public float3 max;

        public float3 Center => .5f * (min + max);
        public float3 Size => max - min;
        
      
        /// <summary>
        /// Construct an AABB
        /// </summary>
        /// <param name="min">Bottom left</param>
        /// <param name="max">Top right</param>
        /// <remarks>Does not check wether max is greater than min for maximum performance.</remarks>
        public AABB(float3 min, float3 max)
        {
            this.min = min;
            this.max = max;
        }

        /// <summary>
        /// Returns wether this AABB overlaps with another AABB
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Overlaps(in AABB other) =>
            all(max >= other.min) && 
            all(other.max >= min);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(float3 point) => all(point >= min) && all(point <= max);

        /// <summary>
        /// Returns wether this AABB fully contains another
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(in AABB other)
        {
            return all(min <= other.min) && 
                   all(max >= other.max);
        }

        /// <summary>
        /// Returns the closest point on this AABB from a given point. If the point lies in this AABB, the point itself is returned.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3 ClosestPoint(float3 point) => clamp(point, min, max);

        /// <summary>
        /// Returns the squared distance of a point to this AABB. If the point lies in the box, zero is returned.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float DistanceSquared(float3 point) => distancesq(point, ClosestPoint(point));

        /// <summary>
        /// Returns if a ray intersects with this bounding box. If you need the test the same ray
        /// against a lot of AABB's, it's more efficient to precompute the inverse of the ray direction and call the PrecomputedRay overload.
        /// </summary>
        /// <remarks>This method does not handle the case where a component of the ray is on the edge of the box
        /// and may return a false positive in that case. See https://tavianator.com/2011/ray_box.html and https://tavianator.com/2015/ray_box_nan.html</remarks>
        /// <returns>Wether the ray intersects this bounding box</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IntersectsRay(in Ray ray) => IntersectsRay((PrecomputedRay) ray);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IntersectsRay(in PrecomputedRay ray) => IntersectsRay(ray.origin, ray.invDir, out _);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IntersectsRay(in PrecomputedRay ray, out float3 point)
        {
            if (IntersectsRay(ray.origin, ray.invDir, out float tMin))
            {
                point = ray.origin + ray.dir * tMin;
                return true;
            }

            point = default;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IntersectsRay(in PrecomputedRay ray, out float tMin) => IntersectsRay(ray.origin, ray.invDir, out tMin);

        /// <summary>
        /// Returns if a ray intersects with this bounding box.
        /// </summary>
        /// <remarks>This method does not handle the case where a component of the ray is on the edge of the box
        /// and may return a false positive in that case. See https://tavianator.com/2011/ray_box.html and https://tavianator.com/2015/ray_box_nan.html</remarks>
        /// <returns>Wether the ray intersects this bounding box</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IntersectsRay(in float3 rayPos, in float3 rayInvDir, out float tMin) 
        {
            float3 t1 = (min - rayPos) * rayInvDir;
            float3 t2 = (max - rayPos) * rayInvDir;

            float3 tMin1 = min(t1, t2);
            float3 tMax1 = max(t1, t2);

            tMin = max(0, cmax(tMin1));
            float tMax = cmin(tMax1);
            
            return tMax >= tMin;
        }
        
        /// <summary>
        /// Returns wether max is greater or equal than min
        /// </summary>
        public bool IsValid => all(max >= min);

        public static explicit operator Bounds(AABB aabb) => new Bounds(aabb.Center, aabb.Size);
        public static implicit operator AABB(Bounds bounds) => new AABB(bounds.min, bounds.max);
    }
}