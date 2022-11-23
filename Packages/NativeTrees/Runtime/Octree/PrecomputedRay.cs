using Unity.Mathematics;
using UnityEngine;

namespace NativeTrees
{
    /// <summary>
    /// Describes a ray in 3D space with a precomputed inverse direction, which benefits performance for raycast queries.
    /// </summary>
    public readonly struct PrecomputedRay
    {
        /// <summary>
        /// Origin position of the ray
        /// </summary>
        public readonly float3 origin;
        
        /// <summary>
        /// Direction of the ray
        /// </summary>
        public readonly float3 dir;
        
        /// <summary>
        /// One over the direction of the ray
        /// </summary>
        public readonly float3 invDir;

        public PrecomputedRay(Ray ray)
        {
            this.origin = ray.origin;
            this.dir = ray.direction;
            this.invDir = 1 / dir;
        }

        /// <summary>
        /// Create the pre-computed ray using the source for the direction, but replace it's origin with another position.
        /// </summary>
        public PrecomputedRay(PrecomputedRay source, float3 newOrigin)
        {
            this.dir = source.dir;
            this.invDir = source.invDir;
            this.origin = newOrigin;
        }

        public static explicit operator PrecomputedRay(Ray ray) => new PrecomputedRay(ray);
        public static explicit operator Ray(PrecomputedRay ray) => new Ray(ray.origin, ray.dir);
    }
}