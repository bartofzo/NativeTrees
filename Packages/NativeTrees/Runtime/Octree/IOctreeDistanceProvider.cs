using Unity.Mathematics;

namespace NativeTrees
{
    public interface IOctreeDistanceProvider<T>
    {
        /// <summary>
        /// Return the (squared) distance to an object or it's AABB.
        /// </summary>
        /// <param name="point">The point to measure the distance from</param>
        /// <param name="obj">The object to measure the distance to</param>
        /// <param name="bounds">The bounds of the object</param>
        float DistanceSquared(float3 point, T obj, AABB bounds);
    }
}