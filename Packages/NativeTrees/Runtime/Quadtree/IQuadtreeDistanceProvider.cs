using Unity.Mathematics;

namespace NativeTrees
{
    public interface IQuadtreeDistanceProvider<T>
    {
        /// <summary>
        /// Return the (squared) distance to an object or it's AABB.
        /// </summary>
        /// <param name="point">The point to measure the distance from</param>
        /// <param name="obj">The object to measure the distance to</param>
        /// <param name="bounds">The bounds of the object</param>
        float DistanceSquared(float2 point, T obj, AABB2D bounds);
    }
}