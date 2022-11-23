namespace NativeTrees
{
    public interface IQuadtreeRayIntersecter<T>
    {
        /// <summary>
        /// Return wether the object in question intersects with the input ray.
        /// </summary>
        /// <param name="ray">Query ray</param>
        /// <param name="obj">The object in question</param>
        /// <param name="objBounds">Bounds of the object in question</param>
        /// <param name="distance">Return the distance along from the ray origin to the intersection, if any</param>
        /// <returns>Return if a hit occured</returns>
        /// <remarks>Note that the <see cref="NativeOctree{T}"/> does not prune based on AABB's. So if your
        /// intersection test is fairly expensive, it may be a good idea to first test against the object's bounds in this method.</remarks>
        bool IntersectRay(in PrecomputedRay2D ray, T obj, AABB2D objBounds, out float distance);
    }
}