namespace NativeTrees
{
    public interface IQuadtreeRangeVisitor<T>
    {
        /// <summary>
        /// Gets called for every object contained in a leaf that overlaps with the query's range
        /// </summary>
        /// <param name="obj">The object</param>
        /// <param name="objBounds">The object's bounds</param>
        /// <param name="queryRange">The range from the query</param>
        /// <returns>Return true to keep iterating. If false is returned, iteration will stop and no more objects will be visited.</returns>
        /// <remarks>If the objects are not points, they can appear multiple times. Use a hashset or some mailboxing mechanism to
        /// guarantuee each object gets processed once.</remarks>
        bool OnVisit(T obj, AABB2D objBounds, AABB2D queryRange);
    }
}