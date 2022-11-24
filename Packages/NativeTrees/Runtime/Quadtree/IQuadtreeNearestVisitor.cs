namespace NativeTrees
{
    public interface IQuadtreeNearestVisitor<T>
    {
        /// <summary>
        /// <para>
        /// Gets called for every object as a result of a nearest neighbour query.
        /// This is called in order, so the first call is the nearest object to the point. The second is the second nearest and so on.
        /// Note that if the objects are not points, it is possible for an object to be visited multiple times.
        /// </para>
        /// </summary>
        /// <param name="obj">The object in question</param>
        /// <returns>Return true to keep iterating, false to stop. So for example if you only need to know the one nearest neighbour, return false
        /// immediately.</returns>
        bool OnVist(T obj);
    }
}