using System.Runtime.CompilerServices;
using Unity.Collections;

// https://bartvandesande.nl
// https://github.com/bartofzo

namespace NativeTrees
{
    public partial struct NativeQuadtree<T> : INativeDisposable where T : unmanaged 
    {
        /// <summary>
        /// Visits all objects that are contained in the quadtree leafs that overlap with a range.
        /// Does not check if the object's bounds overlap, that should be implemented on the visitor delegate.
        /// </summary>
        /// <param name="range"></param>
        /// <param name="visitor"></param>
        /// <typeparam name="U"></typeparam>
        /// <remarks>It's possible for objects to be visited multiple times if their bounds span multiple leafs</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Range<U>(in AABB2D range, ref U visitor) where U : struct, IQuadtreeRangeVisitor<T>
        {
            RangeNext(
                range: range,
                nodeId: 1,
                quarterSizeBounds: new QuarterSizeBounds(boundsCenter, boundsQuarterSize),
                visitor: ref visitor,
                parentDepth: 0);
        }
        
        bool RangeNext<U>(in AABB2D range, uint nodeId, in QuarterSizeBounds quarterSizeBounds, ref U visitor, int parentDepth) 
            where U : struct, IQuadtreeRangeVisitor<T>
        {
            parentDepth++;
            int rangeMask = GetBoundsMask(quarterSizeBounds.nodeCenter, range);
            
            for (int i = 0; i < 4; i++)
            {
                int quadMask = QuadMasks[i];
                if ((rangeMask & quadMask) == quadMask)
                {
                    uint octantId = GetQuadId(nodeId, i);
                    if (nodes.TryGetValue(octantId, out int objectCount) &&
                        !Range(
                            range: range,
                            nodeId: octantId,
                            quarterSizeBounds: QuarterSizeBounds.GetQuad(quarterSizeBounds, i),
                            objectCount: objectCount,
                            visitor: ref visitor,
                            depth: parentDepth))
                    {
                        return false;
                    }
                }
            }

            return true;
        }
        
        bool Range<U>(in AABB2D range, uint nodeId, in QuarterSizeBounds quarterSizeBounds, int objectCount, ref U visitor, int depth)
            where U : struct, IQuadtreeRangeVisitor<T>
        {
            // Are we in a leaf node?
            if (objectCount <= objectsPerNode || depth == maxDepth)
            {
                if (objects.TryGetFirstValue(nodeId, out var wrappedObj, out var it))
                {
                    do
                    {
                        if (!visitor.OnVisit(wrappedObj.obj, wrappedObj.bounds, range))
                            return false; // stop traversing if visitor says so
                    } while (objects.TryGetNextValue(out wrappedObj, ref it));
                }
                
                return true;
            }
            
            return RangeNext(
                range: range,
                nodeId: nodeId,
                quarterSizeBounds: quarterSizeBounds,
                visitor: ref visitor,
                parentDepth: depth);
        }
    }
}