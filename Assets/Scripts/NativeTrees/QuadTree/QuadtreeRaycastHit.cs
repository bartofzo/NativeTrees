using Unity.Mathematics;

namespace NativeTrees
{
    public struct QuadtreeRaycastHit<T>
    {
        public float2 point;
        public T obj;
    }
}