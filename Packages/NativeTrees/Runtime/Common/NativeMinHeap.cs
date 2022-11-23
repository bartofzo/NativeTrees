using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Jobs;

namespace NativeTrees
{
    /// <summary>
    /// A simple binary heap useful for nearest neighbour queries
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <typeparam name="TComp"></typeparam>
    public struct NativeMinHeap<T, TComp> : INativeDisposable
        where T : unmanaged
        where TComp : struct, IComparer<T>
    {
        private NativeList<T> values;
        private TComp comparer;

        public NativeMinHeap(TComp comparer, Allocator allocator)
        {
            this.comparer = comparer;
            values = new NativeList<T>(allocator);
            values.Add(default);
        }

        public void Clear()
        {
            values.Clear();
            values.Add(default);
        }
        
        public int Count => values.Length - 1;
        public T Min => values[1];

        public bool TryPop(out T value)
        {
            if (Count > 0)
            {
                value = Pop();
                return true;
            }

            value = default;
            return false;
        }

        public T Pop()
        {
            int count = Count;
            if (count == 0)
                throw new InvalidOperationException("Heap is empty.");
         
            var min = Min;
            values[1] = values[count];
            values.RemoveAt(count);

            if (values.Length > 1)
                BubbleDown(1);
      
            return min;
        }
        
        public void Push(T item)
        {
            values.Add(item);
            BubbleUp(Count);
        }

        private void BubbleUp(int index)
        {
            int parent = index / 2;
            while (index > 1 && CompareResult(parent, index) > 0)
            {
                Exchange(index, parent);
                index = parent;
                parent /= 2;
            }
        }

        private void BubbleDown(int index)
        {
            int min;

            while (true)
            {
                int left = index * 2;
                int right = left + 1;

                if (left < values.Length && CompareResult(left, index) < 0)
                    min = left;
                else
                    min = index;

                if (right < values.Length && CompareResult(right, min) < 0)
                    min = right;
              
                if (min != index)
                {
                    Exchange(index, min);
                    index = min;
                }
                else
                    return;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int CompareResult(int index1, int index2)
        {
            return comparer.Compare(values[index1], values[index2]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Exchange(int index, int max)
        {
            var tmp = values[index];
            values[index] = values[max];
            values[max] = tmp;
        }
        
        public void Dispose()
        {
            values.Dispose();
        }

        public JobHandle Dispose(JobHandle inputDeps)
        {
            return values.Dispose(inputDeps);
        }
    }
}