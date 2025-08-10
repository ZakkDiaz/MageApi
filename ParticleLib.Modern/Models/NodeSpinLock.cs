//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Runtime.CompilerServices;
//using System.Text;
//using System.Threading.Tasks;

//namespace ParticleLib.Modern.Models
//{
//    internal struct NodeSpinLock
//    {
//        private int _state;                         // 0 = free, 1 = held

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public void Enter()
//        {
//            // spin with exponential back-off
//            var spin = new SpinWait();
//            while (Interlocked.CompareExchange(ref _state, 1, 0) != 0)
//                spin.SpinOnce();
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public void Exit()
//        {
//            Volatile.Write(ref _state, 0);
//        }
//    }

//}
