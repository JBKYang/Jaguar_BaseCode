﻿using System;

namespace DrRobot.JaguarControl
{
    public class Tuple<T1, T2>
    {
        public T1 First { get; private set; }
        public T2 Second { get; private set; }
        internal Tuple(T1 first, T2 second)
        {
            First = first;
            Second = second;
        }
    }

    public static class Tuple
    {
        public static Tuple<T1, T2> New<T1, T2>(T1 first, T2 second)
        {
            var tuple = new Tuple<T1, T2>(first, second);
            return tuple;
        }
            // Copy Constructor
        public static Tuple<T1, T2 > newTuple<T1,T2> (Tuple <T1, T2> origTuple) 
        {
            T1 tempFirst = origTuple.First;
            T2 tempSecond = origTuple.Second;
            var tuple = new Tuple<T1, T2>(tempFirst, tempSecond);
            return tuple;
        }
          
    }

}