#ifndef MAX
   template <class T>
   inline static T MAX(const T &x, const T &y) {
      return (x > y ? x : y);
   }
   inline static float MAX(const float x, const int y) {
      return (x > y ? x : y);
   }
   inline static float MAX(const int x, const float y) {
      return (x > y ? x : y);
   }
#endif

#ifndef MIN
   template <class T>
   inline static T MIN(const T &x, const T &y) {
      return (x > y ? y : x);
   }
   inline static float MIN(const float x, const int y) {
      return (x > y ? y : x);
   }
   inline static float MIN(const int x, const float y) {
      return (x > y ? y : x);
   }
#endif

template <class T>
inline static T ABS(const T &x) {
   return (x > 0 ? x : -x);
}

template <class T>
inline static T SQUARE(const T &x) {
   return x * x;
}

template <class T>
inline static T DISTANCE_SQR(const T &xA,
                             const T &yA,
                             const T &xB,
                             const T &yB) {
   return SQUARE(xA - xB) + SQUARE(yA - yB);
}

// This is slow. The sqrt probably sucks.
inline static float DISTANCE(const float xA,
                             const float yA,
                             const float xB,
                             const float yB) {
   return sqrt(DISTANCE_SQR(xA, yA, xB, yB));
}

template <class T>
inline static int SIGN(const T &x) {
   return (x < 0 ? -1 : (x > 0) ? 1 : 0);
}