using UnityEngine;

namespace Csharp {
  public static class M{
    public static Matrix4x4 Rot (Quaternion q, Matrix4x4 a) {
      Matrix4x4 T = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);
      Matrix4x4 c = T*a*T.inverse;
      return c;
    }
    public static Matrix4x4 Mul (Matrix4x4 a, float b) {
      Matrix4x4 c = Matrix4x4.identity;
      for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
          if ((i != j) && (i == 3 || j == 3)) {
            c[i,j] = 0;
          }
          else if (i == j && i == 3) {
            c[3,3] = 1.0f;
          }
          else {
            c[i,j] = a[i,j]*b;
          }
        }
      }
      return c;
    }
    public static Matrix4x4 Add (Matrix4x4 a, Matrix4x4 b) {
      Matrix4x4 c = Matrix4x4.identity;
      for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
          if ((i != j) && (i == 3 || j == 3)) {
            c[i,j] = 0;
          }
          else if (i == j && i == 3) {
            c[3,3] = 1.0f;
          }
          else {
            c[i,j] = a[i,j] + b[i,j];
          }
        }
      }
      return c;
    }
    public static Matrix4x4 Outer (Vector3 a, Vector3 b) {
      Matrix4x4 c = Matrix4x4.identity;
      c[0,0] = a.x*b.x;
      c[0,1] = a.x*b.y;
      c[0,2] = a.x*b.z;
      c[1,0] = a.y*b.x;
      c[1,1] = a.y*b.y;
      c[1,2] = a.y*b.z;
      c[2,0] = a.z*b.x;
      c[2,1] = a.z*b.y;
      c[2,2] = a.z*b.z;
      return c;
    }
  }
}
