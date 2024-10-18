using UnityEngine;
using System;

public class RMSD_fix : MonoBehaviour
{
    //[SerializeField] private float reject = 3.0f;
    private float[] refxyz;
    private Vector3 origin;

    void Start()
    {
      refxyz = GetChildPositions(0, transform, 1.0f);
      origin = transform.localPosition;
      //RMSDkit.DebugUnits();
    }

    void Update()
    {
      float[] trgxyz = GetChildPositions(refxyz.Length, transform, 1.0f);
      float ang = 0.0f;
      Vector3 axis = Vector3.zero;
      Vector3 centroid = GetChildCentroid(transform);

      Quaternion q = RMSDkit.SuperposeQuaternion(refxyz, trgxyz);
      q.ToAngleAxis(out ang, out axis);
      transform.RotateAround(centroid, axis, ang);
      transform.localPosition = origin - centroid;
    }

    private float[] GetChildPositions(int nmax, Transform tf, float weight)
    {
      int cc = transform.childCount;
      int nc = cc * 3;
      nc = nmax > 0 ? (nmax > nc ? nc : nmax) : nc;
      float[] xyz = new float[nc];
      for (int i = 0; i < cc; i++){
         xyz[3*i + 0] = tf.GetChild(i).transform.position.x;
         xyz[3*i + 1] = tf.GetChild(i).transform.position.y;
         xyz[3*i + 2] = tf.GetChild(i).transform.position.z;
      }
      RMSDkit.Centering(ref xyz, RMSDkit.Centroid(xyz));
      return xyz;
    }

    private Vector3 GetChildCentroid(Transform tf)
    {
      int cc = transform.childCount;
      float rn = 1.0f/cc;
      Vector3 cnt = Vector3.zero;
      for (int i = 0; i < cc; i++){
         cnt.x += tf.GetChild(i).transform.localPosition.x;
         cnt.y += tf.GetChild(i).transform.localPosition.y;
         cnt.z += tf.GetChild(i).transform.localPosition.z;
      }
      cnt.x *= rn;
      cnt.y *= rn;
      cnt.z *= rn;
      return cnt;
    }
}

  public class RMSDkit
  {
    const float threshold = 1.0e-8f;  // Epsilon.
    const float degeneracy = 1.0e-4f; // Degeneracy threshold.

    /******************
     Struct definition
    ******************/

    struct SymmetricMatrix3x3
    {
      public float x00 { get; }
      public float x10 { get; } public float x11 { get; }
      public float x20 { get; } public float x21 { get; } public float x22 { get; }

      public SymmetricMatrix3x3(in float X00,
                                in float X10, in float X11,
                                in float X20, in float X21, in float X22)
      {
        x00 = X00;
        x10 = X10; x11 = X11;
        x20 = X20; x21 = X21; x22 = X22;
      }

      override public string ToString() =>
          $"\n| {pp(x00,6,2)} {pp(x10,6,2)} {pp(x20,6,2)} |"
        + $"\n| {pp(x10,6,2)} {pp(x11,6,2)} {pp(x21,6,2)} |"
        + $"\n| {pp(x20,6,2)} {pp(x21,6,2)} {pp(x22,6,2)} |";
    }

    struct SymmetricMatrix4x4
    {
      public float x00 { get; }
      public float x10 { get; } public float x11 { get; }
      public float x20 { get; } public float x21 { get; } public float x22 { get; }
      public float x30 { get; } public float x31 { get; } public float x32 { get; } public float x33 { get; }

      public SymmetricMatrix4x4(in float X00,
                                in float X10, in float X11,
                                in float X20, in float X21, in float X22,
                                in float X30, in float X31, in float X32, in float X33)
      {
        x00 = X00;
        x10 = X10; x11 = X11;
        x20 = X20; x21 = X21; x22 = X22;
        x30 = X30; x31 = X31; x32 = X32; x33 = X32;
      }

      override public string ToString() =>
          $"\n| {pp(x00,6,2)} {pp(x10,6,2)} {pp(x20,6,2)} {pp(x30,6,2)} |"
        + $"\n| {pp(x10,6,2)} {pp(x11,6,2)} {pp(x21,6,2)} {pp(x31,6,2)} |"
        + $"\n| {pp(x20,6,2)} {pp(x21,6,2)} {pp(x22,6,2)} {pp(x32,6,2)} |"
        + $"\n| {pp(x30,6,2)} {pp(x31,6,2)} {pp(x32,6,2)} {pp(x33,6,2)} |";
    }

    private struct Matrix3x3
    {
      public float x00 { get; } public float x01 { get; } public float x02 { get; }
      public float x10 { get; } public float x11 { get; } public float x12 { get; }
      public float x20 { get; } public float x21 { get; } public float x22 { get; }

      public Matrix3x3(in float X00, in float X01, in float X02,
                       in float X10, in float X11, in float X12,
                       in float X20, in float X21, in float X22)
      {
        x00 = X00; x01 = X01; x02 = X02;
        x10 = X10; x11 = X11; x12 = X12;
        x20 = X20; x21 = X21; x22 = X22;
      }

      public SymmetricMatrix3x3 MxMt()
      {
        return new SymmetricMatrix3x3(
          x00*x00+x01*x01+x02*x02,
          x10*x00+x11*x01+x12*x02, x10*x10+x11*x11+x12*x12,
          x20*x00+x21*x01+x22*x02, x20*x10+x21*x11+x22*x12, x20*x20+x21*x21+x22*x22);
      }

      override public string ToString() =>
          $"\n| {pp(x00,6,2)} {pp(x01,6,2)} {pp(x02,6,2)} |"
        + $"\n| {pp(x10,6,2)} {pp(x11,6,2)} {pp(x12,6,2)} |"
        + $"\n| {pp(x20,6,2)} {pp(x21,6,2)} {pp(x22,6,2)} |";
    }

    /******************
         Interface
    ******************/

    public static Quaternion SuperposeQuaternion(in float[] rxyz, in float[] txyz)
    {
      Matrix3x3 C = Cov(rxyz, txyz);
      float lmax = FindLambdaMax(C);
      return GetSuperpose(C, lmax);
    }

    public static Vector3 Centroid(in float[] x)
    {
      if (x.Length<1) { return new Vector3(0.0f, 0.0f, 0.0f); }
      float cx=0.0f, cy=0.0f, cz=0.0f;
      for (int i = 0; i < x.Length; i+=3){
        cx += x[i+0];
        cy += x[i+1];
        cz += x[i+2];
      }
      float nrm = (3.0f / x.Length);
      return new Vector3(cx*nrm, cy*nrm, cz*nrm);
    }

    public static void Centering(ref float[] x, in Vector3 c)
    {
      for (int i = 0; i < x.Length; i+=3){
         x[i+0] -= c.x;
         x[i+1] -= c.y;
         x[i+2] -= c.z;
      }
    }

    /******************
       core routines
    ******************/

    /**
       FindLambdaMax

         Returns maximum eigen value corresponds to a covariance matrix, C.

       ---
       Let

         D = C@C^T
         p = D11 * (D22+D33) + D22*D33 - D21*D12 - D31*D13 - D32*D23
         q = |C| (where, |.| denotes determinant)
         r = trace[D]

       The eigenvalues are obtained as the roots of the quartic equation:

         X^4 - 2 r X^2 - 8 q X + r^2 - 4 p = 0.

       By rescaling X = sqrt(r) Y, the quartic is transformed to

         Y^4 - 2 Y^2 + a Y + b = 0.

       where,

         a = - 8 q r^{-3/2}
         b = 1 - 4 p r^{-2}

       respectively.

       See [10.1002/jcc.25802](https://onlinelibrary.wiley.com/doi/10.1002/jcc.25802) for detail.
    **/
    private static float FindLambdaMax(in Matrix3x3 C)
    {
      // D = C @ C^T
      SymmetricMatrix3x3 D = C.MxMt();
      // Check trace. if r=0, C is the zero matrix.
      float r = D.x00 + D.x11 + D.x22; if(r < threshold) { return 0.0f; }
      //float r = D.Trace(); if(r < threshold) { return 0.0f; }
      float p = D.x00 * (D.x11 + D.x22) + D.x11 * D.x22 - (D.x10 * D.x10 + D.x20 * D.x20 + D.x21 * D.x21);
      float q = det3(C.x00, C.x01, C.x02,
                     C.x10, C.x11, C.x12,
                     C.x20, C.x21, C.x22);
      float sr = sqrt(r), rr = 4.0f / (r * r);
      float a = - 2.0f * q * rr * sr;
      float b = 1.0f - p * rr;
      float y = find_a_quartic_root(a, b);

      //Debug.Log($" p    {pp(p,9,6)} q    {pp(q,9,6)} r    {pp(r,9,6)}\n"
      //                + $" a    {pp(a,9,6)} b    {pp(b,9,6)}\n"
      //                + $" y    {pp(y,9,6)} f(y) {pp(y*y*y*y-2.0f*y*y+a*y+b,9,6)} x    {pp(sr*y,9,6)}");

      return sr * y;
    }

    /**
       The explicit Quaternion of covariance matrix is

           ( c00+c11+c22  c12-c21      c20-c02      c01-c10     )
       F = ( c12-c21      c00-c11-c22  c01+c10      c02+c20     )
           ( c20-c02      c01+c10     -c00+c11-c22  c12+c21     )
           ( c01-c10      c02+c20      c12+c21     -c00-c11+c22 )

       We want to find a eigen vector q, satisfies F q = lambda * q.
       We are looking for a null vector that is A q = 0, where A = L - lambda I.
    **/
    private static Quaternion GetSuperpose(in Matrix3x3 C, in float lambda)
    {
      float p01 = C.x00 + C.x11,  m01 = C.x00 - C.x11;
      float p2d = C.x22 - lambda, m2d =-C.x22 - lambda;
      float a00 = p01+p2d;
      float a11 = m01+m2d;
      float a22 =-m01+m2d;
      float a33 =-p01+p2d;
      float a10,
            a20, a21,
            a30, a31, a32;

      float abs00 = abs(a00), abs22 = abs(a22);
      float q0, q1, q2, q3;

//    float a10 = C.x12-C.x21, a11 = m01+m2d;
//    float a20 = C.x20-C.x02, a21 = C.x01+C.x10, a22 =-m01+m2d;
//    float a30 = C.x01-C.x10, a31 = C.x02+C.x20, a32 = C.x12+C.x21, a33 =-p01+p2d;

      if (abs00 > abs22)
      {
        float abs11 = abs(a11);
        if(abs00 > abs11)
        {
         a00 = 1.0f/a00;
         a10 = a00*(C.x12-C.x21); a11 *= a00;
         a20 = a00*(C.x20-C.x02); a21 = a00*(C.x01+C.x10); a22 *= a00;
         a30 = a00*(C.x01-C.x10); a31 = a00*(C.x02+C.x20); a32 = a00*(C.x12+C.x21); a33 *= a00;
         (q0, q1, q2, q3) = find_nullvector(1.0f,
                                            a10, a11,
                                            a20, a21, a22,
                                            a30, a31, a32, a33);
       //(q0, q1, q2, q3) = find_nullvector(a00,
       //                                   a10, a11,
       //                                   a20, a21, a22,
       //                                   a30, a31, a32, a33);
        }
        else
        {
          a11 = 1.0f/a11;
          a10 = a11*(C.x12-C.x21); a00 *= a11;
          a20 = a11*(C.x20-C.x02); a21 = a11*(C.x01+C.x10); a22*= a11;
          a30 = a11*(C.x01-C.x10); a31 = a11*(C.x02+C.x20); a32 = a11*(C.x12+C.x21); a33*= a11;
          (q1, q0, q2, q3) = find_nullvector(1.0f,
                                             a10, a00,
                                             a21, a20, a22,
                                             a31, a30, a32, a33);
        }
      }
      else
      {
        float abs33 = abs(a33);
        if(abs22 > abs33)
        {
          a22 = 1.0f/a22;
          a10 = a22*(C.x12-C.x21); a11*= a22;
          a20 = a22*(C.x20-C.x02); a21 = a22*(C.x01+C.x10); a00*= a22;
          a30 = a22*(C.x01-C.x10); a31 = a22*(C.x02+C.x20); a32 = a22*(C.x12+C.x21); a33*= a22;
          (q2, q1, q0, q3) = find_nullvector(1.0f,
                                             a21, a11,
                                             a20, a10, a00,
                                             a32, a31, a30, a33);
        }
        else
        {
          a33 = 1.0f/a33;
          a10 = a33*(C.x12-C.x21); a11*= a33;
          a20 = a33*(C.x20-C.x02); a21 = a33*(C.x01+C.x10); a22*= a33;
          a30 = a33*(C.x01-C.x10); a31 = a33*(C.x02+C.x20); a32 = a33*(C.x12+C.x21); a00*= a33;
          (q3, q1, q2, q0) = find_nullvector(1.0f,
                                             a31, a11,
                                             a32, a21, a22,
                                             a30, a10, a20, a00);
        }
      }

      /**
      float d00 = a00+lambda, d11 = a11+lambda, d22 = a22+lambda, d33 = a33+lambda;
      Debug.Log("L "
             + $"\n{pp(d00)} {pp(a10)} {pp(a20)} {pp(a30)}"
             + $"\n{pp(a10)} {pp(d11)} {pp(a21)} {pp(a31)}"
             + $"\n{pp(a20)} {pp(a21)} {pp(d22)} {pp(a32)}"
             + $"\n{pp(a30)} {pp(a31)} {pp(a32)} {pp(d33)}"
             + "\nA "
             + $"\n{pp(a00)} {pp(a10)} {pp(a20)} {pp(a30)}"
             + $"\n{pp(a10)} {pp(a11)} {pp(a21)} {pp(a31)}"
             + $"\n{pp(a20)} {pp(a21)} {pp(a22)} {pp(a32)}"
             + $"\n{pp(a30)} {pp(a31)} {pp(a32)} {pp(a33)}"
             + "\nLambda     " + lambda
             + "\nNull vector"
             + $" {pp(q0,12,6)}"
             + $" {pp(q1,12,6)}"
             + $" {pp(q2,12,6)}"
             + $" {pp(q3,12,6)}"
             + "\nLq/Lambda  "
             + $" {pp((d00*q0+a10*q1+a20*q2+a30*q3)/lambda,12,6)}"
             + $" {pp((a10*q0+d11*q1+a21*q2+a31*q3)/lambda,12,6)}"
             + $" {pp((a20*q0+a21*q1+d22*q2+a32*q3)/lambda,12,6)}"
             + $" {pp((a30*q0+a31*q1+a32*q2+d33*q3)/lambda,12,6)}"
             + "\nAq"
             + $" {pp(a00*q0+a10*q1+a20*q2+a30*q3,12,6)}"
             + $" {pp(a10*q0+a11*q1+a21*q2+a31*q3,12,6)}"
             + $" {pp(a20*q0+a21*q1+a22*q2+a32*q3,12,6)}"
             + $" {pp(a30*q0+a31*q1+a32*q2+a33*q3,12,6)}"
           );
      **/

      Quaternion q = new Quaternion(q1, q2, q3, q0);;
      q.Normalize();
      return q;
    }

    // Solve A q = 0, where A is 4x4 symmetric matrix.
    // Perform symmetric Gauss elimination of A to a 3x3 matrix S.
    // Let L be  I - (0 a10/a00 a20/a00 a30/a00)@(1 0 0 0), i.e.,
    //
    //       ( 1        0  0  0 )
    //   L = (-a10/a00  1  0  0 ).
    //       (-a20/a00  0  1  0 )
    //       (-a30/a00  0  0  1 )
    //
    // This results L@F@L^T ((L^T)^{-1} q) = (e^T@e + S) q', i.e.
    //
    //             ( a00 0   0   0   )
    //   L@F@L^T = ( 0   s11 s12 s13 ),
    //             ( 0   s21 s22 s23 )
    //             ( 0   s31 s32 s33 )
    //
    // and q^T = (-(l1 l2 l3)@(q1 q2 q3)^T q1 q2 q3).
    // (S)ij = aij-ai0*aj0/a00 (for i,j=1,2,3).
    //
    private static (float, float, float, float) find_nullvector (
        in float a00,
        in float a10, in float a11,
        in float a20, in float a21, in float a22,
        in float a30, in float a31, in float a32, in float a33)
    {
      // If |a00| ~ 0, compute by direct cofactor expansion.
      //
      if(abs(a00) < 1.0e-6){
        return (
                det3(a11, a21, a31,
                     a21, a22, a32,
                     a31, a32, a33),
               -det3(a10, a21, a31,
                     a20, a22, a32,
                     a30, a32, a33),
                det3(a10, a11, a31,
                     a20, a21, a32,
                     a30, a31, a33),
               -det3(a10, a11, a21,
                     a20, a21, a22,
                     a30, a31, a32)
              );
      }

      //const float degeneracy = 0.05f;
      float l0 = 1.0f/a00;
      float l1 = -a10*l0;
      float l2 = -a20*l0;
      float l3 = -a30*l0;

      float s11=a11+a10*l1;//s12=a12+a10*l2,  s13=a13+a10*l3
      float s21=a21+a20*l1,  s22=a22+a20*l2;//s23=a23+a20*l3
      float s31=a31+a30*l1,  s32=a32+a30*l2,  s33=a33+a30*l3;

      // Cofactor expansion for first column.
      float m11 =  s22*s33-s32*s32;
      float m21 = -s21*s33+s31*s32;
      float m31 =  s21*s32-s31*s22;
      float mm = abs(m11) + abs(m21) + abs(m31);

      //Debug.Log( "Null A     "
      //         + $"\n{pp(a00)} {pp(a10)} {pp(a20)} {pp(a30)}"
      //         + $"\n{pp(a10)} {pp(a11)} {pp(a21)} {pp(a31)}"
      //         + $"\n{pp(a20)} {pp(a21)} {pp(a22)} {pp(a32)}"
      //         + $"\n{pp(a30)} {pp(a31)} {pp(a32)} {pp(a33)}"
      //         + "\nNull S     "
      //         + $"\n{pp(s11)} {pp(s21)} {pp(s31)}"
      //         + $"\n{pp(s21)} {pp(s22)} {pp(s32)}"
      //         + $"\n{pp(s31)} {pp(s32)} {pp(s33)}"
      //         + $"\nDeterminants {m11} {m21} {m31} {mm}"
      //         + "\nSq         "
      //         + $" {pp(s11*m11+s21*m21+s31*m31)}"
      //         + $" {pp(s21*m11+s22*m21+s32*m31)}"
      //         + $" {pp(s31*m11+s32*m21+s33*m31)}"
      //         );

      if (mm < degeneracy)
      {
        //float m12 = 0.0f;
        float m22 = s11*s33 - s31*s31;
        float m32 =-s11*s32 + s21*s31;
        mm = abs(m22) + abs(m32);
        if(mm < degeneracy)
        {
          //float m13 = 0.0f;
          //float m23 = 0.0f;
          float m33 = s11*s22 - s21*s21;
          mm = abs(m33);
          if(mm < degeneracy)
          {
            if     (abs(s11) > 1.0e-4f){ return (-l1*s21+l2*s11, -s21,  s11, 0.0f); }
            else if(abs(s22) > 1.0e-4f){ return (-l2*s32+l3*s22, 0.0f, -s32,  s22); }
            else if(abs(s33) > 1.0e-4f){ return ( l1*s33-l3*s31,  s33, 0.0f, -s31); }
            else { return ( l1, 1.0f, 0.0f, 0.0f); }
          }
          else
          {
            return ( l3, 0.0f, 0.0f, 1.0f);
          }
        }
        else
        {
          return (l2*m22+l3*m32, 0.0f, m22, m32);
        }
      }
      else
      {
        return (l1*m11+l2*m21+l3*m31, m11, m21, m31);
      }
    }

    // find_a_cubic_root
    //
    // Find the largest root of Cardano form cubic using Viete's solution,
    //
    //   X^3 + p X + q = 0.
    //
    // X = S Y, where 2 sqrt(p/3)
    // (2 sqrt(p/3) Y)^3 + p (2 sqrt(p/3) Y) = -q.
    // 4/3 p (2 sqrt(p/3)) Y^3 + p (2 sqrt(p/3)) Y = -q.
    // 4/3 pSY^3 + pSY = -q.
    // 4 Y^3 + 3 Y = -3(q/pS).
    // Y = cos_acos(-3(q/pS))
    // X = S cos_acos(-3(q/pS))
    //
    private static float find_a_cubic_root(in float p, in float q)
    {
      if (abs(p) < threshold)
      {
        if(abs(q) < threshold)
        {
          // X^3 = 0
          return 0.0f;
        }
        else
        {
          // X^3 = -q
          return -cbrt(q);
        }
      }
      else if(abs(q) < threshold)
      {
        // X^2 = -p
        if (p < 0.0f)
        {
          return sqrt(-p);
        }
        else
        {
          return 0.0f;
        }
      }

      if(p < 0.0f)
      {
        float s = 2.0f * sqrt(- (1.0f/3.0f) * p);
        float h = - (1.0f/3.0f) * p * s; // > 0

        if (h < abs(q))
        {
          if(q < 0.0f)
          {
            return s * cosh_acosh3(- h / q); // 0 < x < 1
          }
          else
          {
            return - s * cosh_acosh3(h / q); // 0 < x < 1
          }
        }
        else
        {
          return s * cos_acos3(-q / h); // -1 <= x <= 1;
        }
      }
      else
      {
        float s = 2.0f * sqrt((1.0f/3.0f) * p);
        return -s * sinh_asinh3(3.0f * q / (p * s));
      }
    }

    // Solve X^4 - 2 X^2 + a X + b = 0.
    private static float find_a_quartic_root(in float a, in float b)
    {
      if (abs(a) < threshold){
        // Solve X^4 - 2 X^2 + b = 0, using quadratic formula.
        if(b>1.0f)
        {
          return 0.0f;
        }
        else
        {
          return sqrt(max(1.0f + sqrt(1.0f - b), 0.0f));
        }
      }

      // Compute the largest candidate solutions.
      float x0 = find_a_quartic_root_third_order(a, b);
      // Check discriminant.
      float disc = discriminant(a, b);

      if (abs(disc) < 1.0e-6){
        return x0; // Since it is degenerate, a cubic solution is adopted.
      }

      // Refinement the root by NewtonRaphson method.
      float xx, f, df, x = x0;
      for (int i=0; i<20; i++)
      {
        xx = x * x;
        df = - 2.0f + xx;
        f = df * xx + a * x + b;
        if (abs(f) < threshold) {break;}
        df = a + 2.0f * x * (df + xx);
        if (abs(df) < threshold) {break;}
        f /= df;
        x -= f;
      }
      return x;
    }

    private static float find_a_quartic_root_third_order(in float k1, in float k0)
    {
      float y;

      // Find positive minimal point.
      y = find_a_cubic_root(-1.0f, 0.25f * k1);
      //Debug.Log("cubicA " + y + "  " + (4.0f * y*y*y-4.0f*y+k1));
      if(y < (1.0f/1.73205080757f)) // y < 1/sqrt(3)
      {
        // Taylor expansion around x = 1/sqrt(3) (positive inflection point)
        y = find_a_cubic_root(
            1.73205080757f * 0.25f * k1 - (2.0f/3.0f),
            0.25f * (k1 + 1.73205080757f * k0) - (5.0f*1.73205080757f/36.0f)
            ) + (1.0f/1.73205080757f);
      }
      else
      {
        // Taylor expansion around y (positive minimal point)
        float yy = y * y, y1 = 1.0f / y;
        float f = yy * (yy - 2.0f) + k1 * y + k0;
        float g = 0.5f * y - (1.0f/6.0f) * y1;
        y = find_a_cubic_root(- 3.0f * g * g, 0.25f * f * y1 + 2.0f * g * g * g) - g + y;
      }
      return y;
    }

    private static float discriminant(in float k1, in float k0)
    {
      float m0 = k1 * k1, kk = k0 - 1.0f;
      return -( 27.0f * m0 + (288.0f * 32.0f) * k0 ) * m0 + 256.0f * k0 * kk * kk;
    }

    private static float sqrt(in float x)
    {
      if (x <= 0.0f){ return 0.0f; }
      return 1.0f / invsqrt(x);
    }

    private static float invsqrt(in float x)
    {
      //float xh = 0.5f * x;
      //float f = BitConverter.ToSingle(BitConverter.GetBytes(0x5F375A86 - (BitConverter.ToInt32( BitConverter.GetBytes(x), 0)>>1)), 0);
      //f *= (1.5f - (xHalf * f * f));
      float f = BitConverter.ToSingle(BitConverter.GetBytes(0x5F1FFFF9 - (BitConverter.ToInt32( BitConverter.GetBytes(x), 0)>>1)), 0);
      f *= 0.703952253f * (2.38924456f - (x * f * f));
      f *= 0.5f * (3.0f - (x * f * f));
      f *= 0.5f * (3.0f - (x * f * f));
      return f;
    }

    private static float cbrt(in float x)
    {
      if (x == 0.0f){
        return 0.0f;
      }
      return 1.0f / invcbrt(x);
    }

    // Reference : https://www.mdpi.com/1996-1073/14/4/1058
    private static float invcbrt(in float x)
    {
      float y = abs(x);
      float f = BitConverter.ToSingle(BitConverter.GetBytes(0x548C2B4B - (BitConverter.ToInt32(BitConverter.GetBytes(y), 0) / 3)), 0);
      if(x < 0.0f){ f *= -1.0f; }
      float c = f * f * f * x;
      f *= 1.752319676f - c * (1.2509524245f - 0.5093818292f * c);
      //c = 1.0f - f * f * f * x;
      f *= 1.33333333333333f - 0.33333333333333f * f * f * f * x;
      return f;
    }

    private static float abs(in float x)
    {
      return x >= 0.0f ? x : -x;
    }

    private static float max(in float x, in float y)
    {
      return x > y ? x : y;
    }

    private static float det3_(in float c00, in float c01, in float c02,
                               in float c10, in float c11, in float c12,
                               in float c20, in float c21, in float c22)
    {
      return  c00 * (c11 * c22 - c12 * c21)
            - c10 * (c01 * c22 - c02 * c21)
            + c20 * (c01 * c12 - c11 * c02);
    }

    private static float det3(in float c00, in float c01, in float c02,
                              in float c10, in float c11, in float c12,
                              in float c20, in float c21, in float c22)
    {
      float abs00 = abs(c00);
      float abs10 = abs(c10);
      float abs20 = abs(c20);
      if (abs10 < abs00)
      {
        if (abs20 < abs00) // |c10| < |c00| and |c20| < |c00|
        {
          if(abs00 < threshold){
            return 0.0f;
          }
          float abs11 = abs(c11);
          float abs21 = abs(c21);
          if (abs11 > abs21)
          {
            return _det3p(c00, c01, c02,  // | 1 4 7 |
                          c10, c11, c12,  // | 2 5 8 |
                          c20, c21, c22); // | 3 6 9 |, pivot = 0
          }
          else
          {
            return - _det3p(c00, c01, c02,  // | 1 4 7 |
                            c20, c21, c22,  // | 3 6 9 |
                            c10, c11, c12); // | 2 5 8 |, pivot = 1
          }
        }
      }
      else
      {
        if (abs20 < abs10) // |c00| <= |c10| and |c20| < |c10|
        {
          if(abs10<threshold){
            return 0.0f;
          }
          float abs01 = abs(c01);
          float abs21 = abs(c21);
          if (abs01 > abs21)
          {
            return - _det3p(c10, c11, c12,  // | 2 5 8 |
                            c00, c01, c02,  // | 1 4 7 |
                            c20, c21, c22); // | 3 6 9 |, pivot = 1
          }
          else
          {
            return _det3p(c10, c11, c12,  // | 2 5 8 |
                          c20, c21, c22,  // | 3 6 9 |
                          c00, c01, c02); // | 1 4 7 |, pivot = 2
          }
        }
      }

      // |c10| <= |c20| and |c00| <= |c20|
      if(abs20 < threshold){ return 0.0f; }
      else
      {
        float abs01 = abs(c01);
        float abs11 = abs(c11);
        if (abs01 > abs11)
        {
          return _det3p(c20, c21, c22,  // | 3 6 9 |
                        c00, c01, c02,  // | 1 4 7 |
                        c10, c11, c12); // | 2 5 8 |, pivot = 2
        }
        else
        {
          return - _det3p(c20, c21, c22,  // | 3 6 9 |
                          c10, c11, c12,  // | 2 5 8 |
                          c00, c01, c02); // | 1 4 7 |, pivot = 1
        }
      }
    }

    private static float _det3p(in float c00, in float c01, in float c02,
                                in float c10, in float c11, in float c12,
                                in float c20, in float c21, in float c22)
    {
      return ( (c00 * c11 - c10 * c01) * (c00 * c22 - c20 * c02)
             - (c00 * c12 - c10 * c02) * (c00 * c21 - c20 * c01)
             ) / c00;
    }

    // compute cos( arccos(x) / 3 ) using spline expansion.
    private static float cos_acos3(in float x)
    {
      if (x < -1.0f)
      {
        return x;
      }
      else if (x > 1.0f)
      {
        return x;
      }

      float ret, yy;

      if (x < -0.333333333333333f)
      {
        yy = sqrt(x + 1.0f);
        if (x < -0.666666666666666f)
        {
          if (x < -0.90236892706218241f)
          {
            ret = (3.9847929621330180E-04f * yy + (2.7150043732313152E-03f)) * x
                + (2.7756772944468894E-03f * yy + (8.0584699091552827E-03f));
            ret = ret * x + (9.9787382004175255E-03f * yy + (-3.0422013335204556E-04f));
            ret = ret * x + (3.2124745520119027E-02f * yy + (-6.9480186781926440E-02f));
            ret = ret * x + (4.3277149578179819E-01f * yy + (4.3616749888734951E-01f));
          }
          else
          {
            ret = (3.9847929621330180E-04f * yy + (-8.1880793225013100E-04f)) * x
                + (2.7756772944468894E-03f * yy + (-5.3682021468007667E-03f));
            ret = ret * x + (9.9787382004175255E-03f * yy + (-1.9427315377984092E-02f));
            ret = ret * x + (3.2124745520119027E-02f * yy + (-8.1580601406840383E-02f));
            ret = ret * x + (4.3277149578179819E-01f * yy + ( 4.3329732093336737E-01f));
          }
        }
        else
        {
          if (x < -0.43096440627115074f)
          {
            ret = (3.9847929621330180E-04f * yy + (-1.3962794333934664E-03f)) * x
                + (2.7756772944468894E-03f * yy + (-6.7281227183138212E-03f));
            ret = ret * x + (9.9787382004175255E-03f * yy + (-2.0640121298405912E-02f));
            ret = ret * x + (3.2124745520119027E-02f * yy + (-8.2067600037457583E-02f));
            ret = ret * x + (4.3277149578179819E-01f * yy + ( 4.3322280904921723E-01f));
          }
          else
          {
            ret = (3.9847929621330180E-04f * yy + (1.1713621213896104E-02f)) * x
                + (2.7756772944468894E-03f * yy + (1.3560409301341475E-02f));
            ret = ret * x + (9.9787382004175255E-03f * yy + (-8.8387594483930760E-03f));
            ret = ret * x + (3.2124745520119027E-02f * yy + (-7.9004467646912643E-02f));
            ret = ret * x + (4.3277149578179819E-01f * yy + ( 4.3352276164963782E-01f));
          }
        }
      }
      else if (x < 0.3333333333333333E+00f)
      {
        if (x < 2.0410779985789219E-17f)
        {
          if (x < -0.23570226039551581f)
          {
            ret = (6.3261501181342716E-01f) * x + (7.7758615573901924E-01f);
            ret = ret * x + (2.7435256739631902E-01f);
            ret = ret * x + (2.2752170341132330E-01f);
            ret = ret * x + (8.7030281603695292E-01f);
          }
          else
          {
            ret = (-5.1407513495624758E-02f) * x + (1.0016161738288792E-02f);
            ret = ret * x + (-5.0149303892811525E-02f);
            ret = ret * x + ( 1.6657415217801974E-01f);
            ret = ret * x + ( 8.6602540378443871E-01f);
          }
        }
        else
        {
          if (x < 2.3570226039551587E-01f)
          {
            ret = (9.3008662532715076E-03f) * x + (1.4372359674536180E-02f);
            ret = ret * x + (-4.6656484877630987E-02f);
            ret = ret * x + ( 1.6659959026260604E-01f);
            ret = ret * x + ( 8.6602540378443871E-01f);
          }
          else
          {
            ret = (-0.34902175486090642f) * x + (0.41033381483828746f);
            ret = ret * x + (-0.21260486589396638f);
            ret = ret * x + ( 0.19761921280113542f);
            ret = ret * x + ( 0.86385435215153961f);
          }
        }
      }
      else if (x <= 1.0f)
      {
        if (x < 0.66666666666666666f)
        {
          if (x < 0.43096440627115085f)
          {
            ret = (0.092989836420019442f) * x + (-0.13124380721245635f);
            ret = ret * x + (0.039551533881730334f);
            ret = ret * x + (0.14461452858482687f );
            ret = ret * x + (0.86810669969142074f );
          }
          else
          {
            ret = (-6.7055964504391377E-03f) * x + (2.2911999129408750E-02f);
            ret = ret * x + (-0.050039084385563551f);
            ret = ret * x + ( 0.16784857275128981f );
            ret = ret * x + ( 0.86583329929197761f );
          }
        }
        else
        {
          if (x < 0.90236892706218252f)
          {
            ret = (-1.0881827591406440E-03f) * x + (9.1295179354582787E-03f);
            ret = ret * x + (-0.037133080284270058f);
            ret = ret * x + ( 0.16236757221824985f );
            ret = ret * x + ( 0.86672538337508409f );
          }
          else
          {
            ret = (4.6017500027684044E-03f) * x + (-1.2962733889034270E-02f);
            ret = ret * x + (-5.1111183599694618E-03f);
            ret = ret * x + ( 1.4181596019335754E-01f);
            ret = ret * x + ( 8.7165614205287767E-01f);
          }
        }
      }
      else
      {
        return x;
      }
      // Refinement by NewtonRaphson method.
      //
      // float df;
      // for (int i=0; i < 2; i++)
      // {
      //   yy = ret * ret;
      //   df = 12.0f * yy - 3.0f;
      //   if (ABS(df) < 1E-18f){break;}
      //   df = ((4.0f * yy - 3.0f) * ret - 1.0f / x) / df;
      //   ret = ret - df;
      //   if (abs(df) < 1.0E-6){break;}
      // }
      return ret;
    }

    // compute cosh( arccosh(1/x) / 3 ) using spline expansion.
    private static float cosh_acosh3(in float x)
    {
      // Out of Range.
      if (x < 0.0f)
      {
        return 0.5f;
      }
      if (1.0f < x)
      {
        return 1.0f;
      }

      float ret, yy;

      if (x < 0.5f)
      {
        if (x < 0.14644660940672627f)
        {
          ret = 2.9546700898745084f * x - 0.68732996465706475f;
          ret = ret * x - 0.013519484012953734f;
          ret = ret * x - 0.0046741735498024902f;
          ret = ret * x;
        }
        else
        {
          ret = -0.10623675833993933f * x + 0.15654255346132895f;
          ret = ret * x - 0.096794886272540959f;
          ret = ret * x + 0.0018177193622178180f;
          ret = ret * x - 0.00040727563438808847f;
        }
      }
      else
      {
        if (x < 0.85355339059327373f)
        {
          ret = 0.023919197448076808f * x - 0.059903084576736369f;
          ret = ret * x + 0.050794386881566130f;
          ret = ret * x - 0.047880140922667278f;
          ret = ret * x + 0.0064652937375348487f;
        }
        else
        {
          ret = -0.13435433998222815f * x + 0.50040317286635916f;
          ret = ret * x - 0.69989274807980062f;
          ret = ret * x + 0.40248132481808807f;
          ret = ret * x - 0.095448197561904868f;
        }
      }
      // ret = 2**(-2/3) * x**(1/3) - 2**(-4/3) * x**(-1/3)
      yy = invcbrt(x);
      ret += 0.6299605249474366f / yy + 0.3968502629920499f * yy;

      // Refinement by NewtonRaphson method.
      float df, x1 = 1.0f/x;
      for (int i=0; i < 5; i++)
      {
        yy = ret * ret;
        df = 12.0f * yy - 3.0f;
        if (abs(df) < 1.0E-12){break;}
        df = ((4.0f * yy - 3.0f) * ret - x1) / df;
        ret = ret - df;
        if (abs(df) < 1.0E-6){break;}
      }

      return ret;
    }

    private static float sinh_asinh3(in float x)
    {

      float ret;
      if (abs(x) < 1.3f){
        // Third-order Maclaurin expansion.
        float xx = x * x;
        ret = x * ((1.0f/3.0f) - xx * (4.0f/81.0f));
      }
      else
      {
        // sinh(arcsinh(x)/3) approaches asymptotic to
        // 2**(-2/3) * x**(1/3) - 2**(-4/3) * x**(-1/3)
        // at x \to \pm \infty.
        float invcbrtx = invcbrt(x);
        ret = 0.6299605249474366f / invcbrtx - 0.3968502629920499f * invcbrtx;
      }

      // Refinement by Newton-Raphson method.
      float yy = ret * ret;
      ret -= ((4.0f * yy + 3.0f) * ret - x) / (12.0f * yy + 3.0f);
      yy = ret * ret;
      ret -= ((4.0f * yy + 3.0f) * ret - x) / (12.0f * yy + 3.0f);

      return ret;
    }

    /******************
     Matrix operations
    ******************/

    private static float SquaredSum(in float[] x)
    {
      float ret = 0.0f;
      for (int i = 0; i < x.Length; i++){
        ret += x[i] * x[i];
      }
      return ret;
    }

    private static Matrix3x3 Cov(in float[] r, in float[] t)
    {
      float c00 = 0.0f, c01 = 0.0f, c02 = 0.0f;
      float c10 = 0.0f, c11 = 0.0f, c12 = 0.0f;
      float c20 = 0.0f, c21 = 0.0f, c22 = 0.0f;
      int iy, iz;
      // r.Length >= t.Length, always.
      for (int ix = 0; ix < t.Length; ix+=3){
        iy = ix + 1; iz = iy + 1;
        c00 += t[ix]*r[ix]; c01 += t[ix]*r[iy]; c02 += t[ix]*r[iz];
        c10 += t[iy]*r[ix]; c11 += t[iy]*r[iy]; c12 += t[iy]*r[iz];
        c20 += t[iz]*r[ix]; c21 += t[iz]*r[iy]; c22 += t[iz]*r[iz];
      }
      return new Matrix3x3(c00, c01, c02, c10, c11, c12, c20, c21, c22);
    }

    private static SymmetricMatrix3x3 SelfCov(in float[] x)
    {
      float c00 = 0.0f;
      float c10 = 0.0f, c11 = 0.0f;
      float c20 = 0.0f, c21 = 0.0f, c22 = 0.0f;
      int iy, iz;
      for (int ix = 0; ix < x.Length; ix+=3){
        iy = ix + 1; iz = iy + 1;
        c00 += x[ix]*x[ix];
        c10 += x[iy]*x[ix]; c11 += x[iy]*x[iy];
        c20 += x[iz]*x[ix]; c21 += x[iz]*x[iy]; c22 += x[iz]*x[iz];
      }
      return new SymmetricMatrix3x3(c00, c10, c11, c20, c21, c22);
    }

    /******************
     * Util. functions
    ******************/

    // Pretty printer
    private static string pp(in float a, in int d=7, in int p=2)
    {
      return a.ToString("F" + p.ToString()).PadLeft(d, ' ');
    }

    // Returns |MM^t - I|^2
    private static float IsOrthogonal(Matrix3x3 M)
    {
      float e00 = M.x00*M.x00+M.x01*M.x01+M.x02*M.x02, e01 = M.x00*M.x10+M.x01*M.x11+M.x02*M.x12, e02 = M.x00*M.x20+M.x01*M.x21+M.x02*M.x22;
      float e10 = M.x10*M.x00+M.x11*M.x01+M.x12*M.x02, e11 = M.x10*M.x10+M.x11*M.x11+M.x12*M.x12, e12 = M.x10*M.x20+M.x11*M.x21+M.x12*M.x22;
      float e20 = M.x20*M.x00+M.x21*M.x01+M.x22*M.x02, e21 = M.x20*M.x10+M.x21*M.x11+M.x22*M.x12, e22 = M.x20*M.x20+M.x21*M.x21+M.x22*M.x22;
      return (e00-1.0f)*(e00-1.0f) +  e10*e10              +  e20*e20
            + e10*e10              + (e11-1.0f)*(e11-1.0f) +  e21*e21
            + e20*e20              +  e12*e12              + (e22-1.0f)*(e22-1.0f);
    }

    private static (float[], float[], Quaternion) PrepareX(in float c, in float[] x)
    {
      float[] x_ = Neaten(x);
      float[] y_ = new float[x_.Length];
      Vector3 t;
      Quaternion q = new Quaternion(c, c, c, 1.0f - c*c);
      q.Normalize();
      for (int i = 0; i < x_.Length; i+=3){
        t = QRot(x_[i+0], x_[i+1], x_[i+2], q);
        y_[i+0] = t.x;
        y_[i+1] = t.y;
        y_[i+2] = t.z;
      }
      return (x_, y_, q);
    }

    private static (float, float) QRrot(
      ref float a01, ref float a02,
      ref float a11, ref float a12,
      ref float a21, ref float a22)
    {
      float c, s, tmp;
      c = invsqrt(a21*a21+a22*a22); s = a21 * c; c*= a22;
      // a21 becomes 0.
      tmp = c * a01 - s * a02; a02 = s * a01 + c * a02; a01 = tmp;
      tmp = c * a11 - s * a12; a12 = s * a11 + c * a12; a11 = tmp;
      tmp = c * a21 - s * a22; a22 = s * a21 + c * a22; a21 = tmp;
//    a22 = s * a21 + c * a22;
//    a21 = 0.0f;
      return (c, s);
    }

    private static float[] Neaten(in float[] x)
    {
      float[] x_ = new float[x.Length];
      for (int i = 0; i < x_.Length; i++){ x_[i] = x[i]; }
      Centering(ref x_, Centroid(x));
      return x_;
    }

    public static Vector3 QRot(in float x1, in float x2, in float x3, in Quaternion q)
    {
      const float x0=1.0f;
      float qx0 = q.w*x0+q.x*x1+q.y*x2+q.z*x3,
            qx1 = q.w*x1-q.x*x0+q.y*x3-q.z*x2,
            qx2 = q.w*x2-q.x*x3-q.y*x0+q.z*x1,
            qx3 = q.w*x3+q.x*x2-q.y*x1-q.z*x0;
      return new Vector3 (
          qx0*q.x+qx1*q.w-qx2*q.z+qx3*q.y,
          qx0*q.y+qx1*q.z+qx2*q.w-qx3*q.x,
          qx0*q.z-qx1*q.y+qx2*q.x+qx3*q.w);
    }

    private static Matrix3x3 QuaternionToRotMat(Quaternion q)
    {
      const float sqrt2 = 1.41421356237f;
      float p0 = sqrt2 * q.w, p1 = sqrt2 * q.x, p2 = sqrt2 * q.y, p3 = sqrt2 * q.z;
      float p10 = p1 * p0;
      float p20 = p2 * p0, p21 = p2 * p1;
      float p30 = p3 * p0, p31 = p3 * p1, p32 = p3 * p2;
      float q00 = q.w*q.w, q11 = q.x*q.x, q22 = q.y*q.y, q33 = q.z*q.z;
      return new Matrix3x3(
          q00+q11-q22-q33, p21-p30,         p31+p20,
          p21+p30,         q00-q11+q22-q33, p32-p10,
          p31-p20,         p32+p10,         q00-q11-q22+q33);
    }

    /******************
        Debug utils
    ******************/

    public static void DebugUnits()
    {
      int n = 10000;
      float k;

      Debug.Log("=== math functions ===");
      unittest("sqrt",        (x)=>sqrt(x),        (x,y)=>x-y*y,                      n, 0.0f, 100.0f);
      unittest("cbrt",        (x)=>cbrt(x),        (x,y)=>x-y*y*y,                    n, 0.0f, 100.0f);
      unittest("invsqrt",     (x)=>invsqrt(x),     (x,y)=>x-1.0f/(y*y),               n, 0.0001f, 100.0f);
      unittest("invcbrt",     (x)=>invcbrt(x),     (x,y)=>x-1.0f/(y*y*y),             n, 0.0001f, 100.0f);

      Debug.Log("=== special functions ===");
      unittest("cos_acos3",   (x)=>cos_acos3(x),   (x,y)=>x-(4.0f*y*y-3.0f)*y,        n, -1.0f, 1.0f);
      unittest("cosh_acosh3", (x)=>cosh_acosh3(x), (x,y)=>x-1.0f/((4.0f*y*y-3.0f)*y), n,  0.001f, 1.0f);
      unittest("sinh_asinh3", (x)=>sinh_asinh3(x), (x,y)=>x-((4.0f*y*y+3.0f)*y),      n,  0.0f, 100.0f);

      Debug.Log("=== cubic solver ===");
      k =-2.0f;   unittest("find_a_cubic_root p=" + k.ToString("F3"), (x)=>find_a_cubic_root(k, x), (x,y)=>(y*y*y+k*y+x), n, -100.0f, 100.0f);
      k =-1.0f;   unittest("find_a_cubic_root p=" + k.ToString("F3"), (x)=>find_a_cubic_root(k, x), (x,y)=>(y*y*y+k*y+x), n, -100.0f, 100.0f);
      k = 0.0f;   unittest("find_a_cubic_root p=" + k.ToString("F3"), (x)=>find_a_cubic_root(k, x), (x,y)=>(y*y*y+k*y+x), n, -100.0f, 100.0f);
      k = 1.0f;   unittest("find_a_cubic_root p=" + k.ToString("F3"), (x)=>find_a_cubic_root(k, x), (x,y)=>(y*y*y+k*y+x), n, -100.0f, 100.0f);
      k = 2.0f;   unittest("find_a_cubic_root p=" + k.ToString("F3"), (x)=>find_a_cubic_root(k, x), (x,y)=>(y*y*y+k*y+x), n, -100.0f, 100.0f);

      Debug.Log("=== quartic solver ===");
      k = 1.0f;   unittest("find_a_quartic_root x=" + k.ToString("F3"), (x)=>find_a_quartic_root(x,-k*x-k*k*(k*k-2.0f)), (x,y)=>((y*y-2.0f)*y*y+x*y+(-k*x-k*k*(k*k-2.0f))), n,-100.0f, 100.0f);
      k = 1.001f; unittest("find_a_quartic_root x=" + k.ToString("F3"), (x)=>find_a_quartic_root(x,-k*x-k*k*(k*k-2.0f)), (x,y)=>((y*y-2.0f)*y*y+x*y+(-k*x-k*k*(k*k-2.0f))), n,-100.0f, 100.0f);
      k = 1.01f;  unittest("find_a_quartic_root x=" + k.ToString("F3"), (x)=>find_a_quartic_root(x,-k*x-k*k*(k*k-2.0f)), (x,y)=>((y*y-2.0f)*y*y+x*y+(-k*x-k*k*(k*k-2.0f))), n,-100.0f, 100.0f);
      k = 1.1f;   unittest("find_a_quartic_root x=" + k.ToString("F3"), (x)=>find_a_quartic_root(x,-k*x-k*k*(k*k-2.0f)), (x,y)=>((y*y-2.0f)*y*y+x*y+(-k*x-k*k*(k*k-2.0f))), n,-100.0f, 100.0f);
      k = 1.2f;   unittest("find_a_quartic_root x=" + k.ToString("F3"), (x)=>find_a_quartic_root(x,-k*x-k*k*(k*k-2.0f)), (x,y)=>((y*y-2.0f)*y*y+x*y+(-k*x-k*k*(k*k-2.0f))), n,-100.0f, 100.0f);
      k = 2.0f;   unittest("find_a_quartic_root x=" + k.ToString("F3"), (x)=>find_a_quartic_root(x,-k*x-k*k*(k*k-2.0f)), (x,y)=>((y*y-2.0f)*y*y+x*y+(-k*x-k*k*(k*k-2.0f))), n,-100.0f, 100.0f);
      k = 5.0f;   unittest("find_a_quartic_root x=" + k.ToString("F3"), (x)=>find_a_quartic_root(x,-k*x-k*k*(k*k-2.0f)), (x,y)=>((y*y-2.0f)*y*y+x*y+(-k*x-k*k*(k*k-2.0f))), n,-100.0f, 100.0f);

      Debug.Log("=== find nullvector ===");
      unittest_null_vector(
        "Degenerate none", new float[] {10.0f, sqrt(10.0f), -sqrt(10.0f), 0.0f},
        1.0f,
        2.0f, 4.0f,
        3.0f, 1.0f, 4.0f,
        4.0f, 3.0f, 2.0f, 1.0f);
      unittest_null_vector(
        "Double degenerate 1", new float[] {16.0f, 12.0f-sqrt(154.0f), 12.0f+sqrt(154.0f)},
        20.0f,
        2.0f*sqrt(3.0f), 16.0f,
        sqrt(3.0f),       6.0f, 10.0f,
        sqrt(3.0f),       6.0f, -6.0f, 10.0f);
      unittest_null_vector(
        "Double degenerate 2", new float[] {-3.0f, -1.0f, 3.0f},
        1.0f,
        0.0f, 0.0f,
        0.0f, 3.0f, 0.0f,
        2.0f, 0.0f, 0.0f, 1.0f);
      unittest_null_vector(
        "Double degenerate 3", new float[] {-3.0f, -1.0f, 3.0f},
        0.0f,
        0.0f, 1.0f,
        3.0f, 0.0f, 0.0f,
        0.0f, 2.0f, 0.0f, 1.0f);
      unittest_null_vector(
        "Double degenerate 4", new float[] {-3.0f, -1.0f, 3.0f},
        0.0f,
        3.0f, 0.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 2.0f, 1.0f);
      unittest_null_vector(
        "Double degenerate 5", new float[] {-1.0f, 1.0f},
        0.0f,
        0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        1.0f, 0.0f, 0.0f, 0.0f);
      unittest_null_vector(
        "Triple degenerate 1", new float[] {1.0f, 2.0f},
        1.0f,
        0.0f, 2.0f,
        0.0f, 0.0f, 2.0f,
        0.0f, 0.0f, 0.0f, 2.0f);
      unittest_null_vector(
        "Triple degenerate 2", new float[] {1.0f, 2.0f},
        0.0f,
        0.0f, 1.0f,
        2.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 2.0f);
      unittest_null_vector(
        "Triple degenerate 3", new float[] {3.0f,-1.0f},
        0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f, 0.0f,
        1.0f, 1.0f, 1.0f, 0.0f);

      Debug.Log("=== Lambda max ===");
      unittest_lambda_max(
          "N = 1", 0.0f,
          new float[] {1.0f, 2.0f, 6.0f}
          );
      unittest_lambda_max(
          "N = 2", 0.5f,
          new float[] {1.0f, 2.0f, 6.0f, 4.0f, 0.0f, 6.0f}
          );
      unittest_lambda_max(
          "N = 3", 0.2f,
          new float[] {1.0f,-2.0f, 3.0f, 1.0f, 2.0f, 3.0f, 2.0f, 1.0f, 3.0f}
          );
      unittest_lambda_max(
          "N = 4", 0.0f,
          new float[] {1.0f,0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f}
          );
      unittest_lambda_max(
          "N = 4", 0.8f,
          new float[] {1.0f,0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f}
          );

      Debug.Log("=== Rotation ===");
      unittest_rotation(
          "N = 2, identity", 0.0f,
          new float[] {1.0f, 2.0f, 6.0f, 4.0f, 0.0f, 6.0f}
          );
      unittest_rotation(
          "N = 2, rotation", 0.5f,
          new float[] {1.0f, 2.0f, 6.0f, 4.0f, 0.0f, 6.0f}
          );
      unittest_rotation(
          "N = 3, identity", 0.0f,
          new float[] {1.0f,-2.0f, 3.0f, 1.0f, 2.0f, 3.0f, 2.0f, 1.0f, 3.0f}
          );
      unittest_rotation(
          "N = 3, rotation",-0.5f,
          new float[] {1.0f,-2.0f, 3.0f, 1.0f, 2.0f, 3.0f, 2.0f, 1.0f, 3.0f}
          );
      unittest_rotation(
          "N = 4, identity", 0.0f,
          new float[] {1.0f,0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f}
          );
      unittest_rotation(
          "N = 4, rotation", 0.5f,
          new float[] {1.0f,0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f}
          );

      Debug.Log("=== QR factorization ===");
      //Debug.Log(qr3x3(new Matrix3x3(1.0f, 2.0f, 1.0f, 1.0f, 2.0f, 4.0f, 2.0f, 3.0f, 4.0f)).ToString());
      //Debug.Log(qr3x3(new Matrix3x3(0.0f, 2.0f, 1.0f, 1.0f, 1.0f, 4.0f, 2.0f, 3.0f, 4.0f)).ToString());
      //Debug.Log(qr3x3(new Matrix3x3(3.0f, 2.0f, 1.0f, 1.0f, 5.0f, 4.0f, 2.0f, 3.0f, 4.0f)).ToString());
      //Debug.Log(qr3x3(new SymmetricMatrix3x3(4.0f,-2.0f, 1.0f, 1.0f, 2.0f, 4.0f)).ToString());
    }

    private static void unittest(
        in string title,
        Func<float, float> rfun,
        Func<float, float, float> test,
        in int n,
        in float xmin,
        in float xmax
        )
    {
      var stopwatch = new System.Diagnostics.Stopwatch();
      float dx = (xmax - xmin) / n;
      float norm = 0.0f, nmax = 0.0f, narg = 0.0f,
            rnrm = 0.0f, rmax = 0.0f, rarg = 0.0f,
            rms = 0.0f,  y, diff;
      stopwatch.Reset();
      stopwatch.Start();
      for (float x = xmin; x < xmax; x+=dx)
      {
        y = rfun(x);
        diff = test(x, y);
        rms += diff * diff;
        if (rnrm < abs(diff)/max(abs(x), 1.0e-3f))
        {
          rnrm = abs(diff)/max(abs(x), 1.0e-3f);
          rmax = diff/max(abs(x), 1.0e-3f);
          rarg = x;
        }
        if (norm < abs(diff))
        {
          norm = max(norm, abs(diff));
          nmax = diff;
          narg = x;
        }
      }
      stopwatch.Stop();
      rms = sqrt(rms/n);
      float ts = (float) stopwatch.ElapsedTicks / (float) (n * System.Diagnostics.Stopwatch.Frequency);
      Debug.Log("Unittest [ " + title + " ]"
          + "\n----Summary-----  Norm : " + nmax.ToString("F4") + " RMS : " + rms.ToString("F4")
          + " Time/Run : " + (1000000000.0f * ts).ToString("F2") + " nanosec ( " + (1.0f/ts).ToString("F2") + " FPS )"
          + "\n----MaxError----  " + nmax.ToString("F8") + " where [ " + narg.ToString("F8") + " ]"
          + "\n--MaxRelError---  " + rmax.ToString("F8") + " where [ " + rarg.ToString("F8") + " ]"
          + "\n------RMS-------  " + rms.ToString("F8")
          + "\n-----Range------  [ " + xmin.ToString("F2") + ", " +  xmax.ToString("F2") + " ]"
          + "\n--SampleNumber--  " + n
          + "\n--ElapsedTime---  " + stopwatch.Elapsed
          );
    }

    private static void unittest_null_vector(
        in string title, in float[] l,
        in float a00,
        in float a10, in float a11,
        in float a20, in float a21, in float a22,
        in float a30, in float a31, in float a32, in float a33
        )
    {
      float q0, q1, q2, q3, b00, b11, b22, b33;;
      foreach (float li in l)
      {
        b00 = a00 - li;
        b11 = a11 - li;
        b22 = a22 - li;
        b33 = a33 - li;
        (q0, q1, q2, q3) =
        find_nullvector (b00,
                         a10, b11,
                         a20, a21, b22,
                         a30, a31, a32, b33);
        float aq0 = b00*q0+a10*q1+a20*q2+a30*q3;
        float aq1 = a10*q0+b11*q1+a21*q2+a31*q3;
        float aq2 = a20*q0+a21*q1+b22*q2+a32*q3;
        float aq3 = a30*q0+a31*q1+a32*q2+b33*q3;
        float nrm = max(max(max(abs(aq0), abs(aq1)), abs(aq2)), abs(aq3));
        float rms = aq0*aq0 + aq1*aq1 + aq2*aq2 + aq3*aq3;

        Debug.Log("[ " + title + " ] "
                + "norm : " + pp(nrm) + " rms : " + pp(rms)
                + "\nA =  | " + pp(a00) + pp(a10) + pp(a20) + pp(a30) + " |"
                + "\n     | " + pp(a10) + pp(a11) + pp(a21) + pp(a31) + " |"
                + "\n     | " + pp(a20) + pp(a21) + pp(a22) + pp(a32) + " |"
                + "\n     | " + pp(a30) + pp(a31) + pp(a32) + pp(a33) + " |"
                + "\nq =  | " + pp(q0)  + pp(q1)  + pp(q2)  + pp(q3)  + " |"
                + "\nAq = | " + pp(aq0) + pp(aq1) + pp(aq2) + pp(aq3) + " |"
                + "\neigenvalue = " + pp(li, 9, 6)
                );
      }
    }

    private static void unittest_lambda_max(in string title, in float c, in float[] x)
    {
      (float[] x_, float[] y_, Quaternion q) = PrepareX(c, x);
      Matrix3x3 C = Cov(x_, y_);
      float lmax = FindLambdaMax(C);
      float sd = SquaredSum(x_)+SquaredSum(y_)-2.0f*lmax;

      string coords = "\nCoordinates";
      for (int i = 0; i < x_.Length; i+=3){
        coords += $"\n{i/3+1} | {pp(x_[i+0],6,2)} {pp(x_[i+1],6,2)} {pp(x_[i+2],6,2)} | {pp(y_[i+0],6,2)} {pp(y_[i+1],6,2)} {pp(y_[i+2],6,2)} |";
      }

      Debug.Log($"Unittest lambda_max [ {title} ]  sd : {pp(sd)}"
                      + $"\nGX {pp(SquaredSum(x_))} GY {pp(SquaredSum(y_))} Lambda {pp(lmax)}"
                      + $"\nRMSD = (GX + GY - 2 Lambda) = {pp(sd,16,12)}"
                      + $"\nRotation" + QuaternionToRotMat(q).ToString()
                      + coords
                      );
    }

    private static void unittest_rotation(in string title, in float c, in float[] x)
    {
      (float[] x_, float[] y_, Quaternion q) = PrepareX(c, x);

      Matrix3x3 C = Cov(x_, y_);
      float lmax = FindLambdaMax(C);
      Quaternion qs = GetSuperpose(C, lmax);

      float sd=0.0f;
      string coords = "\nCoordinates";
      Vector3 t;
      for (int i = 0; i < x_.Length; i+=3){
        t = QRot(y_[i+0], y_[i+1], y_[i+2], qs);
        sd += (x_[i+0] - t.x)*(x_[i+0] - t.x);
        sd += (x_[i+1] - t.y)*(x_[i+1] - t.y);
        sd += (x_[i+2] - t.z)*(x_[i+2] - t.z);
        coords += $"\n{i/3+1} | {pp(x_[i+0],6,2)} {pp(x_[i+1],6,2)} {pp(x_[i+2],6,2)} |"
                + $" {pp(y_[i+0],6,2)} {pp(y_[i+1],6,2)} {pp(y_[i+2],6,2)} |"
                + $" {pp(t.x,6,2)} {pp(t.y,6,2)} {pp(t.z,6,2)} |";
      }

      Debug.Log($"Unittest rotation [ {title} ]  SD : {pp(sd)}"
              + $"  IsOrthogonal : {pp( IsOrthogonal(QuaternionToRotMat(qs)))}"
              + $"\nGX     {pp(SquaredSum(x_),9,6)} GY      {pp(SquaredSum(y_),9,6)}"
              + $"\nLambda {pp(lmax,9,6)}"
              + $"\nGX+GY-2Lambda {pp(SquaredSum(x_)+SquaredSum(y_)-2.0f*lmax,9,6)}"
              + $"\nOriginal Rotation" + QuaternionToRotMat(q).ToString()
              + $"\nReverse Rotation" + QuaternionToRotMat(qs).ToString()
              + coords
              );
    }

//  public static void main(string[] args)
//  {
//    DebugUnits();
//  }

    /******************
    Under construction!
    ******************/

    private static Matrix3x3 qr3x3(in Matrix3x3 x)
    {
      float l00, l01, l02;
      float l10, l11, l12;
      float l20, l21, l22;
      Matrix3x3 R;

      if(abs(x.x22)>abs(x.x00))
      {
        if(abs(x.x22)>abs(x.x11))
        {
          if(abs(x.x11)>abs(x.x00))
          {  // |x22| > |x11| > |x00|
            l00 = x.x00; l01 = x.x01; l02 = x.x02;
            l10 = x.x10; l11 = x.x11; l12 = x.x12;
            l20 = x.x20; l21 = x.x21; l22 = x.x22;
          }
          else
          {  // |x22| > |x00| >= |x11|
            l00 = x.x11; l01 = x.x10; l02 = x.x12;
            l10 = x.x01; l11 = x.x00; l12 = x.x02;
            l20 = x.x21; l21 = x.x20; l22 = x.x22;
          }
        }
        else
        {  // |x11| > |x22| > |x00|
           l00 = x.x00; l01 = x.x02; l02 = x.x01;
           l10 = x.x20; l11 = x.x22; l12 = x.x21;
           l20 = x.x10; l21 = x.x12; l22 = x.x11;
        }
      }
      else
      {
        if(abs(x.x11)>abs(x.x00))
        {  // |x11| > |x00| >= |x22|
           l00 = x.x22; l01 = x.x20; l02 = x.x21;
           l10 = x.x02; l11 = x.x00; l12 = x.x01;
           l20 = x.x12; l21 = x.x10; l22 = x.x11;
        }
        else
        {
          if(abs(x.x22)>abs(x.x11))
          { // |x00| >= |x22| > |x11|
            l00 = x.x11; l01 = x.x12; l02 = x.x10;
            l10 = x.x21; l11 = x.x22; l12 = x.x20;
            l20 = x.x01; l21 = x.x02; l22 = x.x00;
          }
          else
          { // |x00| >=|x11| >= |x22|
            l00 = x.x22; l01 = x.x21; l02 = x.x20;
            l10 = x.x12; l11 = x.x11; l12 = x.x10;
            l20 = x.x02; l21 = x.x01; l22 = x.x00;
          }
        }
      }

      (float c1, float s1) = QRrot(ref l01, ref l02,
                                   ref l11, ref l12,
                                   ref l21, ref l22);
      (float c2, float s2) = QRrot(ref l10, ref l12,
                                   ref l00, ref l02,
                                   ref l20, ref l22);
      (float c3, float s3) = QRrot(ref l20, ref l21,
                                   ref l00, ref l01,
                                   ref l10, ref l11);
      R = EMxyz(c1, s1, c2, s2, c3, s3);

      Debug.Log(pp(l00) + pp(l01) + pp(l02) + "\n"
              + pp(l10) + pp(l11) + pp(l12) + "\n"
              + pp(l20) + pp(l21) + pp(l22) + "\n."
          );
      Debug.Log(pp(R.x00) + pp(R.x01) + pp(R.x02) + "\n"
              + pp(R.x10) + pp(R.x11) + pp(R.x12) + "\n"
              + pp(R.x20) + pp(R.x21) + pp(R.x22) + "\n."
          );
      Debug.Log(pp(l00*R.x00+l01*R.x01+l02*R.x02) + pp(l00*R.x10+l01*R.x11+l02*R.x12) + pp(l00*R.x20+l01*R.x21+l02*R.x22) + "\n"
              + pp(l10*R.x00+l11*R.x01+l12*R.x02) + pp(l10*R.x10+l11*R.x11+l12*R.x12) + pp(l10*R.x20+l11*R.x21+l12*R.x22) + "\n"
              + pp(l20*R.x00+l21*R.x01+l22*R.x02) + pp(l20*R.x10+l21*R.x11+l22*R.x12) + pp(l20*R.x20+l21*R.x21+l22*R.x22) + "\n."
          );

      Debug.Log(pp(R.x00*R.x00+R.x01*R.x01+R.x02*R.x02) + pp(R.x00*R.x10+R.x01*R.x11+R.x02*R.x12) + pp(R.x00*R.x20+R.x01*R.x21+R.x02*R.x22) + "\n"
              + pp(R.x10*R.x00+R.x11*R.x01+R.x12*R.x02) + pp(R.x10*R.x10+R.x11*R.x11+R.x12*R.x12) + pp(R.x10*R.x20+R.x11*R.x21+R.x12*R.x22) + "\n"
              + pp(R.x20*R.x00+R.x21*R.x01+R.x22*R.x02) + pp(R.x20*R.x10+R.x21*R.x11+R.x22*R.x12) + pp(R.x20*R.x20+R.x21*R.x21+R.x22*R.x22) + "\n."
          );

      return new Matrix3x3 (0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    private static Matrix3x3 EMxyz(in float cx, in float sx, in float cy, in float sy, in float cz, in float sz)
    {
      return new Matrix3x3(
         cy*cz,           cy*sz,          sy,
        -sx*sy*cz-cx*sz, -sx*sy*sz+cx*cz, sx*cy,
        -cx*sy*cz+sx*sz, -cx*sy*sz-sx*cz, cx*cy);
    }

    private static Matrix3x3 EMxzy(in float cx, in float sx, in float cy, in float sy, in float cz, in float sz)
    {
      return new Matrix3x3(
         cy*cz,          -sz,    sy*cz,
         cx*cy*sz+sx*sy,  cx*cz, cx*sy*sz-sx*cy,
         sx*cy*sz-cx*sy,  sx*cz, sx*sy*sz+cx*cy);
    }

    private static Matrix3x3 EMyxz(in float cx, in float sx, in float cy, in float sy, in float cz, in float sz)
    {
      return new Matrix3x3(
         sx*sy*sz+cy*cz,  sx*sy*cz-cy*sz, cx*sy,
         cx*cz,           cz*cz,         -sx,
         sx*cy*sz-cy*sz,  sx*cy*cz+sy*sz, cx*cy);
    }
  }

