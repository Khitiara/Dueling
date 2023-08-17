using System.Numerics;
using System.Runtime.CompilerServices;

namespace Dueling;

public struct DualQuaternion
    :
#if NET7_0_OR_GREATER
        IEqualityOperators<DualQuaternion, DualQuaternion, bool>,
        IAdditionOperators<DualQuaternion, DualQuaternion, DualQuaternion>,
        IMultiplyOperators<DualQuaternion, DualQuaternion, DualQuaternion>,
        IMultiplyOperators<DualQuaternion, float, DualQuaternion>,
        ISubtractionOperators<DualQuaternion, DualQuaternion, DualQuaternion>,
        IAdditiveIdentity<DualQuaternion, DualQuaternion>,
        IMultiplicativeIdentity<DualQuaternion, DualQuaternion>,
        IDivisionOperators<DualQuaternion, DualQuaternion, DualQuaternion>,
        IDivisionOperators<DualQuaternion, float, DualQuaternion>,
#endif
        IEquatable<DualQuaternion>
{
    public Quaternion Real;
    public Quaternion Dual;

    public bool Equals(DualQuaternion other) => Real.Equals(other.Real) && Dual.Equals(other.Dual);

    public override bool Equals(object? obj) => obj is DualQuaternion other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(Real, Dual);

    public static bool operator ==(DualQuaternion left, DualQuaternion right) => left.Equals(right);

    public static bool operator !=(DualQuaternion left, DualQuaternion right) => !left.Equals(right);

    public static DualQuaternion Zero => default;
    public static DualQuaternion Identity => new(Quaternion.Identity, default);

    public DualQuaternion(Quaternion real, Quaternion dual) {
        Real = real;
        Dual = dual;
    }

    public DualQuaternion(Vector3 realVector, float realW, Vector3 dualVector, float dualW) :
        this(new Quaternion(realVector, realW), new Quaternion(dualVector, dualW)) { }

    public float Length() => Real.Length();

    public static DualQuaternion CreateFromRotationTranslation(Quaternion rotation, Vector3 translation) {
        Quaternion real = Quaternion.Normalize(rotation);
        return new DualQuaternion(real, new Quaternion(translation, 0) * real * 0.5f);
    }

    public static DualQuaternion Normalize(DualQuaternion dq) => dq / dq.Length();

    public static DualQuaternion Conjugate(DualQuaternion dq) =>
        new(Quaternion.Conjugate(dq.Real), Quaternion.Conjugate(dq.Dual));

    public static DualQuaternion Inverse(DualQuaternion dq) {
        Quaternion q = Quaternion.Conjugate(dq.Real);
        return new DualQuaternion(q, q * dq.Dual * q * -1);
    }

    public static DualQuaternion operator +(DualQuaternion a, DualQuaternion b) =>
        new(a.Real + b.Real, a.Dual + b.Dual);

    public static DualQuaternion operator -(DualQuaternion a, DualQuaternion b) =>
        new(a.Real - b.Real, a.Dual - b.Dual);

    public static DualQuaternion operator *(DualQuaternion a, float b) => new(a.Real * b, a.Dual * b);

    public static DualQuaternion operator *(DualQuaternion a, DualQuaternion b) =>
        // TODO: figure out if these mults are backwards for how we want to use this - this is mathematically correct
        // but we may want to use reversed
        new(a.Real * b.Real, b.Real * a.Dual + b.Dual * a.Real);

    public static DualQuaternion operator /(DualQuaternion a, DualQuaternion b) => a * Inverse(b);

    public static DualQuaternion operator /(DualQuaternion a, float b) => a * (1f / b);

    public static void
        DecomposeRotationTranslation(DualQuaternion dq, out Quaternion rotation, out Vector3 translation) {
        rotation = dq.Real;
        Quaternion tQ = dq.Dual * 2f * Quaternion.Conjugate(dq.Real);
        translation = new Vector3(tQ.X, tQ.Y, tQ.Z);
    }

    public static DualQuaternion CreateFromScrewParameters(Vector3 pointOnAxis, Vector3 axisDirection, float pitch,
        float theta) {
        float d;
        if (float.IsInfinity(pitch)) {
            d = theta;
            theta = 0;
        } else {
            d = pitch * theta;
        }

        Vector3 moment = Vector3.Cross(pointOnAxis, axisDirection);
        float halfDistance = d / 2;
        (float sinHalfAngle, float cosHalfAngle) = MathF.SinCos(theta / 2);

        return new DualQuaternion(axisDirection * sinHalfAngle,
            cosHalfAngle,
            moment * sinHalfAngle + axisDirection * cosHalfAngle * halfDistance,
            -halfDistance * sinHalfAngle);
    }

    internal static void AxisAngleFromQuaternion(Quaternion q, out Vector3 axis, out float angle) {
        Vector3 qv = Unsafe.As<Quaternion, Vector3>(ref q);
        if (qv.LengthSquared() < 1e-6f) {
            axis = Vector3.UnitX;
            angle = 0;
            return;
        }

        float halfAngle = MathF.Atan2(qv.Length(), q.W);
        angle = 2 * halfAngle;
        axis = Vector3.Normalize(qv / MathF.Sin(halfAngle));
    }

    public static void DecomposeScrewParameters(DualQuaternion dq, out Vector3 pointOnAxis, out Vector3 axisDirection,
        out float pitch, out float theta) {
        DecomposeRotationTranslation(dq, out Quaternion rotation, out Vector3 translation);
        AxisAngleFromQuaternion(rotation, out axisDirection, out theta);
        if (rotation.IsIdentity || MathF.Abs(theta) < float.Epsilon) {
            // pure translation
            theta = translation.Length();
            if (theta < float.Epsilon) {
                axisDirection = Vector3.UnitX;
            } else {
                axisDirection = translation / theta;
            }

            pointOnAxis = Vector3.Zero;
            pitch = float.PositiveInfinity;
            return;
        }

        // has rotation component so we have a point on the axis and a pitch to figure out
        float dist = Vector3.Dot(translation, axisDirection);
        Vector3 moment = (Vector3.Cross(translation, axisDirection) +
                          (translation - axisDirection * dist) / MathF.Tan(theta / 2)) / 2;
        pointOnAxis = Vector3.Cross(axisDirection, moment);
        pitch = dist / theta;
    }

    public static DualQuaternion Sclerp(DualQuaternion dq1, DualQuaternion dq2, float amount) {
        DecomposeScrewParameters(dq2 * Conjugate(dq1), out Vector3 q, out Vector3 axis,
            out float h, out float theta);
        return dq1 * CreateFromScrewParameters(q, axis, h, theta * amount);
    }

    public static Matrix4x4 ToHomogenousTransformMatrix(DualQuaternion dq) {
        DecomposeRotationTranslation(dq, out Quaternion rot, out Vector3 trans);

        Matrix4x4 m = Matrix4x4.CreateFromQuaternion(rot);
        Unsafe.As<float, Vector3>(ref m.M41) = trans;
        return m;
    }

#if NET7_0_OR_GREATER
    static DualQuaternion IAdditiveIdentity<DualQuaternion, DualQuaternion>.AdditiveIdentity => Zero;
    static DualQuaternion IMultiplicativeIdentity<DualQuaternion, DualQuaternion>.MultiplicativeIdentity => Identity;
#endif
}