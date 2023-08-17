using System.ComponentModel.DataAnnotations;
using System.Numerics;
using AutoFixture.Xunit2;
using FluentAssertions;
using Xunit.Sdk;

namespace Dueling.Tests;

public class DualQuaternionTests
{
    internal const float Epsilon = 0.005f;

    [Fact]
    [Trait("Category", "Basic")]
    public void TestZero() {
        DualQuaternion a = new(new Quaternion(1, 2, 3, 4), new Quaternion(5, 6, 7, 8));
        (a + DualQuaternion.Zero).Should().Be(a);
    }

    [Fact]
    [Trait("Category", "Basic")]
    public void TestRightIdentity() {
        DualQuaternion a = new(new Quaternion(1, 2, 3, 4), new Quaternion(5, 6, 7, 8));
        (a * DualQuaternion.Identity).Should().Be(a);
    }

    [Fact]
    [Trait("Category", "Basic")]
    public void TestLeftIdentity() {
        DualQuaternion a = new(new Quaternion(1, 2, 3, 4), new Quaternion(5, 6, 7, 8));
        (DualQuaternion.Identity * a).Should().Be(a);
    }

    [Fact]
    [Trait("Category", "Basic")]
    public void TestIdentityTransform() {
        DualQuaternion.ToHomogenousTransformMatrix(DualQuaternion.Identity).Should().Be(Matrix4x4.Identity);
    }

    [Theory, AutoData,]
    [Trait("Category", "AxisAngle")]
    public void TestAxisAngleFromQuaternion(Vector3 axis, [Range(-MathF.PI, MathF.PI)] float angle) {
        axis = Vector3.Normalize(axis);
        DualQuaternion.AxisAngleFromQuaternion(Quaternion.Normalize(Quaternion.CreateFromAxisAngle(axis, angle)),
            out Vector3 testAxis, out float testAngle);
        int idx = testAxis.Should().BeApproximatelyOneOf(axis, -axis);
        testAngle.Should().BeApproximately(idx switch {
            0 => angle,
            1 => -angle,
            _ => throw new XunitException($"Unexpected index {idx} in axis option array"),
        }, Epsilon);
    }

    [Fact]
    public void TestAxisAngleIdentity() {
        DualQuaternion.AxisAngleFromQuaternion(Quaternion.Identity, out Vector3 axis, out float angle);
        axis.Should().Be(Vector3.UnitX);
        angle.Should().Be(0);
    }

    [Fact]
    [Trait("Category", "Basic")]
    public void TestIdentityInverts() {
        DualQuaternion.Inverse(DualQuaternion.Identity).Should().BeApproximately(DualQuaternion.Identity);
    }

    [Fact]
    [Trait("Category", "DecomposeScrew")]
    public void TestDecomposeScrewIdentity() {
        DualQuaternion.DecomposeScrewParameters(DualQuaternion.Identity, out Vector3 pt, out Vector3 axis,
            out float pitch, out float theta);
        pt.Should().BeApproximately(Vector3.Zero);
        axis.Should().BeApproximately(Vector3.UnitX);
        pitch.Should().Be(float.PositiveInfinity);
        theta.Should().BeApproximately(0f, Epsilon);
    }

    [Theory, AutoData,]
    [Trait("Category", "DecomposeScrew")]
    public void TestDecomposeScrewTranslationValues(Vector3 v) {
        DualQuaternion dq = DualQuaternion.CreateFromRotationTranslation(Quaternion.Identity, v);
        DualQuaternion.DecomposeScrewParameters(dq, out Vector3 pt, out Vector3 axis,
            out float pitch, out float theta);
        pt.Should().BeApproximately(Vector3.Zero);
        axis.Should().BeApproximately(Vector3.Normalize(v));
        pitch.Should().Be(float.PositiveInfinity);
        theta.Should().BeApproximately(v.Length(), Epsilon);
    }

    [Theory, AutoData,]
    [Trait("Category", "DecomposeScrew")]
    public void TestDecomposeScrewRotationValues(Quaternion q) {
        DualQuaternion.AxisAngleFromQuaternion(q, out Vector3 axis, out float angle);
        DualQuaternion dq = DualQuaternion.CreateFromRotationTranslation(q, Vector3.Zero);
        DualQuaternion.DecomposeScrewParameters(dq, out Vector3 pt, out Vector3 testAxis,
            out float pitch, out float theta);
        pt.Should().BeApproximately(Vector3.Zero);
        testAxis.Should().BeApproximately(axis);
        pitch.Should().BeApproximately(0, Epsilon);
        theta.Should().BeApproximately(angle, Epsilon);
    }

    [Fact]
    [Trait("Category", "CreateScrew")]
    public void TestCreateScrewParametersNothing() {
        DualQuaternion dq =
            DualQuaternion.CreateFromScrewParameters(Vector3.Zero, Vector3.Zero, float.PositiveInfinity, 0);
        dq.Should().BeApproximately(DualQuaternion.Identity);
    }

    [Theory, AutoData,]
    [Trait("Category", "CreateScrew")]
    public void TestCreateScrewParametersTranslation(Vector3 translation) {
        DualQuaternion dq =
            DualQuaternion.CreateFromScrewParameters(Vector3.Zero, Vector3.Normalize(translation),
                float.PositiveInfinity, translation.Length());
        DualQuaternion.DecomposeRotationTranslation(dq, out Quaternion rot, out Vector3 testTranslation);
        testTranslation.Should().BeApproximately(translation);
        rot.Should().BeApproximately(Quaternion.Identity);
    }

    [Theory, AutoData,]
    [Trait("Category", "CreateScrew")]
    public void TestCreateScrewParametersRotation(Quaternion rotation) {
        rotation = Quaternion.Normalize(rotation);
        DualQuaternion.AxisAngleFromQuaternion(rotation, out Vector3 axis, out float angle);
        DualQuaternion dq =
            DualQuaternion.CreateFromScrewParameters(Vector3.Zero, axis,
                0, angle);
        DualQuaternion.DecomposeRotationTranslation(dq, out Quaternion rot, out Vector3 testTranslation);
        testTranslation.Should().BeApproximately(Vector3.Zero);
        rot.Should().BeApproximately(rotation);
    }

    [Fact]
    [Trait("Category", "Basic")]
    public void TestNormalizeIdentity() {
        DualQuaternion.Normalize(DualQuaternion.Identity).Should().BeApproximately(DualQuaternion.Identity);
    }

    [Theory, AutoData,]
    [Trait("Category", "Basic")]
    public void TestNormalizeLength(DualQuaternion dq) {
        DualQuaternion.Normalize(dq).Length().Should().BeApproximately(1f, Epsilon);
    }

    [Theory, AutoData,]
    [Trait("Category", "Sclerp")]
    public void TestSclerpSame(DualQuaternion dq, [Range(0.1f, 0.9f)] float t) {
        dq = DualQuaternion.Normalize(dq);
        DualQuaternion.Sclerp(dq, dq, t).Should().BeApproximately(dq);
    }
}