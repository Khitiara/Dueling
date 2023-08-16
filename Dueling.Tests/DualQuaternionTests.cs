using System.ComponentModel.DataAnnotations;
using System.Numerics;
using AutoFixture.Xunit2;
using FluentAssertions;
using Xunit.Sdk;

namespace Dueling.Tests;

public class DualQuaternionTests
{
    [Fact]
    public void TestZero() {
        DualQuaternion a = new(new Quaternion(1, 2, 3, 4), new Quaternion(5, 6, 7, 8));
        (a + DualQuaternion.Zero).Should().Be(a);
    }

    [Fact]
    public void TestRightIdentity() {
        DualQuaternion a = new(new Quaternion(1, 2, 3, 4), new Quaternion(5, 6, 7, 8));
        (a * DualQuaternion.Identity).Should().Be(a);
    }

    [Fact]
    public void TestLeftIdentity() {
        DualQuaternion a = new(new Quaternion(1, 2, 3, 4), new Quaternion(5, 6, 7, 8));
        (DualQuaternion.Identity * a).Should().Be(a);
    }

    [Fact]
    public void TestIdentityTransform() {
        DualQuaternion.ToHomogenousTransformMatrix(DualQuaternion.Identity).Should().Be(Matrix4x4.Identity);
    }

    [Theory, AutoData,]
    public void TestAxisAngleFromQuaternion(Vector3 axis, [Range(-MathF.PI, MathF.PI)] float angle) {
        axis = Vector3.Normalize(axis);
        DualQuaternion.AxisAngleFromQuaternion(Quaternion.Normalize(Quaternion.CreateFromAxisAngle(axis, angle)),
            out Vector3 testAxis, out float testAngle);
        int idx = testAxis.Should().BeApproximatelyOneOf(axis, -axis);
        testAngle.Should().BeApproximately(idx switch {
            0 => angle,
            1 => -angle,
            _ => throw new XunitException($"Unexpected index {idx} in axis option array"),
        }, 0.005f);
    }

    [Fact]
    public void TestAxisAngleIdentity() {
        DualQuaternion.AxisAngleFromQuaternion(Quaternion.Identity, out Vector3 axis, out float angle);
        axis.Should().Be(Vector3.UnitX);
        angle.Should().Be(0);
    }

    [Fact]
    public void TestIdentityInverts() {
        DualQuaternion.Inverse(DualQuaternion.Identity).Should().Be(DualQuaternion.Identity);
    }
}