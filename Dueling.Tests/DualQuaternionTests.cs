using System.Numerics;
using FluentAssertions;

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
}