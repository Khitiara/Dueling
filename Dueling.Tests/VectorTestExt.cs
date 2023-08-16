using System.Numerics;
using FluentAssertions.Execution;
using FluentAssertions.Primitives;

namespace Dueling.Tests;

internal static class VectorTestExt
{
    public static int BeApproximatelyOneOf(this ObjectAssertions assertions, params Vector3[] options) {
        int idx;
        Execute.Assertion.ForCondition((idx = Array.FindIndex(options, v => assertions.Subject is Vector3 test &&
                                                        MathF.Abs(test.X - v.X) < 0.005f &&
                                                        MathF.Abs(test.Y - v.Y) < 0.005f &&
                                                        MathF.Abs(test.Z - v.Z) < 0.005f)) >= 0)
            .BecauseOf(string.Empty)
            .FailWith("Expected {context:value} to be approximately one of {0}{reason}, but found {1}.", options,
                assertions.Subject);
        return idx;
    }
}