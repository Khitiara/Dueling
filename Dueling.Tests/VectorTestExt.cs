using System.Runtime.InteropServices;
using FluentAssertions.Execution;
using FluentAssertions.Primitives;

namespace Dueling.Tests;

internal static class VectorTestExt
{
    private static bool TestApprox<T>(T actual, T expected)
        where T : unmanaged {
        ReadOnlySpan<float> actualSpan =
            MemoryMarshal.Cast<T, float>(MemoryMarshal.CreateReadOnlySpan(ref actual, 1));
        ReadOnlySpan<float> expectedSpan =
            MemoryMarshal.Cast<T, float>(MemoryMarshal.CreateReadOnlySpan(ref expected, 1));
        for (int i = 0; i < actualSpan.Length; i++) {
            if (MathF.Abs(actualSpan[i] - expectedSpan[i]) >= DualQuaternionTests.Epsilon)
                return false;
        }

        return true;
    }

    public static int BeApproximatelyOneOf<T>(this ObjectAssertions assertions, params T[] options)
        where T : unmanaged {
        int idx = -1;
        Execute.Assertion.ForCondition(assertions.Subject is T test &&
                                       (idx = Array.FindIndex(options, v => TestApprox(test, v))) >= 0)
            .BecauseOf(string.Empty)
            .FailWith("Expected {context:value} to be approximately one of {0}{reason}, but found {1}.", options,
                assertions.Subject);
        return idx;
    }

    public static void BeApproximately<T>(this ObjectAssertions assertions, T v)
        where T : unmanaged {
        Execute.Assertion.ForCondition(assertions.Subject is T test && TestApprox(test, v))
            .BecauseOf(string.Empty)
            .FailWith("Expected {context:value} to be approximately {0}{reason}, but found {1}.", v,
                assertions.Subject);
    }
}