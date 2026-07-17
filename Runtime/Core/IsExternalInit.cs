// Polyfill required by C# 9 records / init-only setters on Unity's .NET profile.
// Internal, so it never conflicts with other assemblies' polyfills.
namespace System.Runtime.CompilerServices {
    internal static class IsExternalInit { }
}
