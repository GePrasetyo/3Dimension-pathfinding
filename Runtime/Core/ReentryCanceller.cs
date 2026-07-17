using System;
using System.Threading;
using System.Threading.Tasks;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Serializes re-entrant async operations: entering cancels the previous holder
    /// and waits for it to exit before handing out a fresh token.
    /// </summary>
    public class ReentryCanceller : IDisposable {
        private const int reentryDelay = 10;

        public CancellationToken token => canceller?.Token ?? default;

        private CancellationTokenSource canceller;

        public async Task<CancellationToken> Enter(CancellationToken cancel) {
            canceller?.Cancel();
            while(canceller != null) {
                await Task.Delay(reentryDelay);
            }
            canceller = CancellationTokenSource.CreateLinkedTokenSource(cancel);
            return canceller.Token;
        }

        public void Exit() {
            canceller?.Dispose();
            canceller = null;
        }

        public void Dispose() {
            canceller?.Cancel();
            canceller?.Dispose();
            canceller = null;
        }
    }
}
