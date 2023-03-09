#include "mem/cache/prefetch/assist.hh"

#include "base/random.hh"
#include "debug/HWPrefetch.hh"
#include "params/AssistPrefetcher.hh"

/* calculatePrefetch->findStream->(allocateStream)->(deallocateStream) */

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

Assist::Assist(const AssistPrefetcherParams &p)
    : Queued(p)
{

}

}
}