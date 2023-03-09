#ifndef __MEM_CACHE_PREFETCH_ASSIST_HH__
#define __MEM_CACHE_PREFETCH_ASSIST_HH__

#include <map>
#include <vector>

#include "mem/cache/prefetch/queued.hh"
#include "mem/packet.hh"

namespace gem5
{

struct AssistPrefetcherParams;

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

class Assist : public Queued
{
    public:
        Assist(const AssistPrefetcherParams &p);
        ~Assist() = default;

        void calculatePrefetch(const PrefetchInfo &pfi,
                std::vector<AddrPriority> &addresses) override {};

};

} // namespace prefetch
} // namespace gem5

#endif /* __MEM_CACHE_PREFETCH_BOP_HH__ */