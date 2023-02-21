#ifndef __MEM_CACHE_PREFETCH_STREAM_HH__
#define __MEM_CACHE_PREFETCH_STREAM_HH__

#include <map>
#include <vector>

#include "mem/cache/prefetch/queued.hh"
#include "mem/packet.hh"

namespace gem5
{

struct StreamPrefetcherParams;

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{
enum State
{
    SEARCHING = 0,
    FORWARD = 1,
    BACKWARD = -1,
    DONE = 2
};
struct StreamInfo
{
    StreamInfo(Addr home, State s): state(s), l2_home(home), fall_back_cnt(0)
    {}
    State state;
    Addr l2_home;
    int fall_back_cnt;
};
class Stream : public Queued
{
    public:
        Stream(const StreamPrefetcherParams &p);
        ~Stream() = default;

        void calculatePrefetch(const PrefetchInfo &pfi,
                std::vector<AddrPriority> &addresses) override;

        StreamInfo* findStream(Addr paddr, bool iside);
        StreamInfo* allocateStream(Addr paddr, bool iside);
        void deallocateStream(bool iside);

        void handleTransition(std::vector<AddrPriority> &addresses,
        bool iside, StreamInfo * info, Addr demand, bool isCacheMiss);

    private:

        Addr dist_tw_l2home;
        Addr dist_fb_l2home;
        Addr dist_demand_l2home;
        Addr dist_within_fb;

        int inst_stream_cnt;
        int data_stream_cnt;

        int prefetch_degree;

        /*Addr is the page tag address */
        //#inst_stream is restricted by inst_stream_cnt
        std::map<Addr, StreamInfo*> inst_stream;
        //#data_stream is restricted by data_stream_cnt
        std::map<Addr, StreamInfo*> data_stream;

        Addr pageTail(Addr addr) {
            return blockAddress(addr | (pageBytes - 1));
        }
        Addr addHome(Addr paddr, Addr offset) {
            return blockAddress(paddr) + (offset << lBlkSize) ;
        }
        Addr decHome(Addr paddr, Addr offset) {
            return blockAddress(paddr) - (offset << lBlkSize) ;
        }

        bool inWindowA(State dir, Addr addr, Addr home);
        bool inWindowB(State dir, Addr addr, Addr home);
        bool inWindowC(State dir, Addr addr, Addr home);
        bool inWindowD(State dir, Addr addr, Addr home);
        void updateHomeline(Addr demand, StreamInfo * info, Addr dist,
                            bool updateFirst, bool prefetch,
                            std::vector<AddrPriority> &addresses);


};

} // namespace prefetch
} // namespace gem5

#endif /* __MEM_CACHE_PREFETCH_BOP_HH__ */