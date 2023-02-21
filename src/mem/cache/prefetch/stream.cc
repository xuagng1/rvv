#include "mem/cache/prefetch/stream.hh"

#include "base/random.hh"
#include "debug/HWPrefetch.hh"
#include "params/StreamPrefetcher.hh"

/* calculatePrefetch->findStream->(allocateStream)->(deallocateStream) */

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

Stream::Stream(const StreamPrefetcherParams &p)
    : Queued(p), dist_tw_l2home(p.dist_tw_l2home),
    dist_fb_l2home(p.dist_fb_l2home),
    dist_demand_l2home(p.dist_demand_l2home),
    dist_within_fb(p.dist_within_fb),
    inst_stream_cnt(p.inst_stream_cnt),
    data_stream_cnt(p.data_stream_cnt),
    prefetch_degree(p.prefetch_degree)
{

}

StreamInfo*
Stream::findStream(Addr paddr, bool iside)
{
    Addr tag = pageAddress(paddr);
    if (iside && inst_stream.find(tag) != inst_stream.end()) {
        return inst_stream[tag];
    }
    else if (!iside && data_stream.find(tag) != data_stream.end()) {
        return data_stream[tag];
    }
    return nullptr;
}



StreamInfo*
Stream::allocateStream(Addr paddr, bool iside)
{
    StreamInfo* streamInfo;
    if (iside) {
        int size = inst_stream.size();
        if (size >= inst_stream_cnt) {
            assert(size == inst_stream_cnt);
            deallocateStream(iside);
        }
        streamInfo = new StreamInfo(
            addHome(paddr, dist_demand_l2home), State::FORWARD);
        inst_stream[pageAddress(paddr)] = streamInfo;
    }
    else {
        int size = data_stream.size();
        if (size >= data_stream_cnt) {
            assert(size == data_stream_cnt);
            deallocateStream(iside);
        }
        if (blockAddress(paddr) == pageAddress(paddr)) {
            streamInfo = new StreamInfo(
                addHome(paddr, dist_demand_l2home), State::FORWARD);
        }
        else if (blockAddress(paddr) == pageTail(paddr)) {
            streamInfo = new StreamInfo(
                decHome(paddr, dist_demand_l2home), State::BACKWARD);
        }
        else {
            streamInfo = new StreamInfo(blockAddress(paddr), State::SEARCHING);
        }
        data_stream[pageAddress(paddr)] = streamInfo;
    }
    return streamInfo;
}

void
Stream::deallocateStream(bool iside)
{
    //randomly select the victim stream
    if (iside) {
        int pos = random_mt.random<unsigned>(0, inst_stream.size() - 1);
        int i = 0;
        for (auto& it : inst_stream) {
            if (pos == i) {
                delete it.second;
                inst_stream.erase(it.first);
                break;
            }
            i++;
        }
    }
    else {
        int pos = random_mt.random<unsigned>(0, data_stream.size() - 1);
        int i = 0;
        for (auto& it : data_stream) {
            if (pos == i) {
                delete it.second;
                data_stream.erase(it.first);
                break;
            }
            i++;
        }
    }
}

bool
Stream::inWindowA(State dir, Addr addr, Addr home)
{
    assert(dir == State::FORWARD || dir == State::BACKWARD);
    if (dir == State::FORWARD) {
        Addr fall_back_edge = decHome(home, dist_fb_l2home);
        return addr <= fall_back_edge;
    }
    else {
        Addr fall_back_edge = addHome(home, dist_fb_l2home);
        return addr >= fall_back_edge;
    }
}

bool
Stream::inWindowB(State dir, Addr addr, Addr home)
{
    assert(dir == State::FORWARD || dir == State::BACKWARD);
    if (dir == State::FORWARD) {
        Addr fall_back_edge = decHome(home, dist_fb_l2home);
        Addr trigger_window_edge = decHome(home, dist_tw_l2home);
        return addr > fall_back_edge && addr <= trigger_window_edge;
    }
    else {
        Addr fall_back_edge = addHome(home, dist_fb_l2home);
        Addr trigger_window_edge = addHome(home, dist_tw_l2home);
        return addr >= trigger_window_edge && addr < fall_back_edge;
    }
}

bool
Stream::inWindowC(State dir, Addr addr, Addr home)
{
    assert(dir == State::FORWARD || dir == State::BACKWARD);
    if (dir == State::FORWARD) {
        Addr trigger_window_edge = decHome(home, dist_tw_l2home);
        return addr > trigger_window_edge && addr < home;
    }
    else {
        Addr trigger_window_edge = addHome(home, dist_tw_l2home);
        return addr < trigger_window_edge && addr > home;
    }
}

bool
Stream::inWindowD(State dir, Addr addr, Addr home)
{
    assert(dir == State::FORWARD || dir == State::BACKWARD);
    if (dir == State::FORWARD) {
        return addr >= home;
    }
    else {
        return addr <= home;
    }
}

void
Stream::updateHomeline(Addr demand, StreamInfo * info, Addr dist,
                        bool updateFirst, bool prefetch,
                        std::vector<AddrPriority> &addresses)
{
    assert(info->state == State::FORWARD
            || info->state == State::BACKWARD);
    if (updateFirst) {
        info->l2_home = addHome(demand, dist * info -> state);
    }
    if (!samePage(demand, info->l2_home)
        || (pageTail(demand) == info->l2_home)
        || (pageAddress(demand) == info->l2_home)) {
        info->state = State::DONE;
        return;
    }
    if (prefetch) {
        for (int i = 0; i < prefetch_degree; i++) {
            addresses.push_back(AddrPriority(info->l2_home, 0));
            info->l2_home = addHome(info->l2_home, 1 * info -> state);
            if (!samePage(demand, info->l2_home)) {
                info->state = State::DONE;
                return;
            }
        }
    }
}

void
Stream::handleTransition(std::vector<AddrPriority> &addresses,
                        bool iside, StreamInfo * info,
                        Addr demand, bool isCacheMiss)
{
    if (iside) {
        if (info->state == State::FORWARD) {
            if (!samePage(demand, info->l2_home)
            || (pageTail(info->l2_home) == info->l2_home)) {
                DPRINTF(HWPrefetch, "ifetch: FORWARD -> DONE\n");
                info->state = State::DONE;
                return;
            }
            if (inWindowB(info->state, demand, info->l2_home)
            || inWindowD(info->state, demand, info->l2_home)) {
                DPRINTF(HWPrefetch, "ifetch: hit windowB or windowD\n");
                updateHomeline(demand, info, dist_demand_l2home,
                                true, true, addresses);
            }
            //hit the trigger window (ideal case)
            else if (inWindowC(info->state, demand, info->l2_home)) {
                DPRINTF(HWPrefetch, "ifetch: hit windowC\n");
                updateHomeline(demand, info, 1, false, true, addresses);
            }
        }
        else if (info->state == State::DONE) {
            if (isCacheMiss) {
                DPRINTF(HWPrefetch, "ifetch: DONE -> FORWARD\n");
                info->state = State::FORWARD;
                updateHomeline(demand, info, dist_demand_l2home,
                                true, false, addresses);
            }
        }
    }
    else {
        if (info->state == State::SEARCHING) {
            //New demand is located in within_init_fwd
            if (demand > info->l2_home
                && samePage(demand, addHome(demand, dist_demand_l2home))
                && demand <= addHome(info->l2_home, dist_within_fb)) {
                DPRINTF(HWPrefetch, "drequest: SEARCHING -> FORWARD\n");
                info->state = State::FORWARD;
                updateHomeline(demand, info, dist_demand_l2home,
                                true, false, addresses);
            }
            //New demand is located in within_init_bck
            else if (demand < info->l2_home
                && samePage(demand, decHome(demand, dist_demand_l2home))
                && demand >= decHome(info->l2_home, dist_within_fb)) {
                DPRINTF(HWPrefetch, "drequest: SEARCHING -> BACKWARD\n");
                info->state = State::BACKWARD;
                updateHomeline(demand, info, dist_demand_l2home,
                                true, false, addresses);
            }
        }
        else if (info->state == State::FORWARD
                || info->state == State::BACKWARD) {
            if (inWindowB(info->state, demand, info->l2_home)
            || inWindowD(info->state, demand, info->l2_home)) {
                DPRINTF(HWPrefetch, "drequest: hit windowB or windowD\n");
                updateHomeline(demand, info, dist_demand_l2home,
                                true, true, addresses);
                info->fall_back_cnt = 0;
            }
            else if (inWindowC(info->state, demand, info->l2_home)) {
                DPRINTF(HWPrefetch, "drequest: hit windowC\n");
                updateHomeline(demand, info, 1, false, true, addresses);
                info->fall_back_cnt = 0;
            }
            else {
                assert(inWindowA(info->state, demand, info->l2_home));
                DPRINTF(HWPrefetch, "drequest: hit windowA\n");
                if (info->fall_back_cnt == 1) {
                    info->state = State::SEARCHING;
                    info->fall_back_cnt = 0;
                    return;
                }
                info->fall_back_cnt += 1;
                updateHomeline(demand, info, dist_demand_l2home,
                                true, false, addresses);
            }
        }
        else {
            assert(info -> state == State::DONE);
            if (isCacheMiss) {
                DPRINTF(HWPrefetch, "drequest: DONE -> SEARCHING\n");
                info->state = State::SEARCHING;
                info->l2_home = blockAddress(demand);
            }
        }
    }
}

void
Stream::calculatePrefetch(const PrefetchInfo &pfi,
        std::vector<AddrPriority> &addresses)
{
    bool iside = pfi.isInstFetch();
    Addr addr = pfi.getAddr();
    StreamInfo* streamInfo = findStream(addr, iside);

    if (!streamInfo) {
        streamInfo = allocateStream(addr, iside);
    }
    handleTransition(addresses, iside, streamInfo, addr, pfi.isCacheMiss());
}

}
}