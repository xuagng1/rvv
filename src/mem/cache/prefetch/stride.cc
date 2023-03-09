/*
 * Copyright (c) 2018 Inria
 * Copyright (c) 2012-2013, 2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * Stride Prefetcher template instantiations.
 */

#include "mem/cache/prefetch/stride.hh"

#include <cassert>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "params/StridePrefetcher.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

Stride::StrideEntry::StrideEntry(int init_confidence)
  : TaggedEntry(), confidence(init_confidence)
{
    invalidate();
}

void
Stride::StrideEntry::invalidate()
{
    TaggedEntry::invalidate();
    lastAddr = 0;
    stride = 0;
    confidence = 0;
}

Stride::Stride(const StridePrefetcherParams &p)
  : Queued(p),
    initConfidence(p.initial_confidence),
    threshConf(p.confidence_threshold),
    useRequestorId(p.use_requestor_id),
    maxDegree(p.degree),
    pcTableInfo(p.table_assoc, p.table_entries, p.table_indexing_policy,
        p.table_replacement_policy),
    filled(0),
    adjustInterval(p.adjust_interval),
    currentDegree(2)
{
}

Stride::PCTable*
Stride::findTable(int context)
{
    // Check if table for given context exists
    auto it = pcTables.find(context);
    if (it != pcTables.end())
        return &it->second;

    // If table does not exist yet, create one
    return allocateNewContext(context);
}

Stride::PCTable*
Stride::allocateNewContext(int context)
{
    // Create new table
    auto insertion_result = pcTables.insert(std::make_pair(context,
        PCTable(pcTableInfo.assoc, pcTableInfo.numEntries,
        pcTableInfo.indexingPolicy, pcTableInfo.replacementPolicy,
        StrideEntry(initConfidence))));

    DPRINTF(HWPrefetch, "Adding context %i with stride entries\n", context);

    // Get iterator to new pc table, and then return a pointer to the new table
    return &(insertion_result.first->second);
}

void
Stride::calculatePrefetch(const PrefetchInfo &pfi,
                                    std::vector<AddrPriority> &addresses)
{
    if (!pfi.hasPC()) {
        DPRINTF(HWPrefetch, "Ignoring request with no PC.\n");
        return;
    }

    // Get required packet info
    Addr pf_addr = pfi.getAddr();
    Addr line_addr = pf_addr >> lBlkSize;
    Addr pc = pfi.getPC();
    bool is_secure = pfi.isSecure();
    RequestorID requestor_id = useRequestorId ? pfi.getRequestorId() : 0;

    

    // Get corresponding pc table
    PCTable* pcTable = findTable(requestor_id);

    // Search for entry in the pc table
    StrideEntry *entry = pcTable->findEntry(pc, is_secure);

    if (entry != nullptr) {
        pcTable->accessEntry(entry);

        // Hit in table
        int new_stride = line_addr - entry->lastAddr;
        bool stride_match = (new_stride == entry->stride);

        // Adjust confidence for stride entry
        if (stride_match && new_stride != 0) {
            if (entry->confidence < threshConf) {
                entry->confidence++;
            }
        } else {
            if (entry->confidence == 0) {
                entry->stride = new_stride;
            }
            else {
                entry->confidence--;
            }
        }

        DPRINTF(HWPrefetch, "Hit: PC %x pkt_addr %x (%s) stride %d (%s), "
                "conf %d\n", pc, pf_addr, is_secure ? "s" : "ns",
                new_stride, stride_match ? "match" : "change",
                (int)entry->confidence);

        entry->lastAddr = line_addr;

        // Abort prefetch generation if below confidence threshold
        if (entry->confidence < threshConf) {
            return;
        }

        // Generate up to degree prefetches
        for (int d = 1; d <= currentDegree; d++) {
            // Round strides up to atleast 1 cacheline
            int prefetch_stride = new_stride;

            Addr new_addr = (line_addr + prefetch_stride * d) << lBlkSize;
            if (samePage(pf_addr, new_addr))
                addresses.push_back(AddrPriority(new_addr, 0));
        }
    } else {
        // Miss in table
        DPRINTF(HWPrefetch, "Miss: PC %x pkt_addr %x (%s)\n", pc, pf_addr,
                is_secure ? "s" : "ns");

        StrideEntry* entry = pcTable->findVictim(pc);

        // Insert new entry's data
        entry->lastAddr = line_addr;
        pcTable->insertEntry(pc, is_secure, entry);
    }
}

void
Stride::notifyFill(const PacketPtr& pkt)
{
    // Only insert into the RR right way if it's the pkt is a HWP
    if (!pkt->cmd.isHWPrefetch()) return;

    filled++;
    if (filled >= adjustInterval) {
        double accuracy = (double) usefulPrefetches / (double) issuedPrefetches;
        if (accuracy > 0.5 && currentDegree < maxDegree) {
            currentDegree++;
        }
        else if (accuracy < 0.2 && currentDegree > 1) {
            currentDegree--;
        }
        filled = 0;
    }
}



uint32_t
StridePrefetcherHashedSetAssociative::extractSet(const Addr pc) const
{
    const Addr hash1 = pc >> 1;
    const Addr hash2 = hash1 >> tagShift;
    return (hash1 ^ hash2) & setMask;
}

Addr
StridePrefetcherHashedSetAssociative::extractTag(const Addr addr) const
{
    return addr;
}

} // namespace prefetch
} // namespace gem5
