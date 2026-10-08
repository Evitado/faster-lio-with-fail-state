#ifndef FASTER_LIO_VOXEL_TABLE_H
#define FASTER_LIO_VOXEL_TABLE_H

#include <Eigen/Core>
#include <cstdint>
#include <utility>
#include <vector>

namespace faster_lio {

/**
 * Voxel storage for IVox: an open-addressing hash table from voxel key to an index into one contiguous array of
 * voxels, plus a least-recently-used list kept as indices in that array.
 *
 * It replaces unordered_map<key, list::iterator> + std::list, which needed four dependent memory accesses per hit
 * (bucket, map node, list node, voxel data); here a lookup is one probe sequence in a flat array and a hit reads one
 * voxel record. Semantics are the same: Insert/Touch make a voxel the most recent one, EvictLeastRecent drops the
 * voxel that was inserted into longest ago.
 *
 * Keys are packed into 21 bits per axis, i.e. +-1048575 voxels from the origin (+-524 km at 0.5 m).
 */
template <int dim, typename NodeType>
class VoxelTable {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    static_assert(dim >= 1 && dim <= 3, "keys are packed into 64 bits");

    VoxelTable() { Clear(); }

    void Clear() {
        entries_.clear();
        slots_.assign(kInitialSlots, Slot{});
        shift_ = 64 - Log2(kInitialSlots);
        head_ = tail_ = -1;
    }

    size_t Size() const { return entries_.size(); }

    /// index of the voxel with this key, or -1
    int Find(const KeyType &key) const {
        const uint64_t packed = Pack(key);
        const size_t mask = slots_.size() - 1;
        for (size_t s = Home(packed);; s = (s + 1) & mask) {
            const Slot &slot = slots_[s];
            if (slot.index < 0) return -1;
            if (slot.key == packed) return slot.index;
        }
    }

    NodeType &Node(int index) { return entries_[index].node; }
    const NodeType &Node(int index) const { return entries_[index].node; }

    /// adds a voxel whose key is not in the table yet, as the most recently used one; returns its index
    int Insert(const KeyType &key, NodeType node) {
        if ((entries_.size() + 1) * 2 > slots_.size()) {
            Grow();
        }
        const int index = static_cast<int>(entries_.size());
        entries_.push_back(Entry{key, std::move(node), -1, -1});
        PlaceSlot(Pack(key), index);
        LinkFront(index);
        return index;
    }

    /// marks a voxel as the most recently used one
    void Touch(int index) {
        if (index == head_) return;
        Unlink(index);
        LinkFront(index);
    }

    /// removes the least recently used voxel; moves the last voxel of the array into its place
    void EvictLeastRecent() {
        const int victim = tail_;
        if (victim < 0) return;
        EraseSlot(Pack(entries_[victim].key));
        Unlink(victim);
        const int last = static_cast<int>(entries_.size()) - 1;
        if (victim != last) {
            entries_[victim] = std::move(entries_[last]);
            Entry &moved = entries_[victim];
            (moved.prev >= 0 ? entries_[moved.prev].next : head_) = victim;
            (moved.next >= 0 ? entries_[moved.next].prev : tail_) = victim;
            slots_[FindSlot(Pack(moved.key))].index = victim;
        }
        entries_.pop_back();
    }

   private:
    struct Slot {
        uint64_t key = 0;
        int32_t index = -1;  // -1: empty
    };
    struct Entry {
        KeyType key;
        NodeType node;
        int32_t prev, next;  // LRU list, prev = more recently used
    };

    static constexpr size_t kInitialSlots = 1024;

    static int Log2(size_t n) {
        int r = 0;
        while ((size_t(1) << r) < n) ++r;
        return r;
    }

    static uint64_t Pack(const KeyType &key) {
        uint64_t packed = 0;
        for (int d = 0; d < dim; ++d) {
            packed = (packed << 21) | (uint64_t(uint32_t(key[d])) & 0x1FFFFFu);
        }
        return packed;
    }

    /// Fibonacci hashing: top bits of the product
    size_t Home(uint64_t packed) const { return size_t((packed * 0x9E3779B97F4A7C15ull) >> shift_); }

    size_t FindSlot(uint64_t packed) const {
        const size_t mask = slots_.size() - 1;
        size_t s = Home(packed);
        while (slots_[s].key != packed || slots_[s].index < 0) s = (s + 1) & mask;
        return s;
    }

    void PlaceSlot(uint64_t packed, int index) {
        const size_t mask = slots_.size() - 1;
        size_t s = Home(packed);
        while (slots_[s].index >= 0) s = (s + 1) & mask;
        slots_[s] = Slot{packed, index};
    }

    /// linear-probing delete with backward shift, so lookups never need tombstones
    void EraseSlot(uint64_t packed) {
        const size_t mask = slots_.size() - 1;
        size_t hole = FindSlot(packed);
        slots_[hole].index = -1;
        for (size_t j = (hole + 1) & mask; slots_[j].index >= 0; j = (j + 1) & mask) {
            const size_t home = Home(slots_[j].key);
            // the entry at j may fill the hole unless its home lies cyclically in (hole, j]
            const bool home_in_range = hole <= j ? (hole < home && home <= j) : (hole < home || home <= j);
            if (!home_in_range) {
                slots_[hole] = slots_[j];
                slots_[j].index = -1;
                hole = j;
            }
        }
    }

    void Grow() {
        const size_t size = slots_.size() * 2;
        slots_.assign(size, Slot{});
        shift_ = 64 - Log2(size);
        for (size_t i = 0; i < entries_.size(); ++i) {
            PlaceSlot(Pack(entries_[i].key), static_cast<int>(i));
        }
    }

    void LinkFront(int index) {
        Entry &e = entries_[index];
        e.prev = -1;
        e.next = head_;
        (head_ >= 0 ? entries_[head_].prev : tail_) = index;
        head_ = index;
    }

    void Unlink(int index) {
        Entry &e = entries_[index];
        (e.prev >= 0 ? entries_[e.prev].next : head_) = e.next;
        (e.next >= 0 ? entries_[e.next].prev : tail_) = e.prev;
    }

    std::vector<Slot> slots_;
    std::vector<Entry> entries_;
    int shift_ = 54;
    int head_ = -1, tail_ = -1;
};

}  // namespace faster_lio

#endif  // FASTER_LIO_VOXEL_TABLE_H
