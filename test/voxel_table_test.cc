#include "ivox3d/voxel_table.h"

#include <gtest/gtest.h>

#include <list>
#include <map>
#include <random>
#include <tuple>

namespace faster_lio {
namespace {

using Table = VoxelTable<3, int>;
using Key = Table::KeyType;

Key K(int x, int y, int z) { return Key(x, y, z); }

std::tuple<int, int, int> Tuple(const Key &k) { return {k.x(), k.y(), k.z()}; }

TEST(VoxelTable, EmptyTable) {
    const Table table;
    EXPECT_EQ(table.Size(), 0u);
    EXPECT_EQ(table.Find(K(0, 0, 0)), -1);
}

TEST(VoxelTable, InsertThenFind) {
    Table table;
    const int a = table.Insert(K(1, 2, 3), 10);
    const int b = table.Insert(K(-1, -2, -3), 20);
    EXPECT_EQ(table.Size(), 2u);
    ASSERT_EQ(table.Find(K(1, 2, 3)), a);
    ASSERT_EQ(table.Find(K(-1, -2, -3)), b);
    EXPECT_EQ(table.Node(a), 10);
    EXPECT_EQ(table.Node(b), 20);
    EXPECT_EQ(table.Find(K(3, 2, 1)), -1);
}

TEST(VoxelTable, NegativeAndLargeKeysStayDistinct) {
    constexpr int kMax = (1 << 20) - 1;  // largest magnitude that fits the 21 bit packing
    const Key keys[] = {K(0, 0, 0),     K(-1, 0, 0),         K(1, 0, 0),           K(0, -1, 0),
                        K(0, 0, -1),    K(-1, -1, -1),       K(kMax, kMax, kMax),  K(-kMax, -kMax, -kMax),
                        K(kMax, 0, -kMax), K(-kMax - 1, 0, 0), K(0, kMax, 0),        K(0, 0, -kMax)};
    Table table;
    for (int i = 0; i < int(std::size(keys)); ++i) {
        table.Insert(keys[i], i);
    }
    for (int i = 0; i < int(std::size(keys)); ++i) {
        const int index = table.Find(keys[i]);
        ASSERT_GE(index, 0) << keys[i].transpose();
        EXPECT_EQ(table.Node(index), i) << keys[i].transpose();
    }
}

TEST(VoxelTable, GrowKeepsEveryVoxel) {
    Table table;
    std::vector<Key> keys;
    for (int x = -10; x < 10; ++x) {
        for (int y = -10; y < 10; ++y) {
            for (int z = -7; z < 7; ++z) {
                keys.push_back(K(x, y, z));
            }
        }
    }
    ASSERT_GT(keys.size(), 5000u);  // several times the initial 1024 slots
    for (int i = 0; i < int(keys.size()); ++i) {
        EXPECT_EQ(table.Insert(keys[i], i), i);
    }
    EXPECT_EQ(table.Size(), keys.size());
    for (int i = 0; i < int(keys.size()); ++i) {
        ASSERT_EQ(table.Find(keys[i]), i);
        EXPECT_EQ(table.Node(i), i);
    }
    EXPECT_EQ(table.Find(K(100, 100, 100)), -1);
}

TEST(VoxelTable, EvictsLeastRecentlyInserted) {
    Table table;
    table.Insert(K(1, 0, 0), 1);
    table.Insert(K(2, 0, 0), 2);
    table.Insert(K(3, 0, 0), 3);
    table.EvictLeastRecent();
    EXPECT_EQ(table.Size(), 2u);
    EXPECT_EQ(table.Find(K(1, 0, 0)), -1);
    EXPECT_GE(table.Find(K(2, 0, 0)), 0);
    EXPECT_GE(table.Find(K(3, 0, 0)), 0);
}

TEST(VoxelTable, TouchMakesVoxelMostRecent) {
    Table table;
    table.Insert(K(1, 0, 0), 1);
    table.Insert(K(2, 0, 0), 2);
    table.Insert(K(3, 0, 0), 3);
    table.Touch(table.Find(K(1, 0, 0)));
    table.EvictLeastRecent();
    EXPECT_GE(table.Find(K(1, 0, 0)), 0);
    EXPECT_EQ(table.Find(K(2, 0, 0)), -1);
    table.EvictLeastRecent();
    EXPECT_GE(table.Find(K(1, 0, 0)), 0);
    EXPECT_EQ(table.Find(K(3, 0, 0)), -1);
    table.EvictLeastRecent();
    EXPECT_EQ(table.Size(), 0u);
}

TEST(VoxelTable, TouchOfMostRecentChangesNothing) {
    Table table;
    table.Insert(K(1, 0, 0), 1);
    table.Insert(K(2, 0, 0), 2);
    table.Touch(table.Find(K(2, 0, 0)));
    table.EvictLeastRecent();
    EXPECT_EQ(table.Find(K(1, 0, 0)), -1);
    EXPECT_GE(table.Find(K(2, 0, 0)), 0);
}

TEST(VoxelTable, EvictMovesLastVoxelIntoTheHole) {
    Table table;
    const int first = table.Insert(K(1, 0, 0), 1);
    table.Insert(K(2, 0, 0), 2);
    table.Insert(K(3, 0, 0), 3);
    table.EvictLeastRecent();  // removes index 0, the voxel at index 2 takes its place
    const int moved = table.Find(K(3, 0, 0));
    EXPECT_EQ(moved, first);
    EXPECT_EQ(table.Node(moved), 3);
    // the moved voxel keeps its place in the recency order: (2, 0, 0) is still the oldest
    table.EvictLeastRecent();
    EXPECT_EQ(table.Find(K(2, 0, 0)), -1);
    EXPECT_EQ(table.Node(table.Find(K(3, 0, 0))), 3);
}

TEST(VoxelTable, EvictOnEmptyTableIsANoOp) {
    Table table;
    table.EvictLeastRecent();
    EXPECT_EQ(table.Size(), 0u);
    table.Insert(K(1, 1, 1), 7);
    table.EvictLeastRecent();
    table.EvictLeastRecent();
    EXPECT_EQ(table.Size(), 0u);
    // still usable afterwards
    const int index = table.Insert(K(1, 1, 1), 8);
    EXPECT_EQ(table.Find(K(1, 1, 1)), index);
    EXPECT_EQ(table.Node(index), 8);
}

TEST(VoxelTable, ClearResets) {
    Table table;
    for (int i = 0; i < 2000; ++i) {
        table.Insert(K(i, 0, 0), i);
    }
    table.Clear();
    EXPECT_EQ(table.Size(), 0u);
    EXPECT_EQ(table.Find(K(5, 0, 0)), -1);
    table.EvictLeastRecent();
    const int index = table.Insert(K(5, 0, 0), 55);
    EXPECT_EQ(table.Find(K(5, 0, 0)), index);
    EXPECT_EQ(table.Node(index), 55);
}

/// random inserts, touches and evictions against std::map + std::list; keys from a small cube so probe sequences
/// collide and deletions shift entries back
TEST(VoxelTable, MatchesReferenceModel) {
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> coord(-8, 7);
    std::uniform_real_distribution<double> op(0, 1);

    Table table;
    std::map<std::tuple<int, int, int>, int> nodes;  // key -> node
    std::list<Key> lru;                               // front: most recent
    int next_node = 0;
    size_t max_size = 0;

    const auto check_all = [&] {
        ASSERT_EQ(table.Size(), nodes.size());
        for (const auto &[key, node] : nodes) {
            const int index = table.Find(K(std::get<0>(key), std::get<1>(key), std::get<2>(key)));
            ASSERT_GE(index, 0);
            ASSERT_EQ(table.Node(index), node);
        }
    };

    for (int step = 0; step < 20000; ++step) {
        const double r = op(rng);
        if (r < 0.45) {
            const Key key = K(coord(rng), coord(rng), coord(rng));
            const int index = table.Find(key);
            if (nodes.count(Tuple(key))) {
                ASSERT_GE(index, 0);
                table.Touch(index);
                lru.remove(key);
                lru.push_front(key);
            } else {
                ASSERT_EQ(index, -1);
                const int added = table.Insert(key, next_node);
                ASSERT_EQ(table.Find(key), added);
                nodes[Tuple(key)] = next_node++;
                lru.push_front(key);
            }
        } else if (r < 0.7 && !lru.empty()) {
            auto it = lru.begin();
            std::advance(it, std::uniform_int_distribution<size_t>(0, lru.size() - 1)(rng));
            const Key key = *it;
            table.Touch(table.Find(key));
            lru.erase(it);
            lru.push_front(key);
        } else {
            table.EvictLeastRecent();
            if (!lru.empty()) {
                const Key victim = lru.back();
                lru.pop_back();
                nodes.erase(Tuple(victim));
                ASSERT_EQ(table.Find(victim), -1) << "step " << step;
            }
        }
        max_size = std::max(max_size, nodes.size());
        if (step % 50 == 0) {
            check_all();
            if (HasFatalFailure()) {
                FAIL() << "step " << step;
            }
        }
    }
    check_all();
    EXPECT_GT(max_size, 600u);  // the table grew past its initial 1024 slots at least once

    // drain: the remaining voxels leave in exactly the reference order
    while (!lru.empty()) {
        table.EvictLeastRecent();
        ASSERT_EQ(table.Find(lru.back()), -1);
        lru.pop_back();
        ASSERT_EQ(table.Size(), lru.size());
    }
}

}  // namespace
}  // namespace faster_lio

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
