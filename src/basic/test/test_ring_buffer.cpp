// Unit tests for basic/buffer/RingBuffer.hpp:
//  - overflow policy (oldest element dropped when full)
//  - close()/waitAndPop() semantics (drain remaining, then return false)
//  - pushes after close() are ignored
//  - waitAndPopFor timeout behavior
//  - size()/empty() bookkeeping

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "basic/buffer/RingBuffer.hpp"

namespace {

TEST(RingBufferTest, FifoOrderWithoutOverflow) {
  RingBuffer<int, 4> buf;
  EXPECT_TRUE(buf.empty());
  EXPECT_EQ(buf.size(), 0u);

  buf.push(1);
  buf.push(2);
  buf.push(3);
  EXPECT_EQ(buf.size(), 3u);
  EXPECT_FALSE(buf.empty());

  int v = 0;
  EXPECT_TRUE(buf.tryPop(v));
  EXPECT_EQ(v, 1);
  EXPECT_TRUE(buf.tryPop(v));
  EXPECT_EQ(v, 2);
  EXPECT_TRUE(buf.tryPop(v));
  EXPECT_EQ(v, 3);
  EXPECT_FALSE(buf.tryPop(v));
  EXPECT_TRUE(buf.empty());
}

TEST(RingBufferTest, OverflowDropsOldest) {
  RingBuffer<int, 4> buf;
  for (int i = 0; i < 6; ++i) buf.push(i);  // 0 and 1 must be dropped
  EXPECT_EQ(buf.size(), 4u);

  std::vector<int> popped;
  int v = 0;
  while (buf.tryPop(v)) popped.push_back(v);
  ASSERT_EQ(popped.size(), 4u);
  EXPECT_EQ(popped, (std::vector<int>{2, 3, 4, 5}));
}

TEST(RingBufferTest, OverflowByOneKeepsCapacityNewest) {
  RingBuffer<int, 3> buf;
  buf.push(10);
  buf.push(11);
  buf.push(12);
  EXPECT_EQ(buf.size(), 3u);
  buf.push(13);  // drops 10
  EXPECT_EQ(buf.size(), 3u);

  int v = 0;
  EXPECT_TRUE(buf.tryPop(v));
  EXPECT_EQ(v, 11);
  EXPECT_TRUE(buf.tryPop(v));
  EXPECT_EQ(v, 12);
  EXPECT_TRUE(buf.tryPop(v));
  EXPECT_EQ(v, 13);
  EXPECT_FALSE(buf.tryPop(v));
}

TEST(RingBufferTest, InterleavedPushPopKeepsOrder) {
  RingBuffer<int, 4> buf;
  int v = 0;
  for (int round = 0; round < 3; ++round) {
    buf.push(2 * round);
    buf.push(2 * round + 1);
    EXPECT_TRUE(buf.tryPop(v));
    EXPECT_EQ(v, 2 * round);
    EXPECT_TRUE(buf.tryPop(v));
    EXPECT_EQ(v, 2 * round + 1);
  }
  EXPECT_TRUE(buf.empty());
}

TEST(RingBufferTest, CloseAllowsDrainingThenReturnsFalse) {
  RingBuffer<int, 8> buf;
  buf.push(1);
  buf.push(2);
  buf.close();

  int v = 0;
  EXPECT_TRUE(buf.waitAndPop(v));
  EXPECT_EQ(v, 1);
  EXPECT_TRUE(buf.waitAndPop(v));
  EXPECT_EQ(v, 2);
  EXPECT_FALSE(buf.waitAndPop(v));  // drained + closed -> false, no block
}

TEST(RingBufferTest, PushAfterCloseIsIgnored) {
  RingBuffer<int, 4> buf;
  buf.close();
  buf.push(42);
  int moved = 43;
  buf.push(std::move(moved));
  EXPECT_TRUE(buf.empty());
  int v = 0;
  EXPECT_FALSE(buf.waitAndPop(v));
}

TEST(RingBufferTest, WaitAndPopBlocksUntilPush) {
  RingBuffer<int, 4> buf;
  std::atomic<bool> got{false};
  int v = 0;
  std::thread consumer([&]() {
    if (buf.waitAndPop(v)) got = true;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(got.load());
  buf.push(99);
  consumer.join();
  EXPECT_TRUE(got.load());
  EXPECT_EQ(v, 99);
}

TEST(RingBufferTest, CloseWakesBlockedConsumer) {
  RingBuffer<int, 4> buf;
  std::atomic<bool> returned{false};
  std::atomic<bool> result{true};
  std::thread consumer([&]() {
    int v = 0;
    result = buf.waitAndPop(v);
    returned = true;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(returned.load());
  buf.close();
  consumer.join();
  EXPECT_TRUE(returned.load());
  EXPECT_FALSE(result.load());  // woke up empty+closed -> false
}

TEST(RingBufferTest, WaitAndPopForTimesOutOnEmpty) {
  RingBuffer<int, 4> buf;
  int v = 0;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(buf.waitAndPopFor(v, std::chrono::milliseconds(60)));
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
            40);
}

TEST(RingBufferTest, WaitAndPopForReturnsDataImmediately) {
  RingBuffer<int, 4> buf;
  buf.push(7);
  int v = 0;
  EXPECT_TRUE(buf.waitAndPopFor(v, std::chrono::milliseconds(1000)));
  EXPECT_EQ(v, 7);
}

TEST(RingBufferTest, SizeAfterWrapAround) {
  RingBuffer<int, 4> buf;
  int v = 0;
  for (int i = 0; i < 3; ++i) buf.push(i);
  EXPECT_TRUE(buf.tryPop(v));
  EXPECT_TRUE(buf.tryPop(v));
  // head wrapped past the end relative to tail
  buf.push(3);
  buf.push(4);
  buf.push(5);
  EXPECT_EQ(buf.size(), 4u);  // full
  buf.push(6);                // drops 2
  EXPECT_EQ(buf.size(), 4u);
  std::vector<int> rest;
  while (buf.tryPop(v)) rest.push_back(v);
  EXPECT_EQ(rest, (std::vector<int>{3, 4, 5, 6}));
}

}  // namespace
