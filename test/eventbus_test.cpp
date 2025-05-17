#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <vector>
#include <chrono>
#include "sensfus/utils/eventbus.h"

namespace sensfus {
namespace testing {

TEST(EventBusTest, BasicPublishSubscribe) {
  utils::EventBus bus;
  bus.AddChannel<int>("numbers");
  auto sub1 = bus.Subscribe<int>("numbers");
  auto sub2 = bus.Subscribe<int>("numbers");

  bus.Publish<int>("numbers", 42);

  auto data1 = sub1->Fetch();
  auto data2 = sub2->Fetch();

  ASSERT_TRUE(data1);
  ASSERT_TRUE(data2);
  EXPECT_EQ(*data1, 42);
  EXPECT_EQ(*data2, 42);

  // After Fetch, queues should be empty
  EXPECT_FALSE(sub1->Fetch());
  EXPECT_FALSE(sub2->Fetch());
}

TEST(EventBusTest, MultipleMessages) {
  utils::EventBus bus;
  bus.AddChannel<std::string>("strings");
  auto sub = bus.Subscribe<std::string>("strings");

  bus.Publish<std::string>("strings", "hello");
  bus.Publish<std::string>("strings", "world");

  auto d1 = sub->Fetch();
  auto d2 = sub->Fetch();

  ASSERT_TRUE(d1);
  ASSERT_TRUE(d2);
  EXPECT_EQ(*d1, "hello");
  EXPECT_EQ(*d2, "world");
  EXPECT_FALSE(sub->Fetch());
}

TEST(EventBusTest, DataIsRemovedAfterAllFetched) {
  utils::EventBus bus;
  bus.AddChannel<int>("numbers");
  auto sub1 = bus.Subscribe<int>("numbers");
  auto sub2 = bus.Subscribe<int>("numbers");

  bus.Publish<int>("numbers", 99);

  // Both fetch
  auto d1 = sub1->Fetch();
  auto d2 = sub2->Fetch();

  // After both fetch, the shared_ptr should be destroyed (refcount == 0)
  EXPECT_EQ(d1.use_count(), 2);
  EXPECT_EQ(d2.use_count(), 2);
  EXPECT_EQ(*d1, 99);
  EXPECT_EQ(*d2, 99);

  d1.reset();                    // Reset one of the shared_ptrs
  EXPECT_EQ(d2.use_count(), 1);  // Only sub2 should hold the data

  // Queues are empty
  EXPECT_FALSE(sub1->Fetch());
  EXPECT_FALSE(sub2->Fetch());
}

TEST(EventBusTest, MultithreadedPublishSubscribe) {
  utils::EventBus bus;
  bus.AddChannel<int>("numbers");
  auto sub1 = bus.Subscribe<int>("numbers");
  auto sub2 = bus.Subscribe<int>("numbers");

  std::atomic<int> received1{0};
  std::atomic<int> received2{0};

  std::thread pub([&bus]() {
    for (int i = 0; i < 100; ++i) {
      bus.Publish<int>("numbers", i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  std::thread t1([&]() {
    for (int i = 0; i < 100; ++i) {
      while (!sub1->Fetch()) std::this_thread::yield();
      ++received1;
    }
  });

  std::thread t2([&]() {
    for (int i = 0; i < 100; ++i) {
      while (!sub2->Fetch()) std::this_thread::yield();
      ++received2;
    }
  });

  pub.join();
  t1.join();
  t2.join();

  EXPECT_EQ(received1, 100);
  EXPECT_EQ(received2, 100);
}

}  // namespace testing
}  // namespace sensfus