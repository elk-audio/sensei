#include "gtest/gtest.h"
#include "locked_queue.h"

class TestContainer
{
public:
    int a;
    float b;
};

TEST(LockedQueueTest, test_ordering_with_unique_pointers)
{
    LockedQueue<std::unique_ptr<int>> module_under_test;
    std::unique_ptr<int> a(new int);
    *a = 1;
    std::unique_ptr<int> b(new int);
    *b = 2;
    std::unique_ptr<int> c(new int);
    *c = 3;

    module_under_test.push(std::move(a));
    module_under_test.push(std::move(b));
    module_under_test.push(std::move(c));

    ASSERT_FALSE(module_under_test.empty());

    ASSERT_EQ(1, *module_under_test.pop());
    ASSERT_EQ(2, *module_under_test.pop());
    ASSERT_EQ(3, *module_under_test.pop());

    ASSERT_TRUE(module_under_test.empty());
}

TEST(LockedQueueTest, test_ordering_with_object)
{
    LockedQueue<TestContainer> module_under_test;
    TestContainer a;
    a.a = 2;
    TestContainer b;
    b.a = 4;

    module_under_test.push(b);
    module_under_test.push(std::move(a));
    ASSERT_FALSE(module_under_test.empty());

    TestContainer ret_b = module_under_test.pop();
    TestContainer ret_a = module_under_test.pop();

    ASSERT_EQ(2, ret_a.a);
    ASSERT_EQ(4, ret_b.a);

    ASSERT_TRUE(module_under_test.empty());
}