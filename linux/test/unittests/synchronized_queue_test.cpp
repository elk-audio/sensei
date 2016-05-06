#include <thread>

#include "gtest/gtest.h"
#include "synchronized_queue.h"

class TestContainer
{
public:
    int a;
    float b;
};

const int ITERATIONS = 5;
std::mutex mutex;

void push_loop(SynchronizedQueue<TestContainer>* module_under_test, std::condition_variable* notifier)
{

    std::unique_lock<std::mutex> lock(mutex);
    for (int i = 0; i < ITERATIONS ; ++i)
    {
        TestContainer m;
        m.a = i;
        module_under_test->push(m);
        // wait for notifications from the main thread
        notifier->wait_for(lock, std::chrono::milliseconds(20));
    }
}
/*
 * Sets up a separate thread that pushes messages on to the queue
 * when triggered from the main thread.
 * The main thread reads the messages and verifies that they are
 * in order and all messages went through
 */
TEST(SynchronizedQueueTest, ordertest)
{
    SynchronizedQueue<TestContainer> module_under_test;
    std::condition_variable push_notifier;
    // Initial state should be empty
    ASSERT_TRUE(module_under_test.empty());

    std::thread push_thread = std::thread(push_loop, &module_under_test, &push_notifier);
    for (int i = 0; i < ITERATIONS; ++i)
    {
        // Normally the push thread shouldn't had time to push the first message yet,
        // But that is not critical since wait_for_data returns immediately if there
        // is data.
        module_under_test.wait_for_data(std::chrono::milliseconds(10));
        std::lock_guard<std::mutex> lock(mutex);
        if (module_under_test.empty() == false)
        {
            TestContainer m = module_under_test.pop();
            ASSERT_TRUE(module_under_test.empty());
            ASSERT_EQ(i, m.a);
        }
        // tell the push thread to push a new message on the queue
        push_notifier.notify_one();
    }
    // We should have got all messages by now
    ASSERT_TRUE(module_under_test.empty());
    push_thread.join();
}

