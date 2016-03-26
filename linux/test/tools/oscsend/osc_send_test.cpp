/**
 * @file osc_send_test.cpp
 * @brief Test liblo integration by sending various OSC messages on a given port
 * @author Stefano Zambon
 * @copyright MIND Music Labs AB, Stockholm
 * @date 2016-03-22
 */

#include <lo/lo_cpp.h>

// Test it by launching e.g. oscdump DESTINATION_PORT
static const char* DESTINATION_PORT = "9999";

static void send_integer(const lo::Address& address, const char* path)
{
    lo::Message m;
    m.add_int32(123456);
    address.send(path, m);
}

static void send_float(const lo::Address& address, const char* path)
{
    lo::Message m;
    m.add_float(-1.98765f);
    address.send(path, m);
}

static void send_mixed(const lo::Address& address, const char* path)
{
    lo::Message m;
    m.add_int32(101);
    m.add_float(12.34567f);
    m.add_string("pippo");
    address.send(path, m);
}


int main()
{
    lo::Address a("localhost", DESTINATION_PORT);

    // Prepare messages and send them
    send_integer(a, "/foo/bar");
    send_float(a, "/foo/floatbar");
    send_mixed(a, "/foo/morestuff");

}

