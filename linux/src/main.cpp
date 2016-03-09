#include "public_header.h"
#include "libserialport.h"

int main() {
    sp_port* port;
    sp_get_port_by_name("/dev/ttyS01", &port);

    // Do something
    return 0;
}