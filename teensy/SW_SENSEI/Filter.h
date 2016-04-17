#ifndef FILTER_H
#define FILTER_H

#include "common.h"

class Filter {
public:

    Filter();
    int32_t setFilter(uint8_t _filterOrder, type_filter_var* _filterCoeff_a, type_filter_var* filterCoeff_b);
    ~Filter();
    type_filter_var processFilter(type_filter_var x);

private:
    bool isEnable;
    uint8_t filterOrder;
    vector<type_filter_var> filterCoeff_a;
    vector<type_filter_var> filterCoeff_b;
    vector<type_filter_var> zbuffer;
};

#endif
