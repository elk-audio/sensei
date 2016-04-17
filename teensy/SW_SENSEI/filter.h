#ifndef FILTER_H
#define FILTER_H

#include "common.h"

class Filter {
public:

    Filter();
    int32_t set_filter(uint8_t filter_order, FilterType* filter_coeff_a, FilterType* filter_coeff_b);
    ~Filter();
    FilterType process_filter(FilterType x);

private:
    bool enable;
    uint8_t _filter_order;
    vector<FilterType> _filter_coeff_a;
    vector<FilterType> _filter_coeff_b;
    vector<FilterType> _buffer;
};

#endif // FILTER_H
