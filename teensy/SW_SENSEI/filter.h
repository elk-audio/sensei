#ifndef FILTER_H
#define FILTER_H

#include "common.h"

class Filter {
public:
    Filter();
    void setFilter(uint8_t filter_order, sensei::FilterType* filter_coeff_a, sensei::FilterType* filter_coeff_b);
    ~Filter();
    sensei::FilterType processFilter(sensei::FilterType x);

private:
    bool _enable;
    uint8_t _filter_order;
    std::vector<  sensei::FilterType> _filter_coeff_a;
    std::vector<  sensei::FilterType> _filter_coeff_b;
    std::vector<  sensei::FilterType> _buffer;
};

#endif // FILTER_H
