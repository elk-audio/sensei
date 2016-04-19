#include "filter.h"
#include "common.h"

Filter::Filter()
{
    _enable=false;
}

void Filter::setFilter(uint8_t filter_order,FilterType* filter_coeff_a,FilterType* filter_coeff_b)
{
    _filter_order=filter_order;
    if ((_filter_order>0) && (_filter_order <= MAX_FILTER_ORDER))
    {
        for (int i = 0; i < _filter_order + 1; i++)
        {
            _filter_coeff_a.push_back(filter_coeff_a[i]);
            _filter_coeff_b.push_back(filter_coeff_b[i]);
        }

        _buffer.resize(_filter_order);

        if (DEBUG)
        {
            SerialDebug.println("------------------------------------");
            SerialDebug.println("filterCoeff");
            SerialDebug.println("------------------------------------");
            for (int i = 0; i < _filter_order + 1; i++)
            {
                SerialDebug.println(String(_filter_coeff_b[i]) + " " + String(_filter_coeff_b[i]));
            }
            SerialDebug.println("------------------------------------");
            SerialDebug.println("");
        }

        _enable=true;
    }
}

Filter::~Filter()
{
    //SerialDebug.println("~Filter()");
    _buffer.clear();
    _filter_coeff_a.clear();
    _filter_coeff_b.clear();
}

FilterType Filter::processFilter(FilterType x)
{
    if (_enable)
    {
        FilterType y;

        y=_filter_coeff_b[0]*x+_buffer[0];

        for(int i=0;i<_filter_order-1;i++)
        {
            _buffer[i]=_buffer[i+1]+_filter_coeff_b[i+1]*x-_filter_coeff_a[i+1]*y;
        }
        _buffer[_filter_order-1]=_filter_coeff_b[_filter_order]*x-_filter_coeff_a[_filter_order]*y;
        return y;
    }
    else return x;
}
