#include "Filter.h"
//#include <arm_neon.h>

Filter::Filter()
{
    isInitialized=false;
    isEnable=false;
}

int32_t Filter::setFilter(uint8_t _filterOrder,type_filter_var* _filterCoeff_a,type_filter_var* _filterCoeff_b)
{
    filterOrder=_filterOrder;
    if (filterOrder>0)
    {
        for (int idx = 0; idx < filterOrder + 1; idx++)
        {
            filterCoeff_a.push_back(_filterCoeff_a[idx]);
            filterCoeff_b.push_back(_filterCoeff_b[idx]);
        }

        zbuffer.resize(filterOrder);

        if (DEBUG)
        {
            SerialDebug.println("------------------------------------");
            SerialDebug.println("filterCoeff");
            SerialDebug.println("------------------------------------");
            for (int idx = 0; idx < filterOrder + 1; idx++)
            {
                SerialDebug.println(String(filterCoeff_a[idx]) + " " + String(filterCoeff_b[idx]));
            }
            SerialDebug.println("------------------------------------");
            SerialDebug.println("");
        }

        isEnable=true;
    }
    isInitialized=true;
    return SENSEI_ERROR_CODE::OK;
}

Filter::~Filter()
{
    //SerialDebug.println("~Filter()");
    zbuffer.clear();
    filterCoeff_a.clear();
    filterCoeff_b.clear();
}


type_filter_var Filter::processFilter(type_filter_var x)
{
    if (isEnable)
    {
        type_filter_var y;

        y=filterCoeff_b[0]*x+zbuffer[0];

        for(int i=0;i<filterOrder-1;i++)
        {
            zbuffer[i]=zbuffer[i+1]+filterCoeff_b[i+1]*x-filterCoeff_a[i+1]*y;
        }
        zbuffer[filterOrder-1]=filterCoeff_b[filterOrder]*x-filterCoeff_a[filterOrder]*y;
        return y;
    }
    else return x;
}
