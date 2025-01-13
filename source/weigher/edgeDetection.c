
#include "edgeDetection.h"

#include <limits.h>
#include <stdlib.h>

#define STDDEV 10                                   // load cell standard deviation (taken from an hour of measurements)
#define STDDEV_VAR (STDDEV * STDDEV)                // load cell standard deviation variance

///////////////////////////////////////////////////////////////////////////////////////////////////
void edgeDetectInit( edgeDetect *f )
{
    f->index = 0;
    f->init = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool edgeDetectAppend( edgeDetect *f, long c )
{
    f->z[f->index++] = c;

    // wrap index
    if ( EDGEDETECT_LEN <= f->index )
    {
        f->index = 0;
        f->init = true;
    }

    // check filter initialized
    if ( !f->init )
        return false;

    // calculate filtered values
    long long value_lower = 0;
    long lower_min = INT_MAX;
    long lower_max = INT_MIN;

    long long value_upper = 0;
    long upper_min = INT_MAX;
    long upper_max = INT_MIN;

    for ( unsigned long i = 0; i < EDGEDETECT_LEN; ++i )
    {
        const unsigned long j = (f->index + i) % EDGEDETECT_LEN;

        const long v = f->z[j];

        // split values into before edge and after edge
        // detect edge based on characteristics of each set
        if ( i < EDGEDETECT_LEN_SPLIT )
        {
            value_lower += v;

            if ( v < lower_min )
                lower_min = v;

            if ( lower_max < v )
                lower_max = v;
        }
        else
        {
            value_upper += v;

            if ( v < upper_min )
                upper_min = v;

            if ( upper_max < v )
                upper_max = v;

            if ( EDGEDETECT_LEN_SPLIT == i )
                f->c = v;
        }
    }

    value_lower /= EDGEDETECT_LEN_SPLIT;
    value_upper /= (EDGEDETECT_LEN - EDGEDETECT_LEN_SPLIT);

    // calculate velocity
    f->velocity = (value_upper - value_lower) / (EDGEDETECT_LEN - EDGEDETECT_LEN_SPLIT);

    // detect edge
    // counts changed more than deviation variance
    if (( STDDEV_VAR <= labs( (long)( value_upper - value_lower ) ) ) && (( lower_max < upper_min ) || ( upper_max < lower_min )))
        return true;

    return false;
}
