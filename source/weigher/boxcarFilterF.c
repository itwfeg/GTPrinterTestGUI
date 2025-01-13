
#include "boxcarFilterF.h"

#include <float.h>
#include <limits.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
void boxcarFilterInitF( boxcarFilterF *f, unsigned long maxLength )
{
    f->index = 0;

    boxcarFilterResetF( f );
    boxcarFilterSetMaxLengthF( f, maxLength );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void boxcarFilterResetF( boxcarFilterF *f )
{
    f->length = f->numCounts = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double boxcarFilterAppendF( boxcarFilterF *f, double c )
{
    f->z[f->index++] = c;

    // wrap index
    if ( BOXCARFILTERF_MAX_LEN <= f->index )
        f->index = 0;

    // check filter length
    if ( f->length < f->maxLength )
        ++f->length;

    // increase counts
    ++f->numCounts;

    // calculate filtered value
    const unsigned long j0 = BOXCARFILTERF_MAX_LEN + f->index - f->length;

    double value = 0.0;

    f->min = DBL_MAX;
    f->max = DBL_MIN;

    for ( unsigned long i = f->length; i--; )
    {
        const unsigned long j = (j0 + i) % BOXCARFILTERF_MAX_LEN;
        const double v = f->z[j];

        value += v;

        if ( v < f->min )
            f->min = v;

        if ( f->max < v )
            f->max = v;
    }

    return (value / f->length);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double boxcarFilterRangeF( const boxcarFilterF *f )
{
    return (f->max - f->min);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void boxcarFilterSetMaxLengthF( boxcarFilterF *f, unsigned long maxLength )
{
    if ( BOXCARFILTERF_MAX_LEN < maxLength )
        maxLength = BOXCARFILTERF_MAX_LEN;

    f->maxLength = maxLength;

    if ( f->maxLength < f->length )
        f->length = f->maxLength;
}
