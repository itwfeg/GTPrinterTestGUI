
#include "boxcarFilter.h"

#include <float.h>
#include <limits.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
void boxcarFilterInit( boxcarFilter *f, unsigned long maxLength )
{
    f->index = 0;

    boxcarFilterReset( f );
    boxcarFilterSetMaxLength( f, maxLength );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void boxcarFilterReset( boxcarFilter *f )
{
    f->length = f->numCounts = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double boxcarFilterAppend( boxcarFilter *f, long c )
{
    f->z[f->index++] = c;

    // wrap index
    if ( BOXCARFILTER_MAX_LEN <= f->index )
        f->index = 0;

    // check filter length
    if ( f->length < f->maxLength )
        ++f->length;

    // increase counts
    ++f->numCounts;

    // calculate filtered value
    const unsigned long j0 = BOXCARFILTER_MAX_LEN + f->index - f->length;

    double value = 0.0;

    f->min = INT_MAX;
    f->max = INT_MIN;

    for ( unsigned long i = f->length; i--; )
    {
        const unsigned long j = (j0 + i) % BOXCARFILTER_MAX_LEN;
        const long v = f->z[j];

        value += (double)( v );

        if ( v < f->min )
            f->min = v;

        if ( f->max < v )
            f->max = v;
    }

    return (value / f->length);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long boxcarFilterRange( const boxcarFilter *f )
{
    return (unsigned long)( f->max - f->min );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void boxcarFilterShrink( boxcarFilter *f )
{
    if ( 0 < f->length )
        --f->length;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void boxcarFilterSetMaxLength( boxcarFilter *f, unsigned long maxLength )
{
    if ( BOXCARFILTER_MAX_LEN < maxLength )
        maxLength = BOXCARFILTER_MAX_LEN;

    f->maxLength = maxLength;

    if ( f->maxLength < f->length )
        f->length = f->maxLength;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double boxcarFilterCalcMeanSquareError( const boxcarFilter *f, double r )
{
    // sum square of errors
    const unsigned long j0 = BOXCARFILTER_MAX_LEN + f->index - f->length;

    double value = 0.0;

    for ( unsigned long i = f->length; i--; )
    {
        const unsigned long j = (j0 + i) % BOXCARFILTER_MAX_LEN;
        const double delta = (double) f->z[j] - r;

        value += delta * delta;
    }

    // find mean
    return ( value / f->length );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double boxcarFilterCalcVelocity( const boxcarFilter *f )
{
    // make sure we have enough data
    if ( f->length < 2 )
        return 0.0;

    // sum all deltas
    const unsigned long j0 = BOXCARFILTER_MAX_LEN + f->index - f->length;

    double value = 0.0;

    unsigned long num = 0;
    long previous;

    for ( unsigned long i = f->length; i--; )
    {
        const unsigned long j = (j0 + i) % BOXCARFILTER_MAX_LEN;
        const long v = f->z[j];

        if ( num++ )
            value += (v - previous);

        previous = v;
    }

    // find mean
    return ( value / num );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double boxcarFilterCalcSlope( const boxcarFilter *f )
{
    // make sure we have enough data
    if ( f->length < 2 )
        return 0.0;

    // calc mean
    const unsigned long j0 = BOXCARFILTER_MAX_LEN + f->index - f->length;

    double xmean = (double) (f->length - 1.0) / 2.0;
    double ymean = 0.0;

    for ( unsigned long i = f->length; i--; )
    {
        const unsigned long j = (j0 + i) % BOXCARFILTER_MAX_LEN;

        ymean += f->z[j];
    }

    ymean /= f->length;

    // calc sums
    double dividend = 0.0;
    double divisor = 0.0;

    for ( unsigned long i = f->length; i--; )
    {
        const unsigned long j = (j0 + i) % BOXCARFILTER_MAX_LEN;

        const double xdelta = (double) i - xmean;
        const double ydelta = (double)  f->z[j] - ymean;

        dividend += xdelta * ydelta;
        divisor += xdelta * xdelta;
    }

    // calc slope
    return (dividend / divisor);
}

