
#include "firFilter.h"

#include <limits.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
void firFilterInit( firFilter *f, unsigned long maxLength, const double *coeff )
{
    if ( FIRFILTER_MAX_LEN < maxLength )
        maxLength = FIRFILTER_MAX_LEN;

    f->index = f->length = 0;
    f->maxLength = maxLength;

    firFilterSetCoefficients( f, coeff );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double firFilterAppend( firFilter *f, long c )
{
    f->z[f->index++] = c;

    // wrap index
    if ( f->maxLength <= f->index )
        f->index = 0;

    // check filter length
    if ( f->length < f->maxLength )
        ++f->length;

    // calculate filtered value
    double value = 0.0;

    f->min = INT_MAX;
    f->max = INT_MIN;

    for ( unsigned long i = 0; i < f->length; ++i )
    {
        const unsigned long j = (f->index + i) % f->maxLength;
        const long v = f->z[j];

        value += (double)( v ) * f->coeff[i];

        if ( v < f->min )
            f->min = v;

        if ( f->max < v )
            f->max = v;
    }

    return value;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long firFilterRange( const firFilter *f )
{
    return (unsigned long)( f->max - f->min );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void firFilterSetCoefficients( firFilter *f, const double *coeff )
{
    for ( unsigned long i = f->maxLength; i--; )
        f->coeff[i] = coeff[i];
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool firFilterReady( const firFilter *f )
{
    return (f->maxLength == f->length);
}
