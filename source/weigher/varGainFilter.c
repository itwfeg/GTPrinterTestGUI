
#include "varGainFilter.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
void varGainFilterInit( varGainFilter *f, double value, double gain )
{
    f->value = value;

    varGainFilterSetGainMinMax( f, 0.0, 1.0 );
    varGainFilterSetGain( f, gain );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double varGainFilterAppend( varGainFilter *f, double c )
{
    // update filtered value
    f->value += f->gain * (c - f->value);

    return f->value;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double varGainFilterAppendDelta( varGainFilter *f, double d )
{
    // update filtered value
    f->value += f->gain * d;

    return f->value;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void varGainFilterSetGain( varGainFilter *f, double gain )
{
    if ( gain < f->minGain )
        gain = f->minGain;
    else if ( f->maxGain < gain )
        gain = f->maxGain;

    f->gain = gain;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void varGainFilterAdjustGain( varGainFilter *f, double amount )
{
    varGainFilterSetGain( f, f->gain * amount );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void varGainFilterSetGainMinMax( varGainFilter *f, double minGain, double maxGain )
{
    if ( minGain < 0.0 )
        minGain = 0.0;
    else if ( 1.0 < minGain )
        minGain = 1.0;

    if ( maxGain <= minGain )
        maxGain = minGain;
    else if ( 1.0 < maxGain )
        maxGain = 1.0;

    f->minGain = minGain;
    f->maxGain = maxGain;
}
