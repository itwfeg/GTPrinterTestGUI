#ifndef VARGAINFILTER_H
#define VARGAINFILTER_H

// check for c++
#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////

/// Variable Gain Filter
typedef struct
{
    double value;
    double gain;

    double minGain;
    double maxGain;

} varGainFilter;

/// Initialize Variable Gain Filter
/**
 * @param[in] f  filter
 * @param[in] value  initial filter value
 * @param[in] gain  initial filter gain
 */
void varGainFilterInit( varGainFilter *f, double value, double gain );

/// Append count to filter.
/**
 * @param[in] f  filter
 * @param[in] c  count
 * @return  filtered value
 */
double varGainFilterAppend( varGainFilter *f, double c );

/// Append count delta value to filter.
/**
 * @param[in] f  filter
 * @param[in] d  delta value
 * @return  filtered value
 */
double varGainFilterAppendDelta( varGainFilter *f, double d );

/// Set filter gain.
/**
 * @param[in] f  filter
 * @param[in] gain  new gain
 */
void varGainFilterSetGain( varGainFilter *f, double gain );

/// Adjust filter gain.
/**
 * @param[in] f  filter
 * @param[in] amount  amount to multiply gain by
 */
void varGainFilterAdjustGain( varGainFilter *f, double amount );

/// Set filter min/max gain values.
/**
 * @param[in] f  filter
 * @param[in] minGain  minimum gain value
 * @param[in] maxGain  maximum gain value
 */
void varGainFilterSetGainMinMax( varGainFilter *f, double minGain, double maxGain );


///////////////////////////////////////////////////////////////////////////////////////////////////

// check for c++
#ifdef __cplusplus
}
#endif

#endif // VARGAINFILTER_H
