#ifndef FIRFILTER_H
#define FIRFILTER_H

#include <stdbool.h>

// check for c++
#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////

/// @c firFilter maximum length
#define FIRFILTER_MAX_LEN 32

/// FIR Filter.
/**
 * The Finite Infinite Response filter applies different weights to each count.
 */
typedef struct
{
    long z[FIRFILTER_MAX_LEN];                      ///< Raw counts.
    unsigned long index;                            ///< Current write index.

    long min;                                       ///< Min value.
    long max;                                       ///< Max value.

    unsigned long maxLength;                        ///< Filter maximum length.
    unsigned long length;                           ///< Current filter length.

    double coeff[FIRFILTER_MAX_LEN];                ///< Filter coefficients.

} firFilter;

/// Initialize FIR Filter.
/**
 * @param[in] f  filter
 * @param[in] maxLength  maximum length of filter
 * @param[in] coeff  filter coefficients (oldest entry at @c 0 to newest entry at @c maxLength-1)
 */
void firFilterInit( firFilter *f, unsigned long maxLength, const double *coeff );

/// Append count to filter.
/**
 * @param[in] f  filter
 * @param[in] c  count
 * @return  filtered value
 */
double firFilterAppend( firFilter *f, long c );

/// Retrieve filter min/max range.
/**
 * @param[in] f  filter
 * @return  range of filter min/max value
 */
unsigned long firFilterRange( const firFilter *f );

/// Set filter cefficients.
/**
 * @param[in] f  filter
 * @param[in] coeff  filter coefficients (oldest entry at @c 0 to newest entry at @c maxLength-1)
 */
void firFilterSetCoefficients( firFilter *f, const double *coeff );

/// Retrieve if filter is ready (initialized).
/**
 * @param[in] f  filter
 * @return  @c true if filter has enough data and is ready, @c false otherwise
 */
bool firFilterReady( const firFilter *f );


///////////////////////////////////////////////////////////////////////////////////////////////////

// check for c++
#ifdef __cplusplus
}
#endif

#endif // FIRFILTER_H
