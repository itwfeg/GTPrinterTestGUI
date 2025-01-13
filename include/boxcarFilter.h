#ifndef BOXCARFILTER_H
#define BOXCARFILTER_H

// check for c++
#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////

/// @c boxcarFilter maximum length
#define BOXCARFILTER_MAX_LEN 32

/// Boxcar Filter.
/**
 * Variable length boxcar filter.
 */
typedef struct
{
    long z[BOXCARFILTER_MAX_LEN];                   ///< Raw counts.
    unsigned long index;                            ///< Current write index.

    long min;                                       ///< Min value.
    long max;                                       ///< Max value.

    unsigned long maxLength;                        ///< Filter maximum length.
    unsigned long length;                           ///< Current filter length.

    unsigned long numCounts;                        ///< Number of counts appended since reset.

} boxcarFilter;

/// Initialize Boxcar Filter.
/**
 * @param[in] f  filter
 * @param[in] maxLength  maximum length of filter
 */
void boxcarFilterInit( boxcarFilter *f, unsigned long maxLength );

/// Reset filter to zero entries.
/**
 * @param[in] f  filter
 */
void boxcarFilterReset( boxcarFilter *f );

/// Append count to filter.
/**
 * @param[in] f  filter
 * @param[in] c  count
 * @return  filtered value
 */
double boxcarFilterAppend( boxcarFilter *f, long c );

/// Retrieve filter min/max range.
/**
 * @param[in] f  filter
 * @return  range of filter min/max value
 */
unsigned long boxcarFilterRange( const boxcarFilter *f );

/// Shrink filter.
/**
 * This method removes one count (the oldest value) from the raw values.
 * @param[in] f  filter
 */
void boxcarFilterShrink( boxcarFilter *f );

/// Set maximum filter length.
/**
 * @param[in] f  filter
 * @param[in] maxLength  maximum length of filter
 */
void boxcarFilterSetMaxLength( boxcarFilter *f, unsigned long maxLength );

/// Calculate Mean Square Error (MSE).
/**
 * Calculates MSE around regression value @p r.
 * @param[in] f  filter
 * @param[in] r  regression value
 * @return  MSE value
 */
double boxcarFilterCalcMeanSquareError( const boxcarFilter *f, double r );

/// Calculate average velocity.
/**
 * @param[in] f  filter
 * @return  velocity in counts/period
 */
double boxcarFilterCalcVelocity( const boxcarFilter *f );

/// Calculate slope.
/**
 * @param[in] f  filter
 * @return  slope
 */
double boxcarFilterCalcSlope( const boxcarFilter *f );


///////////////////////////////////////////////////////////////////////////////////////////////////

// check for c++
#ifdef __cplusplus
}
#endif

#endif // BOXCARFILTER_H
