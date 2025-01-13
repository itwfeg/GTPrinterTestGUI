#ifndef BOXCARFILTERF_H
#define BOXCARFILTERF_H

// check for c++
#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////

/// @c boxcarFilterF maximum length
#define BOXCARFILTERF_MAX_LEN 16

/// Boxcar Filter.
/**
 * Variable length boxcar filter.
 */
typedef struct
{
    double z[BOXCARFILTERF_MAX_LEN];                ///< Raw counts.
    unsigned long index;                            ///< Current write index.

    double min;                                     ///< Min value.
    double max;                                     ///< Max value.

    unsigned long maxLength;                        ///< Filter maximum length.
    unsigned long length;                           ///< Current filter length.

    unsigned long numCounts;                        ///< Number of counts appended since reset.

} boxcarFilterF;

/// Initialize Boxcar Filter.
/**
 * @param[in] f  filter
 * @param[in] maxLength  maximum length of filter
 */
void boxcarFilterInitF( boxcarFilterF *f, unsigned long maxLength );

/// Reset filter to zero entries.
/**
 * @param[in] f  filter
 */
void boxcarFilterResetF( boxcarFilterF *f );

/// Append count to filter.
/**
 * @param[in] f  filter
 * @param[in] c  count
 * @return  filtered value
 */
double boxcarFilterAppendF( boxcarFilterF *f, double c );

/// Retrieve filter min/max range.
/**
 * @param[in] f  filter
 * @return  range of filter min/max value
 */
double boxcarFilterRangeF( const boxcarFilterF *f );

/// Set maximum filter length.
/**
 * @param[in] f  filter
 * @param[in] maxLength  maximum length of filter
 */
void boxcarFilterSetMaxLengthF( boxcarFilterF *f, unsigned long maxLength );


///////////////////////////////////////////////////////////////////////////////////////////////////

// check for c++
#ifdef __cplusplus
}
#endif

#endif // BOXCARFILTERF_H
