#ifndef EDGEDETECTION_H
#define EDGEDETECTION_H

#include <stdbool.h>

// check for c++
#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////

/// @c edgeDetect filter length
#define EDGEDETECT_LEN 8

/// @c edgeDetect filter split location for edge detection
#define EDGEDETECT_LEN_SPLIT (EDGEDETECT_LEN / 2)

/// Edge Detection.
typedef struct
{
    long z[EDGEDETECT_LEN];                         ///< Raw counts.
    unsigned long index;                            ///< Current write index.

    bool init;                                      ///< Filter is initialized.

    double velocity;                                ///< Filter velocity.

    long c;                                         ///< First raw count in edge portion of filter.

} edgeDetect;

/// Initialize Edge Detection.
/**
 * @param[in] f  filter
 */
void edgeDetectInit( edgeDetect *f );

/// Append count to filter.
/**
 * @param[in] f  filter
 * @return  @c true if edge detected, @c false otherwise
 */
bool edgeDetectAppend( edgeDetect *f, long c );


///////////////////////////////////////////////////////////////////////////////////////////////////

// check for c++
#ifdef __cplusplus
}
#endif

#endif // EDGEDETECTION_H
