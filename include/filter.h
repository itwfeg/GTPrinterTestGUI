/****************************************************************
*                                                               *
*  Module Name            Program Name         Pgm Asm #        *
*  filter.h                  NEEP                               *
*                                                               *
*===============================================================*
*                                                               *
*  Copyright (c) 1991, by                                       * 
*  Hobart Corporation                                           * 
*  401 W. Market St.                                            *
*  Troy, Ohio 45374                                             *
*  (513) 332-3000                                               *
*                                                               *
*  Synopsis:                                                    *
*   None                                                        *
*                                                               *
*  Descriptions:                                                *
*   This file contains the required include information         *
*   necessary to use the analog to digital filtering and        *
*   adjusting functions found in the library file (filter.c).   *
*                                                               *
*  Assumptions:                                                 *
*   None                                                        *
*                                                               *
*  Warnings:                                                    *
*   None                                                        *
*                                                               *
*  Implementation:                                              *
*   None                                                        *
*                                                               *
*  Author(s):                             Creation Date:        *
*   Mark P. Strauser                        4/30/91             *
*  ------------------------------------------------------------ *
*  Revision Record:                                             *
*  ------------------------------------------------------------ *
*  Rev #.#   Date      S/W Eng.         Description of Change   *
*                                                               *
*****************************************************************/
#ifndef FILTER_H 
#define FILTER_H  


/* Different ways to set an adjustable threshold */
typedef enum thresholdtype { LOW_THRESHOLD, 
                     MIDDLE_THRESHOLD,
                     HIGH_THRESHOLD }ThresholdType;


/* Self contained structure used to filter A to D readings. */
struct filter_data
{
        short     average;
        short     accumulator;
        short     count;
        short     master;
        short     delta;
        short     period;
        short     shift;
};
typedef struct filter_data FilterData;

/* Self contained structure to dynamically adjust a treshold. */
struct adjust_data
{
        FilterData      reading;
        FilterData      high;
        FilterData      low;
        ThresholdType   thresholdType; 
        short             bandwidth;
        short             threshold;
};
typedef struct adjust_data AdjustData;     

/* Function Prototypes */
void filter( FilterData *object, short reading );
#if 0
void adjust( AdjustData *object, short reading );
#endif
#endif


