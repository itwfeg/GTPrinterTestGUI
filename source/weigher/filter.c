#include "filter.h"
#include "stdlib.h"


/******************************************************************************/
/*!   \fn void filter( FilterData *object, INTEGER reading )                                                    
 
      \brief
        This function 
        
      \author
        
*******************************************************************************/ 
void filter( FilterData *object, short reading )
{
    if (abs (object->master - reading) > object->delta)
    {
        /* reading is on the move */
        /* do some initialization */
        object->master = reading;
        object->count = 0; 
        object->accumulator = 0;
    }
    else
    {
        /* accumulate readings */
        object->accumulator += reading;
        ++object->count;

        /* when the readings have been stable within the  */
        /* designated bounds for some period of time.     */
        if (object->count == object->period)
        {
            /* determine an average of the accumulated readings */
            object->average = object->accumulator >> object->shift;
               
            /* do some initialization.*/                       
            object->master = object->average;
            object->count = 0;
            object->accumulator = 0;
        }
    }
}

   
#if 0
/******************************************************************************/
/*!   \fn void filter( FilterData *object, INTEGER reading )                                                    
 
      \brief
        This function 
        
      \author
          Aaron Swift
*******************************************************************************/ 
void adjust( AdjustData *object, short reading )
{
    /* Determine whether the new reading is to be figured into the */
    /* high average or the low average.                        */
            
    if (reading > object->threshold)
    {
        filter(&object->high, reading);
           
        /* maintain the bandwidth. */        
        object->low.average = object->high.average - object->bandwidth; 
        object->low.master = object->low.average;
        object->low.accumulator = 0;
        object->low.count = 0;
    
        /* check for up against a rail. */    
        if (object->low.average < 0)
        {
            object->low.average = 0;
            object->low.master = 0;
            
            object->high.average = object->bandwidth;
            object->high.master = object->high.average;
            object->high.accumulator = 0;
            object->high.count = 0;
        }
    }
    else
    {
        filter(&object->low, reading);
                    
        /* maintain the bandwidth. */        
        object->high.average = object->low.average + object->bandwidth; 
        object->high.master = object->high.average;
        object->high.accumulator = 0; 
        object->high.count = 0;
            
        /* check for up against a rail. */           
        if (object->high.average > 255)
        {
            object->high.average = 255;
            object->high.master = 255;
            
            object->low.average = 255 - object->bandwidth;
            object->low.master = object->low.average;
            object->low.accumulator = 0;
            object->low.count = 0;
        }
    }
    
    /* calculate the new threshold.*/
    switch( object->thresholdType )
    {
        case LOW_THRESHOLD:
        {
            object->threshold = 
                (object->low.average >> 1) + 
                ((object->high.average + object->low.average) >> 2);
                        
            break;
        }
        case MIDDLE_THRESHOLD:
        {
            object->threshold = 
                (object->high.average + object->low.average) >> 1;
                
            break;
        }
        case HIGH_THRESHOLD:
        {
            object->threshold = 
                (object->high.average >> 1) +
                ((object->high.average + object->low.average) >> 2);
                        
            break;
        }
    }
}
#endif    
    




