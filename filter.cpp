#include "filter.h"
#include <math.h>

#define pi 3.1415926
#define toDeg (180/pi)
#define toRad (pi/180)


///**Calculate Function**///
float limit_low_past_filter(float current_value,float new_value)
{   /*
    the filter of velocity
    Function:
         limit the max bias between two sample value and then use low past filter
    Meaning:
         self.limit_value can be adjusted by the actual situation
         value which is the effective value
         new value is the current sample value
         This filter can return the useful value
    */
    float limit_value;  //fan_percent
    float low_pass_a;



    limit_value = 10; // the data between two value is lower than 0.3
    low_pass_a = 0.25;


    if (fabs(current_value - new_value) > limit_value)
        return current_value;
    else
        return(low_pass_a * new_value + (1 - low_pass_a) * current_value);
}

float low_past_filter(float current_value,float new_value)
{   /*
    the filter of velocity
    Function:
         low past filter
    Meaning:
         current_value is the last speed and new_value is the present moment speed,
         they can filter out high frequency interference signals by low_pass_filter equation
    */
    float low_pass_a;
    float limit_value;
    low_pass_a = 0.08;
    limit_value = 45;
    if (fabs(new_value) > limit_value)
        return current_value;
    else
        return(low_pass_a * new_value + (1 - low_pass_a) * current_value);

}


float low_past_filter_z(float current_value,float new_value)
{   /*
    the filter of velocity
    Function:
         low past filter
    Meaning:
         current_value is the last speed and new_value is the present moment speed,
         they can filter out high frequency interference signals by low_pass_filter equation
    */
    float low_pass_a;
    low_pass_a = 0.05;
    return(low_pass_a * new_value + (1 - low_pass_a) * current_value);
}

float limit(float x, float min, float max) {
    if (x > max) {
        return max;
    } else if (x < min) {
        return min;
    } else {
        return x;
    }
}
