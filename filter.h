#ifndef FILTER_H
#define FILTER_H

float limit_low_past_filter(float current_value,float new_value);

float low_past_filter(float current_value,float new_value);

float low_past_filter_z(float current_value,float new_value);

float limit(float x, float min, float max);

#endif // FILTER_H
