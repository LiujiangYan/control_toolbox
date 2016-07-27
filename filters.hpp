#ifndef CONTROL_TOOLBOX__FILTERS_H
#define CONTROL_TOOLBOX__FILTERS_H

#include <algorithm>

namespace filters{
    
    /** Clamp value a between b and c */
    template<typename T>
    static inline const T& clamp(const T &a, const T &b, const T &c){
        return std::min<T>(std::max<T>(b, a), c);
    }

    /** Exponential smoothing filter. Alpha is between 0 and 1. Values closer to 0 weight the last smoothed value more heavily */
    
    static inline double exponentialSmoothing(double current_raw_value, double last_smoothed_value, double alpha){
        return alpha*current_raw_value + (1-alpha)*last_smoothed_value;
    }
 }

#endif