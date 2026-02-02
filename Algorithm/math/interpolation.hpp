#ifndef ALGORITHM_INTERPOLATION_H
#define ALGORITHM_INTERPOLATION_H

#include <cstdint>
#include <stdint.h>

enum InterpolationState
{
    INTERPOLATION_STATE_START,
    INTERPOLATION_STATE_ING,
    INTERPOLATION_STATE_END
};

class Interpolation 
{
private:
    float start_point_;
    float end_point_;
    uint8_t point_num_;
    float output_;
    InterpolationState state_ = INTERPOLATION_STATE_END;
public:
    void Start(float start_point,float end_point,uint8_t point_num)
    {
        start_point_ = start_point;
        end_point_ = end_point;
        point_num_ = point_num;
        state_ = INTERPOLATION_STATE_START;
    }
    float Output()
    {
        static uint8_t times;
        times++;
        if((times >= point_num_ )||(state_ == INTERPOLATION_STATE_END)){
            output_ = end_point_;
            times = 0;
            state_ = INTERPOLATION_STATE_END;
        }else{
            output_ = start_point_ + (((end_point_ - start_point_) / point_num_) * times);
            state_ = INTERPOLATION_STATE_ING;
        }
        return output_;
    }

};



#endif // ALGORITHM_INTERPOLATION_H
