//
// Created by xiang on 2021/10/11.
//

#include "utils.h"

namespace faster_lio {

std::map<std::string, Timer::TimerRecord> Timer::records_;
bool Timer::enabled_ = false;

}