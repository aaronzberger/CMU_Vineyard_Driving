#include "pid.h"

PID::PID(double p, double i, double d, double f = 0) 
    : kP{p}, kI{i}, kD{d}, kF{f}, errorSum{0}, firstLoop{true} {}

void PID::reset() {
    
}