#include "pid.h"
#include <limits>
#include <algorithm>
#include <cmath>
#include <math.h>

PID::PID(double p = 0, double i = 0, double d = 0) 
    :firstLoop{true}, inverted{false}, setPoint{nan}, 
    maxOutput{nan}, minOutput{nan}, maxError{nan}, 
    maxIOutput{nan}, lastSensorData{nan} {
        setP(p);
        setI(i);
        setD(d);
    }

void PID::setP(double p) {
    kP = p;
    checkGainSigns();
}

void PID::setI(double i) {
    kI = i;
    //Scale the integral to smooth kI changing transition
    if(i != 0) {
        integral *= kI / i;
    }
    //Set a maxError to avoid windup
    if(maxIOutput != nan) {
        maxError = maxIOutput / kI;
    }
    checkGainSigns();
}

void PID::setD(double d) {
    kD = d;
    checkGainSigns();
}


void PID::setPID(double p, double i, double d) {
    setP(p);
    setI(i);
    setD(d);
}

void PID::setMaxIOutput(double maxIOutput) {
    this->maxIOutput = maxIOutput;
    //Set a maxError to avoid windup
    if(maxIOutput != nan) {
        maxError = maxIOutput / kI;
    }
}

void PID::setOutputLimits(double minOutput, double maxOutput) {
    this->maxOutput = maxOutput;
    this->minOutput = minOutput;
}

void PID::setSetPoint(double setPoint) {
    this->setPoint = setPoint;
}

void PID::setInverted(bool inverted) {
    this->inverted = inverted;
}

double PID::calculate(double sensorData, double setPoint, double time) {
    if(firstLoop) {
        lastSensorData = sensorData;
        lastTime = time;
        firstLoop = false;
        return 0;
    }

    this->setPoint = setPoint;
    double dt = time - lastTime;

    double error = sensorData - setPoint;

    //Calculate P
    double pOutput{kP * error};

    //Calculate I
    integral += error * dt;
    if(maxError != nan) {
        integral = std::clamp(integral, -maxError, maxError);
    }
    double iOutput{kI * integral};

    if(maxIOutput != nan) {
        iOutput = std::clamp(iOutput, -maxIOutput, maxIOutput);
    }

    //Calculate D
    double derivative{(sensorData - lastSensorData) / dt};
    double dOutput{derivative * kD * -1};

    //Calculate the total
    double totalOutput{pOutput + iOutput + dOutput};
    totalOutput = std::clamp(totalOutput, -maxOutput, maxOutput);

    this->lastSensorData = sensorData;
    this->lastTime = time;

    return totalOutput;
}

double PID::calculate(double sensorData, double time) {
    calculate(sensorData, this->setPoint, time);
}

void PID::reset() {
    integral = 0;
    firstLoop = true;
}

void PID::checkGainSigns() {
    if(inverted) {
        kP = -std::abs(kP);
        kI = -std::abs(kI);
        kD = -std::abs(kD);
    } else {
        kP = std::abs(kP);
        kI = std::abs(kI);
        kD = std::abs(kD);
    }
}