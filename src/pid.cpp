#include "pid.h"
#include <limits>
#include <algorithm>
#include <cmath>
#include <math.h>

PID::PID(double p = 0, double i = 0, double d = 0, double f = 0) 
    :firstLoop{true}, inverted{false}, setPoint{nan}, 
    maxOutput{nan}, minOutput{nan}, maxError{nan}, 
    maxIOutput{nan}, lastSensorData{nan} {
        setP(p);
        setI(i);
        setD(d);
        setF(f);
    }

void PID::setP(double p) {
    kP = p;
    checkGainSigns();
}

void PID::setI(double i) {
    kI = i;
    //Scale the errorSum to smooth kI changing transition
    if(i != 0) {
        errorSum *= kI / i;
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

void PID::setF(double f) {
    kF = f;
    checkGainSigns();
}

void PID::setPID(double p, double i, double d, double f) {
    setP(p);
    setI(i);
    setD(d);
    setF(f);
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

double PID::calculate(double sensorData, double setPoint) {
    this->setPoint = setPoint;

    double error = sensorData - setPoint;

    errorSum += error;
    if(maxError != nan) {
        errorSum = std::clamp(errorSum, -maxError, maxError);
    }

    if(firstLoop) {
        lastSensorData = sensorData;
        firstLoop = false;
    }

    //Calculate P
    double pOutput{kP * error};

    //Calculate I
    double iOutput{kI * errorSum};

    if(maxIOutput != nan) {
        iOutput = std::clamp(iOutput, -maxIOutput, maxIOutput);
    }

    //Calculate D
    double dOutput{(sensorData - lastSensorData) * kD * -1};

    //Calculate F
    double fOutput{kF * setPoint};

    //Calculate the total
    double totalOutput{pOutput + iOutput + dOutput + fOutput};
    totalOutput = std::clamp(totalOutput, -maxOutput, maxOutput);

    lastSensorData = sensorData;

    return totalOutput;
}

double PID::calculate(double sensorData) {
    calculate(sensorData, this->setPoint);
}

void PID::reset() {
    errorSum = 0;
    firstLoop = true;
}

void PID::checkGainSigns() {
    if(inverted) {
        kP = -std::abs(kP);
        kI = -std::abs(kI);
        kD = -std::abs(kD);
        kF = -std::abs(kF);
    } else {
        kP = std::abs(kP);
        kI = std::abs(kI);
        kD = std::abs(kD);
        kF = std::abs(kF);
    }
}