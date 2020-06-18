#include "pid.h"
#include <limits>
#include <algorithm>

PID::PID(double p = 0, double i = 0, double d = 0, double f = 0) 
    : kP{p}, kI{i}, kD{d}, kF{f}, errorSum{0}, firstLoop{true},
    setPoint{0}, maxIOutput{0}, inverted{false},
    maxOutput{std::numeric_limits<double>::max()}, 
    minOutput{std::numeric_limits<double>::min()} {}

void PID::setP(double p) {
    kP = p;
}

void PID::setI(double i) {
    kI = i;
}

void PID::setD(double d) {
    kD = d;
}

void PID::setF(double f) {
    kF = f;
}

void PID::setPID(double p, double i, double d, double f) {
    setP(p);
    setI(i);
    setD(d);
    setF(f);
}

void PID::setMaxIOutput(double maxIOutput) {
    this->maxIOutput = maxIOutput;
}

void PID::setMaxOutput(double maxOutput) {
    this->maxOutput = maxOutput;
}

void PID::setSetPoint(double setPoint) {
    this->setPoint = setPoint;
}

void PID::setInverted(bool inverted) {
    this->inverted = inverted;
}

double PID::calculate(double sensorData, double setPoint) {
    double error = sensorData - setPoint;

    double pOutput{kP * error};

    errorSum += sensorData;
    double iOutput{kI * errorSum};

    iOutput = std::clamp(iOutput, -maxIOutput, maxIOutput);

    if(firstLoop) {
        lastSensorData = sensorData;
        firstLoop = false;
    }

    double dOutput{(sensorData - lastSensorData) * kD * -1};

    double fOutput{kF * setPoint};

    double totalOutput{pOutput + iOutput + dOutput + fOutput};

    totalOutput = std::clamp(totalOutput, -maxOutput, maxOutput);
}

double PID::calculate(double sensorData) {
    calculate(sensorData, this->setPoint);
}

void PID::reset() {
    errorSum = 0;
    firstLoop = true;
}