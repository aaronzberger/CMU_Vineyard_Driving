#include "pid.h"
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>


/**
 * @param p the P gain for the controller
 * @param i the I gain for the controller
 * @param d the D gain for the controller
 */
PID::PID(double p, double i, double d) 
    :firstLoop{true}, inverted{false}, setPoint{nan}, 
    maxOutput{nan}, minOutput{nan}, maxError{nan}, 
    maxIOutput{nan}, lastSensorData{nan}, integral{0} {
        setP(p);
        setI(i);
        setD(d);
    }

/**
 * @brief Update the P gain for the controller
 * 
 * @param p the new P gain
 */
void PID::setP(double p) {
    kP = p;
    checkGainSigns();
}

/**
 * @brief Update the I gain and scale the accumulated integral and maximum integral accumulation.
 * 
 * @param i the new I gain
 */
void PID::setI(double i) {
    kI = i;
    //Scale the integral to smooth kI changing transition
    if(i != 0) {
        integral *= kI / i;
    }
    //Set a maxError to avoid windup
    if(maxIOutput != nan && kI != 0) {
        maxError = maxIOutput / kI;
    }
    checkGainSigns();
}

/**
 * @brief Update the D gain for the controller
 * 
 * @param d the new D gain
 */
void PID::setD(double d) {
    kD = d;
    checkGainSigns();
}

/**
 * @brief Update the P, I, and D gains for the controller
 * 
 * @param p the new P gain
 * @param i the new I gain
 * @param d the new D gain
 */
void PID::setPID(double p, double i, double d) {
    setP(p);
    setI(i);
    setD(d);
}

/**
 * @brief Set the maximum I gain output the controller can produce
 * 
 * @param maxIOutput the maximum I gain output allowed
 */
void PID::setMaxIOutput(double maxIOutput) {
    this->maxIOutput = maxIOutput;
    //Set a maxError to avoid windup
    if(maxIOutput != nan && kI != 0) {
        maxError = maxIOutput / kI;
    }
}

/**
 * @brief Set the output limits for the controller
 * 
 * @param minOutput the minimum output
 * @param maxOutput the maximum output
 */
void PID::setOutputLimits(double minOutput, double maxOutput) {
    this->maxOutput = maxOutput;
    this->minOutput = minOutput;
}

/**
 * @brief Set the set point that the controller will aim towards
 * 
 * @param setPoint the set point
 */
void PID::setSetPoint(double setPoint) {
    this->setPoint = setPoint;
}

/**
 * @brief Set whether the output of the controller should be inverted
 * 
 * @param inverted whether the output should be inverted
 */
void PID::setInverted(bool inverted) {
    this->inverted = inverted;
    checkGainSigns();
}

/**
 * @brief Perform the PID calculations once
 * 
 * @param sensorData the current sensor data
 * @param setPoint the set point for the controller
 * @param time the current time (the controller handles difference in time)
 * @return the output from the controller
 */
double PID::calculate(double sensorData, double setPoint, ros::Time time) {
    if(firstLoop) {
        lastSensorData = sensorData;
        lastTime = time;
        firstLoop = false;
        integral = 0;
        return 0;
    }

    this->setPoint = setPoint;
    double dt = (time - lastTime).toSec();

    double error = sensorData - setPoint;

    //Calculate P
    double pOutput{kP * error};

    //Calculate I
    integral += (error * dt);

    integral = clamp(integral, -maxError, maxError);
    
    double iOutput{kI * integral};

    iOutput = clamp(iOutput, -maxIOutput, maxIOutput);

    //Calculate D
    double derivative{(sensorData - lastSensorData) / dt};
    double dOutput{derivative * kD};

    //Calculate the total
    double totalOutput{pOutput + iOutput + dOutput};
    totalOutput = clamp(totalOutput, -maxOutput, maxOutput);
    
    this->lastSensorData = sensorData;
    this->lastTime = time;

    std::cout << "P: " << pOutput << " I: " << iOutput << " D: " << dOutput << std::endl;

    return totalOutput;
}

/**
 * @brief Perform the PID calculations, keeping the same set point as previously
 * 
 * @param sensorData the current sensor data
 * @param time the current time (the controller handles difference in time)
 * @return the output from the controller
 */
double PID::calculate(double sensorData, ros::Time time) {
    calculate(sensorData, this->setPoint, time);
}

/**
 * @brief Reset the PID controller's accumulated error, time, etc.
 */
void PID::reset() {
    integral = 0;
    firstLoop = true;
}

/**
 * @brief Ensure the signs of the P, I, and D gains are correct after changing them
 */
void PID::checkGainSigns() {
    if(inverted) {
        kP = std::abs(kP) * -1;
        kI = std::abs(kI) * -1;
        kD = std::abs(kD) * -1;
    } else {
        kP = std::abs(kP);
        kI = std::abs(kI);
        kD = std::abs(kD);
    }
}

/**
 * @brief Clamp a double to a minimum and maximum
 * 
 * @param val the value to clamp
 * @param low the lower limit
 * @param high the upper limit
 * @return the clamped value
 */
double PID::clamp(double val, double low, double high) {
    if(std::isnan(low) || std::isnan(high)) return val;
    if(val >= low && val <= high) return val;
    else if(val < low) return low;
    return high;
}