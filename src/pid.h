#ifndef PID_H
#define PID_H

#include <limits>

class PID {
public:
    PID(double p, double i, double d);
    PID() : PID(0.0, 0.0, 0.0) {};

    void reset();

    void setP(double p);
    void setI(double i);
    void setD(double d);
    void setPID(double p, double i, double d);
    void setMaxIOutput(double maxIOutput);
    void setOutputLimits(double minOutput, double maxOutput);
    void setSetPoint(double setPoint);
    void setInverted(bool inverted);

    double calculate(double sensorData, double setPoint, ros::Time time);
    double calculate(double sensorData, ros::Time time);

private:
    void checkGainSigns();
    double clamp(double val, double low, double high);

    static constexpr double nan{std::numeric_limits<double>::quiet_NaN()};

    //GAINS
    double kP, kI, kD;
    
    double setPoint;

    double integral;
    double maxError;

    double maxIOutput;

    double maxOutput;
    double minOutput;

    bool firstLoop;
    bool inverted;

    double lastSensorData;
    ros::Time lastTime;
};

#endif