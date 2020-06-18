#ifndef PID_H
#define PID_H

class PID {
public:
    PID(double p, double i, double d, double f = 0);

    void reset();

    void setP(double p);
    void setI(double i);
    void setD(double d);
    void setF(double f);
    void setPID(double p, double i, double d, double f = 0);
    void setMaxIOutput(double maxIOutput);
    void setMaxOutput(double maxOutput);
    void setSetPoint(double setPoint);
    void setInverted(bool inverted);

    double calculate(double sensorData, double setPoint);
    double calculate(double sensorData);

private:
    //GAINS
    double kP, kI, kD, kF;
    
    double setPoint;

    double errorSum;

    double maxIOutput;

    double maxOutput;
    double minOutput;

    bool firstLoop;
    bool inverted;

    double lastSensorData;
};

#endif