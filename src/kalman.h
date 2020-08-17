#ifndef KALMAN_H
#define KALMAN_H

#include <cmath>
#include <Eigen/Dense>

class Kalman {
public:
    Kalman(double x, double y, double theta, Eigen::MatrixXd initialState,
           Eigen::Matrix2d covInitial = Eigen::Matrix2d::Identity(), Eigen::Matrix2d modelError = Eigen::Matrix2d::Zero(), 
           Eigen::Matrix2d measurementError = Eigen::Matrix2d::Zero(), Eigen::Matrix2d observationTransform = Eigen::Matrix2d::Identity());
    Eigen::MatrixXd filter(double x, double y, double theta, Eigen::MatrixXd detectedState);
    void setModelError(Eigen::Matrix2d modelError) {this->modelError = modelError;};
    void setMeasurementError(Eigen::Matrix2d measurementError) {this->measurementError = measurementError;};
    
private:
    Eigen::Matrix2d covPrediction, covUpdated, kGain;
    Eigen::MatrixXd statePrediction, stateUpdated;
    double lastRobotX, lastRobotY, lastRobotTheta;
    Eigen::Matrix2d initialCovariance, modelError, measurementError, covInitial, observationTransform;
};

#endif