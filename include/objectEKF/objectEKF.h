// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// objectEKF.h
// Purpose: Class that Initializes a Kalman filter using the KFilter library.

// @author Unnar Þór Axelsson
// @version 1.0 19/01/16
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef OBJECTEKF_H
#define OBJECTEKF_H

#include <kalman/ekfilter.hpp>


class ObjectEKF : public Kalman::EKFilter<double,1,false,true,false> {
public:
        ObjectEKF();
        ~ObjectEKF(){}

        // Methods that call the respective methods from the Kfilter library (http://kalman.sourceforge.net/doc/example.html)
        // but also update the time by calling updateTime();
        void initT(unsigned long time_, Vector &x_, Matrix &P_);
        void timeUpdateStepT(unsigned long time_, Vector &u_);
        void measureUpdateStepT(unsigned long time_, Vector &z_);
        void stepT(unsigned long time_, Vector &u_, Vector &z_);
        double timeSinceMeasurement(void);

protected:
        // Create the basic ekf matrices and processes, see KFilter library.
        void makeA();
        void makeBaseA();
        void makeBaseH();
        void makeBaseV();
        void makeBaseR();
        void makeBaseW();
        void makeBaseQ();

        void makeProcess();
        void makeMeasure();

        // Updates the timestamp and deltaT according to the new time measurement.
        void updateTime(unsigned long time_, bool measure);

        double deltaT;
        double timestamp;
        double timestampmeasure;
};

typedef ObjectEKF::Vector Vector;
typedef ObjectEKF::Matrix Matrix;

#endif
