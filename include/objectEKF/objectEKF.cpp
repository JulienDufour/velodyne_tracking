
#include "objectEKF.h"
#include <cmath>
#include <iostream>

ObjectEKF::ObjectEKF(){
	setDim(6,0,3,3,3);
	deltaT = 0.1;
	timestamp = -1.0;
}


void ObjectEKF::initT(unsigned long time_, Vector &x_, Matrix &P_){
	ObjectEKF::updateTime(time_, true);
	init(x_, P_);
}

void ObjectEKF::timeUpdateStepT(unsigned long time_, Vector &u_){
	ObjectEKF::updateTime(time_, false);
	Vector tmp(0);
	timeUpdateStep(tmp);
}

void ObjectEKF::measureUpdateStepT(unsigned long time_, Vector &z_){
	ObjectEKF::updateTime(time_, true);
	measureUpdateStep(z_);
}

void ObjectEKF::stepT(unsigned long time_, Vector &u_, Vector &z_){
	ObjectEKF::updateTime(time_, true);
	step(u_, z_);
}

void ObjectEKF::updateTime(unsigned long time_, bool measure){
	if(timestamp > 0.0){
		deltaT = time_/1000000000.0 - timestamp;
	}
	timestamp = time_/1000000000.0;
	if(measure) timestampmeasure = timestamp;
}

double ObjectEKF::timeSinceMeasurement(void){
	return timestamp - timestampmeasure;
}



void ObjectEKF::makeProcess(){
	Vector x_(x.size());
	x_(1) = x(1) + deltaT * x(4);
	x_(2) = x(2) + deltaT * x(5);
	x_(3) = x(3) + deltaT * x(6);
	x_(4) = x(4);
	x_(5) = x(5);
	x_(6) = x(6);
	x.swap(x_);
}

void ObjectEKF::makeMeasure(){
        z(1)=x(1);
        z(2)=x(2);
        z(3)=x(3);
}

void ObjectEKF::makeBaseA(){
	A(1,1) = 1;
	A(1,2) = 0;
	A(1,3) = 0;
	A(1,4) = deltaT;
	A(1,5) = 0;
	A(1,6) = 0;

	A(2,1) = 0;
	A(2,2) = 1;
	A(2,3) = 0;
	A(2,4) = 0;
	A(2,5) = deltaT;
	A(2,6) = 0;

	A(3,1) = 0;
	A(3,2) = 0;
	A(3,3) = 1;
	A(3,4) = 0;
	A(3,5) = 0;
	A(3,6) = deltaT;

	A(4,1) = 0;
	A(4,2) = 0;
	A(4,3) = 0;
	A(4,4) = 1;
	A(4,5) = 0;
	A(4,6) = 0;

	A(5,1) = 0;
	A(5,2) = 0;
	A(5,3) = 0;
	A(5,4) = 0;
	A(5,5) = 1;
	A(5,6) = 0;

	A(6,1) = 0;
	A(6,2) = 0;
	A(6,3) = 0;
	A(6,4) = 0;
	A(6,5) = 0;
	A(6,6) = 1;
}

void ObjectEKF::makeA(){
	A(1,4) = deltaT;
	A(2,5) = deltaT;
	A(3,6) = deltaT;
}

void ObjectEKF::makeBaseH(){

	H(1,1) = 1;
	H(1,2) = 0;
	H(1,3) = 0;
	H(1,4) = 0;
	H(1,5) = 0;
	H(1,6) = 0;
	H(2,1) = 0;
	H(2,2) = 1;
	H(2,3) = 0;
	H(2,4) = 0;
	H(2,5) = 0;
	H(2,6) = 0;
	H(3,1) = 0;
	H(3,2) = 0;
	H(3,3) = 1;
	H(3,4) = 0;
	H(3,5) = 0;
	H(3,6) = 0;
}

void ObjectEKF::makeBaseW(){
	W(1,1) = 0;
	W(1,2) = 0;
	W(1,3) = 0;
	W(2,1) = 0;
	W(2,2) = 0;
	W(2,3) = 0;
	W(3,1) = 0;
	W(3,2) = 0;
	W(3,3) = 0;
	W(4,1) = 1;
	W(4,2) = 0;
	W(4,3) = 0;
	W(5,1) = 0;
	W(5,2) = 1;
	W(5,3) = 0;
	W(6,1) = 0;
	W(6,2) = 0;
	W(6,3) = 1;
}

void ObjectEKF::makeBaseQ(){

	Q(1,1) = 10;
	Q(1,2) = 0;
	Q(1,3) = 0;
	Q(2,1) = 0;
	Q(2,2) = 10;
	Q(2,3) = 0;
	Q(3,1) = 0;
	Q(3,2) = 0;
	Q(3,3) = 10;
}

void ObjectEKF::makeBaseV(){
	V(1,1) = 1;
	V(1,2) = 0;
	V(1,3) = 0;
	V(2,1) = 0;
	V(2,2) = 1;
	V(2,3) = 0;
	V(3,1) = 0;
	V(3,2) = 0;
	V(3,3) = 1;
}

void ObjectEKF::makeBaseR(){

	R(1,1) = 10;
	R(1,2) = 0;
	R(1,3) = 0;
	R(2,1) = 0;
	R(2,2) = 10;
	R(2,3) = 0;
	R(3,1) = 0;
	R(3,2) = 0;
	R(3,3) = 10;
}
