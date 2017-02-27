#include <iostream>
#include "EstimateState.h"

EstimateState::EstimateState(){
	oriDataMode = EULER;

	switch (oriDataMode){
	case EULER:
		state = VectorXd(9);	// pos,ori,vel
		break;
	case QUATERNION:
		state = VectorXd(10);	// pos,ori,vel
	default:
		break;
	}
}

EstimateState::~EstimateState(){

}

void EstimateState::estimate(){

}

void EstimateState::propagate(){

}

void EstimateState::update(){
	this->state(0) = this->gps(0);
	this->state(1) = this->gps(1);
	this->state(2) = this->gps(2);
	this->state(3) = this->attitude(0);
	this->state(4) = this->attitude(1);
	this->state(5) = this->attitude(2);
	//std::cout << state << "\n" << gps << std::endl;
}

void EstimateState::setGps(VectorXd recvGPS){
	this->gps = recvGPS;
}

void EstimateState::setAttitude(VectorXd recvAtt){
	this->attitude = recvAtt;
}

int EstimateState::getOriMode(){
	return oriDataMode;
}
