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

#include <iostream>
#include <fstream>
double getAbsoluteScale(){
	static int frame_id = 0;
	std::string line;
	int i = 0;
	std::ifstream myfile("E:/dataset/KITTI/poses/00.txt");
	double x = 0, y = 0, z = 0;
	double x_prev, y_prev, z_prev;
	if (myfile.is_open())
	{
		while ((getline(myfile, line)) && (i <= frame_id))
		{
			z_prev = z;
			x_prev = x;
			y_prev = y;
			std::istringstream in(line);
			//cout << line << '\n';
			for (int j = 0; j<12; j++)  {
				in >> z;
				if (j == 7) y = z;
				if (j == 3)  x = z;
			}

			i++;
		}
		myfile.close();
	}

	else {
		std::cout << "Unable to open file";
		return 0;
	}

	frame_id++;
	return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));
}


void EstimateState::estimate(){

}

void EstimateState::propagate(){
	Vector3d pos = state.block(0, 0, 3, 1);
	Vector3d ori = state.block(3, 0, 3, 1);
	Vector3d vel = state.block(6, 0, 3, 1);

	pos += vel*dt;			// if vel is in the inertia frame
	// pos = rot*vel*dt;	// if vel is in the body frame

	vel += imu.imu*dt;

	double phi = ori[0], th = ori[1], psi = ori[2];
	Matrix3d L;
	// Note: the matrix analytically computed is unstable, because tan(th) is in denominator.
	L << 1, 0, -sin(th),
		0, cos(phi), sin(phi) * cos(th),
		0, -sin(phi), cos(phi) * cos(th);
	ori += L.inverse()*imu.gyro*dt;

	this->state(0) = pos(0);
	this->state(1) = pos(1);
	this->state(2) = pos(2);
	this->state(3) = ori(0);
	this->state(4) = ori(1);
	this->state(5) = ori(2);
	this->state(6) = vel(0);
	this->state(7) = vel(1);
	this->state(8) = vel(2);
}

void EstimateState::update(){
	this->state(0) = this->gps(0);
	this->state(1) = this->gps(1);
	this->state(2) = this->gps(2);
	this->state(3) = this->attitudeSensor(0);
	this->state(4) = this->attitudeSensor(1);
	this->state(5) = this->attitudeSensor(2);
	//std::cout << state << "\n" << gps << std::endl;

	static int id = 0;
	if (id < 1){
		pos(0) = state(0);
		pos(1) = state(1);
		pos(2) = state(2);
		rot = Matrix3d::Identity(3, 3);
		t_5pt = Vector3d::Zero(3);
		R_5pt = Matrix3d::Identity(3, 3);
	}
	//std::cout << "Estimate State" << std::endl;
	//std::cout << pos << std::endl;
	//std::cout << rot << std::endl;
	//std::cout << t_5pt << std::endl;
	//std::cout << R_5pt << std::endl;

	lambda = getAbsoluteScale();
	pos = pos + lambda*(rot*t_5pt);
	rot = R_5pt*rot;
	//std::cout << pos << std::endl;
	//std::cout << rot << std::endl;

	id++;
}
void EstimateState::storeIntoHist(){
	stateHist.push_back(state);
	stateHist.push_back(P);
}

void EstimateState::setState(VectorXd recvState){
	this->state = recvState;
}
void EstimateState::setP(MatrixXd recvP){
	this->state = recvP;
}

void EstimateState::setDt(double recvDt){
	this->dt = recvDt;
}

void EstimateState::setImu(ImuData recvImu){
	this->imu = recvImu;
}


void EstimateState::setGps(VectorXd recvGPS){
	this->gps = recvGPS;
}

void EstimateState::setAttitude(VectorXd recvAtt){
	this->attitudeSensor = recvAtt;
}
void EstimateState::setExtransic(Matrix3d recvR, Vector3d recvt){
	R_5pt = recvR;
	t_5pt = recvt;
}

int EstimateState::getOriMode(){
	return oriDataMode;
}
