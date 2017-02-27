#include <iostream>
#include <fstream>
#include "Simulate.h"
#include "LoadData.h"
#include "Utility.h"

Simulate::Simulate(){
}

Simulate::~Simulate(){

}


void Simulate::emulateIMU(int inputMode){
	std::string line;
	MatrixXd poseMatrix(3, 4);
	static bool firstFlag = true;
	static int id = 0;

	switch (inputMode){
	case KITTI:
		static std::ifstream myfile("E:/dataset/KITTI/poses/00.txt");

		if (myfile.is_open())
		{
			for (int i = 0; i < MAX_DATA+2; i++){
				getline(myfile, line);
				std::istringstream in(line);

				double data;
				in >> data;	poseMatrix(0, 0) = data;
				in >> data;	poseMatrix(0, 1) = data;
				in >> data;	poseMatrix(0, 2) = data;
				in >> data;	poseMatrix(0, 3) = data;
				in >> data;	poseMatrix(1, 0) = data;
				in >> data;	poseMatrix(1, 1) = data;
				in >> data;	poseMatrix(1, 2) = data;
				in >> data;	poseMatrix(1, 3) = data;
				in >> data;	poseMatrix(2, 0) = data;
				in >> data;	poseMatrix(2, 1) = data;
				in >> data;	poseMatrix(2, 2) = data;
				in >> data;	poseMatrix(2, 3) = data;

				Vector3d extractedPos(3);
				extractedPos(0) = poseMatrix(0, 3);
				extractedPos(1) = poseMatrix(1, 3);
				extractedPos(2) = poseMatrix(2, 3);

				Matrix3d extractedRot;
				Vector3d euler;
				extractedRot = poseMatrix.block(0, 0, 3, 3);
				euler = extractedRot.eulerAngles(0, 1, 2);

				pos.push_back(extractedPos);
				ori.push_back(euler);
			}
		}

		static std::ifstream timefile("E:/dataset/KITTI/sequences_calib/00/times.txt");
		double currTime = 0.0;

		if (timefile.is_open())
		{
			for (int i = 0; i < MAX_DATA + 2; i++){
				getline(timefile, line);
				std::istringstream in(line);

				in >> currTime;

				time.push_back(currTime);
			}
		}


		break;
	}

	for (int n = 0; n < MAX_DATA; n++){
		// Compute Acc
		{
			Vector3d acc;
			Vector3d v_n, v_nm;				// velocity n(t), velocity n(t+1)
			Vector3d pos_nm, pos_n, pos_np;
			double dt_n, dt_nm;				// velocity n(t), velocity n(t+1)

			// Position
			if (n == 0){
				pos_nm = pos[n];
				dt_nm = time[n + 1] - time[n];
			}
			else{
				pos_nm = pos[n - 1];
				dt_nm = time[n] - time[n - 1];
			}
			pos_n = pos[n];
			dt_n = time[n + 1] - time[n];
			pos_np = pos[n + 1];

			// Velocity
			if (n == 0){
				v_nm = (pos_np - pos_n) / dt_n;
			}
			else{
				v_nm = (pos_n - pos_nm) / dt_nm;
			}
			v_n = (pos_np - pos_n) / dt_n;
			//std::cout << v_n<< std::endl<<v_nm << std::endl;

			// Acc
			acc = (v_n - v_nm) / ((dt_n + dt_nm) / 2);
			acc_I.push_back(acc);
			//std::cout << acc << std::endl;
			//std::cout << std::endl;
		}

		// Compute Angular Velocity
		{
			VectorXd dOri;
			double dt;
			double phi, th, psi;
			if (n == 0){
				dOri = ori[n] - ori[n];
				dOri = eulerInRange(dOri);
				dt = time[n] - time[n];
				phi = ori[n][0];
				th  = ori[n][1];
				psi = ori[n][2];
			}
			else{
				dOri = ori[n] - ori[n-1];
				dOri = eulerInRange(dOri);
				dt = time[n] - time[n - 1];
				phi = ori[n-1][0];
				th = ori[n - 1][1];
				psi = ori[n - 1][2];
			}
			Matrix3d L;
			L << 1, 0, -sin(th),
				0,  cos(phi), sin(phi) * cos(th),
				0, -sin(phi), cos(phi) * cos(th);
			std::cout << L*dOri / dt << std::endl;
			//std::cout << std::endl;
			avel_B.push_back(L*dOri / dt);
		}
	}


}

void Simulate::simulateForCK(){
	simOri.push_back(ori[0]);
	simOri.push_back(ori[1]);
	simOri.push_back(ori[2]);

	for (int n = 3; n < MAX_DATA; n++){
		double phi = simOri.back()[0], th = simOri.back()[1], psi = simOri.back()[2];
		Matrix3d L;
		L << 1, sin(phi)*tan(th), cos(phi)*tan(th),
			0, cos(phi), -sin(phi),
			0, sin(phi) / cos(th), sin(phi) / cos(th);
		Vector3d dOri = L*avel_B[n - 1];
		double dt = time[n] - time[n - 1];
		simOri.push_back(simOri.back() + dOri*dt);
	}

	for (int n = 0; n < MAX_DATA; n++){
		std::cout << ori[n] << std::endl;
		std::cout << simOri[n] << std::endl;
		std::cout  << std::endl;
	}

}