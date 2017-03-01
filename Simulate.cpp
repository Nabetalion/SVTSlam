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
			vel.push_back(v_n);
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
				dOri = eulerDiffInRange(dOri);
				dt = time[n] - time[n];
				phi = ori[n][0];
				th  = ori[n][1];
				psi = ori[n][2];
			}
			else{
				dOri = ori[n] - ori[n-1];
				dOri = eulerDiffInRange(dOri);
				dt = time[n] - time[n - 1];
				phi = ori[n - 1][0];
				th  = ori[n - 1][1];
				psi = ori[n - 1][2];

				std::cout << ori[n - 1] << std::endl;
				std::cout << ori[n] << std::endl;
				std::cout << dOri << std::endl;
				//std::cout << std::endl;
			}
			Matrix3d L;
			// https://www.princeton.edu/~stengel/MAE331Lecture9.pdf
			L << 1, 0, -sin(th),
				0,  cos(phi), sin(phi) * cos(th),
				0, -sin(phi), cos(phi) * cos(th);
			//std::cout << L*dOri / dt << std::endl;
			avel_B.push_back(L*dOri / dt);

			/////////////////////////////////////////////////
			if (n < 3){
				simOri.push_back(ori[n]);
			}
			else{
				Matrix3d L1,L2;
				L1 << 1, 0, -sin(th),
					0, cos(phi), sin(phi) * cos(th),
					0, -sin(phi), cos(phi) * cos(th);
				L2 << 1, sin(phi)*tan(th), cos(phi)*tan(th),
					0, cos(phi), -sin(phi),
					0, -sin(phi)/tan(th),cos(phi) / tan(th);
				dt = time[n] - time[n - 1];

				//std::cout << L1 << std::endl;
				//std::cout << L1.inverse() << std::endl;
				//std::cout << L2 << std::endl;

				simOri.push_back(simOri[n - 1] + (L1.inverse())*avel_B[n]*dt);
			}

			std::cout << ori[n] << std::endl;
			std::cout << simOri[n] << std::endl;
			std::cout << std::endl;
			
		}
	}


}

void Simulate::simulateForCK(){
	simPos.push_back(pos[0]);
	simPos.push_back(pos[1]);
	simPos.push_back(pos[2]);
	simVel.push_back(vel[0]);
	simVel.push_back(vel[1]);
	simVel.push_back(vel[2]);

	simOri.push_back(ori[0]);
	simOri.push_back(ori[1]);
	simOri.push_back(ori[2]);

	for (int n = 3; n < MAX_DATA; n++){
		//std::cout <<"n:"<< n << std::endl;
		double dt = time[n] - time[n - 1];

		// simulate pos and velocity
		simPos.push_back(simPos.back() + simVel.back()*dt);
		simVel.push_back(simVel.back() + acc_I[n]*dt);

		// Simulate angle
		double phi = simOri.back()[0], th = simOri.back()[1], psi = simOri.back()[2];
		Matrix3d L;
		// Note: the matrix analytically computed is unstable, because tan(th) is in denominator.
		L << 1, 0, -sin(th),
			0, cos(phi), sin(phi) * cos(th),
			0, -sin(phi), cos(phi) * cos(th);
		//std::cout << L.determinant() << std::endl;
		Vector3d dOri = L.inverse()*avel_B[n];
		Vector3d simOriData = simOri.back() + dOri*dt;
		//Vector3d simOriDataInRange = eulerInRange(simOriData);
		simOri.push_back(simOriData);

		// For check
		//std::cout << pos[n] << std::endl;
		//std::cout << simPos[n] << std::endl;
		//std::cout << vel[n] << std::endl;
		//std::cout << simVel[n] << std::endl;
		//std::cout << avel_B[n] << std::endl;
		//std::cout << ori[n] << std::endl;
		//std::cout << simOri[n] << std::endl;
		//std::cout << std::endl;
	}


}