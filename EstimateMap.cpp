#include <iostream>
#include "EstimateMap.h"

EstimateMap::EstimateMap(){
	oriDataMode = EULER;
}

EstimateMap::~EstimateMap(){

}


MatrixXd EstimateMap::createRotationMatrix(VectorXd state){
	Matrix3d rot;

	// Create Rot Matrix
	switch (oriDataMode){
	case EULER:
		rot = AngleAxisd(state(3), Vector3d::UnitX())
			* AngleAxisd(state(4), Vector3d::UnitY())
			* AngleAxisd(state(5), Vector3d::UnitZ());
		break;
	case QUATERNION:
	default:
		// Under construction
		break;
	}

	return rot;
}

void EstimateMap::EstimateLS2(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp,
							VectorXd prePos, VectorXd preOri, VectorXd curPos, VectorXd curOri){

	MatrixXd preRot = createRotationMatrix(preOri);
	MatrixXd curRot = createRotationMatrix(curOri);

	MatrixXd A(2 * 2, 3);	// from 2 view and 2 measurement, 3 dimension
	VectorXd B(2 * 2);		// from 2 view and 2 measurement

	double ud1, vd1, ud2, vd2;
	VectorXd preT = preRot*prePos;	// previous Translation vector in terms of camera frame
	VectorXd curT = curRot*curPos;	// current Translation vector in terms of camera frame
	MatrixXd AT, ATA, ATAI;
	MatrixXd NN = MatrixXd::Zero(4, 4);
	Fp3D computed3Ddata;

	for (int i = 0; i < preFp.size(); ++i){
		ud1 = preFp[i].x - cx;
		vd1 = preFp[i].y - cy;
		B(0) = preT(0)*fx - preT(2)*ud1;
		A(0, 0) = preRot(0, 0)*fx - preRot(2, 0)*ud1;
		A(0, 1) = preRot(0, 1)*fx - preRot(2, 1)*ud1;
		A(0, 2) = preRot(0, 2)*fx - preRot(2, 2)*ud1;
		B(1) = preT(1)*fy - preT(2)*vd1;
		A(1, 0) = preRot(1, 0)*fy - preRot(2, 0)*vd1;
		A(1, 1) = preRot(1, 1)*fy - preRot(2, 1)*vd1;
		A(1, 2) = preRot(1, 2)*fy - preRot(2, 2)*vd1;

		ud2 = curFp[i].x - cx;
		vd2 = curFp[i].y - cy;
		B(2) = curT(0)*fx - curT(2)*ud2;
		A(2, 0) = curRot(0, 0)*fx - curRot(2, 0)*ud2;
		A(2, 1) = curRot(0, 1)*fx - curRot(2, 1)*ud2;
		A(2, 2) = curRot(0, 2)*fx - curRot(2, 2)*ud2;
		B(3) = curT(1)*fy - curT(2)*vd2;
		A(3, 0) = curRot(1, 0)*fy - curRot(2, 0)*vd2;
		A(3, 1) = curRot(1, 1)*fy - curRot(2, 1)*vd2;
		A(3, 2) = curRot(1, 2)*fy - curRot(2, 2)*vd2;

		// Least square estimation: position
		AT = A.transpose();
		ATA = AT*A;
		ATAI = ATA.inverse();
		computed3Ddata.pos = ATAI*AT*B;

		// Least square estimation: covariance
		NN(0, 0) = 1;	// Noise of u of pre		// NN: other element must be zero
		NN(1, 1) = 1;	// Noise of v of pre
		NN(2, 2) = 1;	// Noise of u of cur
		NN(3, 3) = 1;	// Noise of v of cur
		computed3Ddata.P = ATAI*AT*NN*A*ATAI;	// P matrix computed by Least square

		fp3DRLS.push_back(computed3Ddata);
		//std::cout << computed3Ddata.pos << std::endl;
		//std::cout << computed3Ddata.P << std::endl;
	}
	std::cout << fp3DRLS.size() << std::endl;
}
void EstimateMap::EstimateLSm(std::vector<std::vector<cv::Point2f>> fp2DInit, std::vector<VectorXd> stateHist){
	MatrixXd rot[NUMLSINIT];	// 0 is current and data is back to state with order i.
	VectorXd pos[NUMLSINIT];	// 0 is current and data is back to state with order i.
	VectorXd tc[NUMLSINIT];	// trans vector in camera frame. 0 is current and data is back to state with order i.
	VectorXd ori;
	int sizeStateHist = stateHist.size();
	for (int i = 0; i < NUMLSINIT; i++){
		ori = stateHist[sizeStateHist - 1 - i].block(3, 0, 3, 1);	// If Euler
		rot[i] = createRotationMatrix(ori);
		pos[i] = stateHist[sizeStateHist - 1 - i].block(0, 0, 3, 1);
		tc[i] = rot[i] * pos[i];
	}

	MatrixXd A(NUMLSINIT * 2, 3);	// from Multiple view and 2 measurement, 3 dimension
	VectorXd B(NUMLSINIT * 2);		// from Multiple view and 2 measurement
	MatrixXd AT, ATA, ATAI;
	MatrixXd NN = MatrixXd::Zero(2 * NUMLSINIT, 2 * NUMLSINIT);
	double ud[NUMLSINIT], vd[NUMLSINIT];
	Fp3D computed3Ddata;

	for (int i = 0; i < fp2DInit.size(); i++){
		if (fp2DInit[i].size() >= NUMLSINIT){


		}
	}

}

void EstimateMap::EstimateRLS(){

}

void EstimateMap::EstimateNpt(){


}

// to keep the sonsitency with 2d tracking
void EstimateMap::erase3DRLS(int eraseId){
	fp3DRLS.erase(fp3DRLS.begin() + eraseId);
}
// to keep the sonsitency with 2d tracking
void EstimateMap::erase3DEKF(int eraseId){
	fp3DEKF.erase(fp3DEKF.begin() + eraseId);
}

void EstimateMap::setCameraIntrinsic(MatrixXd receivedData){
	this->cameraIntrinsic = receivedData;
	fx = cameraIntrinsic(0, 0);
	fy = cameraIntrinsic(1, 1);
	cx = cameraIntrinsic(0, 2);
	cy = cameraIntrinsic(1, 2);
}


void EstimateMap::setfp2dHistPointer(std::vector<std::vector<cv::Point2f>> *manageFpHist){
	this->fp2dHist = manageFpHist;

	// For check
	/*std::cout << "Hist " << fp2dHist->size() << std::endl;
	for (int i = 0; i < fp2dHist->size(); i++){
		std::cout << (*fp2dHist)[i].size() << "\t";
	}
	std::cout << std::endl;
	*/
}
void EstimateMap::decideFpState(){
	fpState.clear();
	for (int i = 0; i < fp2dHist->size(); i++){
		if ((*fp2dHist)[i].size()<NUMLSINIT){
			fpState.push_back(INIT);
		}
		else if ((*fp2dHist)[i].size()<NUMLSINIT + 1){
			fpState.push_back(RLS);
		}
		else if ((*fp2dHist)[i].size()>=NUMLSINIT + 1){
			fpState.push_back(EKF);
		}
		else{
			fpState.push_back(NONE);
		}
	}
	/*
	for (int i = 0; i < fpState.size(); i++){
		std::cout << fpState[i] << "\t";
	}
	std::cout << std::endl;
	*/
}

#ifdef WITH_VIZ
void EstimateMap::initMapWindow(){
	mapWindow = cv::viz::Viz3d("MapResult");
	//mapWindow.spin();
	// Add coordinate axes
	mapWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
}
void EstimateMap::drawMap(){
	//http://docs.opencv.org/2.4/doc/tutorials/viz/table_of_content_viz/table_of_content_viz.html
	mapWindow.spinOnce(1, true);
}
#endif
