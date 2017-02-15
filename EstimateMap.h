
#ifndef _ESTIMATEMAP_H_
#define _ESTIMATEMAP_H_

#include <Eigen\dense>
#include <iostream>
#include <vector>

using namespace Eigen;

typedef struct Fp3D{
	Vector3d pos;
	Matrix3d P;
}Fp3D;


class EstimateMap{
private:
	std::vector<Fp3D> fp3DRLS;
	std::vector<Fp3D> fp3DEKF;
public:
	EstimateMap();
	~EstimateMap();

	void EstimateLS();	
	void EstimateRLS();	
	void EstimateNpt();	// Estimate probability of N point for trajectory generation

	void erase3DRLS(int);	// to keep the sonsitency with 2d tracking
	void erase3DEKF(int);	// to keep the sonsitency with 2d tracking
};

#endif
