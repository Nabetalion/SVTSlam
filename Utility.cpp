#include <iostream>
#include <fstream>
#include "utility.h"


Vector3d eulerDiffInRange(Vector3d euler){
	Vector3d modifiedEuler;

	for (int i = 0; i < 3; i++){
		if (euler[i] > M_PI/2){
			modifiedEuler[i] = euler[i] - M_PI ;
		}
		else if (euler[i] < -M_PI/2){
			modifiedEuler[i] = euler[i] + M_PI ;
		}
		else{
			modifiedEuler[i] = euler[i];
		}
	}
	return modifiedEuler;
}

Vector3d eulerInRange(Vector3d euler){
	Vector3d modifiedEuler;

	for (int i = 0; i < 3; i++){
		if (euler[i] < M_PI){
			modifiedEuler[i] = euler[i] + M_PI;
		}
		else if (euler[i] > M_PI){
			modifiedEuler[i] = euler[i] - M_PI;
		}
		else{
			modifiedEuler[i] = euler[i];
		}
	}
	return modifiedEuler;
}