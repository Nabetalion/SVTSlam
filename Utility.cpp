#include <iostream>
#include <fstream>
#include "utility.h"


Vector3d eulerInRange(Vector3d euler){
	Vector3d modifiedEuler;

	for (int i = 0; i < 3; i++){
		if (euler[i] > M_PI / 2){
			modifiedEuler[i] = euler[i] - M_PI / 2;
		}
		else if (euler[i] < -M_PI / 2){
			modifiedEuler[i] = euler[i] + M_PI / 2;
		}
		else{
			modifiedEuler[i] = euler[i];
		}
	}
	return modifiedEuler;
}