#include <iostream>
#include "EstimateMap.h"

EstimateMap::EstimateMap(){
}

EstimateMap::~EstimateMap(){

}

void EstimateMap::EstimateLS(){

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