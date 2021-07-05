#ifndef __INET4_SENSORDB_H_
#define __INET4_SENSORDB_H_

#include <omnetpp.h>
#include "inet/libraries/Eigen/Dense"
#include "inet/common/geometry/common/Coord.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

using namespace omnetpp;
using namespace Eigen;
using namespace std;

#define MAXBUFSIZE  ((int) 1e6)
namespace inet {

    class SensorDB : public cSimpleModule{
        protected:
            virtual vector<vector<string>> readMatrix(string fileName);  //Import a 2D matrix from a csv file
            virtual void initialize();
            virtual void handleMessage(cMessage *msg);
            vector<vector<string>> dbpositions;
            vector<vector<string>> dbhumidity;
            vector<vector<string>> dbtemperature;
        public:
            double getTemperature(int sample, int index);
            double getHumidity(int sample, int index);
            Coord getPosition(int index);
    };

} //namespace

#endif
