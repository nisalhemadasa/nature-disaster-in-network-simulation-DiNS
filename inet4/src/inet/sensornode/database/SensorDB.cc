#include "SensorDB.h"

namespace inet {

    Define_Module(SensorDB);

    vector<vector<string>> SensorDB::readMatrix(string fileName) {
        string delimeter = " ";
        ifstream file(fileName);
        vector<vector<string>> dataList;
        string line = "";
        // Iterate through each line and split the content using delimeter
        while (getline(file, line)){
            vector<string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
            dataList.push_back(vec);
        }
        // Close the File
        file.close();
        return dataList;
    }

    void SensorDB::initialize(){
        const char *csvposition = par("csvposition").stringValue();
        const char *csvhumidity = par("csvhumidity").stringValue();
        const char *csvtemperature = par("csvtemperature").stringValue();
        dbpositions=readMatrix(csvposition);
        dbhumidity=readMatrix(csvhumidity);
        dbtemperature=readMatrix(csvtemperature);
    }

    double SensorDB::getTemperature(int sample, int index){
        double temperature=stod(dbtemperature[sample][index]);
        return temperature;
    }

    double SensorDB::getHumidity(int sample, int index){
        double humidity=stod(dbhumidity[sample][index]);
        return humidity;
    }

    Coord SensorDB::getPosition(int index){
        Coord position;
        position.x=stod(dbpositions[0][index]);
        position.y=stod(dbpositions[1][index]);
        position.z=stod(dbpositions[2][index]);
        return position;
    }

    void SensorDB::handleMessage(cMessage *msg){

    }

}
