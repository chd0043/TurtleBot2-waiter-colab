#include <ros/ros.h>
#include <ios>
#include <fstream>
#include <ecl/time.hpp>

#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <algorithm>
#include <unistd.h>

const std::string FOLDERPATH = "/home/turtlebot/catkin_ws/src/tb_waiter/data/";
const std::string FILE_EXTENSION = ".csv";

char login[10];
int err = getlogin_r(login, sizeof(login));
std::string str(login);

struct dataStruct{
  std::string winner;
  std::string position;
  double distance;
  double time;
};

class DataLogger
{
private:
  std::stringstream filepath;
  std::string name;
  int counter;

public:
  DataLogger():
      counter(0) { }
  ~DataLogger() {}

  void initLogFile(std::string agent_name, std::string folder="1")
  {
    name = agent_name;
    std::string timeStamp = currentDateTime();
    filepath << FOLDERPATH << "/"<< folder << "/"<< timeStamp <<"_"<< agent_name << FILE_EXTENSION;

    //ROS_INFO("%s", filepath.str().c_str());
    std::ofstream log(filepath.str().c_str(), std::ios_base::app | std::ios_base::out);
    // titles of columns
    log << "Num.,timeStamp, Name, assigner, Position, execute_Time, execute_Distance\n";
  }

  void addline(dataStruct data)
  {
    // adds new data line to Log file.
    counter++;
    std::string timeStamp = currentDateTime();
    std::ofstream log(filepath.str().c_str(), std::ios_base::app | std::ios_base::out);
    // replace commas from data.position
    std::replace( data.position.begin(), data.position.end(), ',', '/');

    log << counter <<","<<timeStamp<<","<< name <<","<<data.winner<<","<< data.position <<","
        << data.time<<","<<data.distance<<"\n";
  }

protected:

  std::string getLinuxUserName()
  {
    char login[20];
    int err = getlogin_r(login, sizeof(login));
    return(std::string(login));
  }

  const std::string currentDateTime()
  {
    // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return (buf);
  }
};
