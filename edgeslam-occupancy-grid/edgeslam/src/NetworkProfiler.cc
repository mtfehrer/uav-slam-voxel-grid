/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "NetworkProfiler.h"

#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <string>
#include <unistd.h>

namespace ORB_SLAM2
{

NetworkProfiler::NetworkProfiler()
{
  outputFileName = "Bandwidths.txt";
  killClient = false;
}

void NetworkProfiler::startServer(std::string args)
{
  std::string cmd = "iperf3 -s -D " + args;
  system(cmd.c_str());
}

void NetworkProfiler::startClient(std::string ip, int t_secs) {
  clientThread = new std::thread(&NetworkProfiler::clientWorker, this, ip, t_secs);
}

void NetworkProfiler::stopClient()
{
  killClient = true;
  clientThread -> join();

  std::ofstream outputFile;
  outputFile.open(outputFileName, std::ofstream::out | std::ofstream::trunc);

  for (int i = 0; i < timestamps.size(); i++) {
    outputFile << timestamps[i] << " " << bandwidths[i] << std::endl;
  }

  outputFile.close();
}

void NetworkProfiler::clientWorker(std::string ip, int t_secs) {
  std::ofstream tempfile;
  tempfile.open("temp.txt", std::ofstream::out | std::ofstream::trunc);
  tempfile.close();

  std::string cmd = "iperf3 -c " + ip + " --format k --timestamps=\"%s \" --logfile temp.txt --time " + std::to_string(t_secs) + " &";
  system(cmd.c_str());

  sleep(3);

  for (int t = 0; t < t_secs; t++) {
    if (killClient) {return;}

    std::ifstream logFile("temp.txt");
    std::string currLine;
    std::string timestamp;
    std::string bandwidth;

    if (logFile.is_open()) {
      for (int i = 0; i < 3; i++) {
	getline(logFile, currLine);
      }

      for (int i = 0; i < t; i++) {
	getline(logFile, currLine);
      }

      logFile >> timestamp;

      for (int i = 0; i < 6; i++) {logFile >> bandwidth;}

      logFile >> bandwidth;

      timestamps.push_back(timestamp);
      bandwidths.push_back(bandwidth);
    }

    sleep(1);
  }

}

}//namespace ORB_SLAM
