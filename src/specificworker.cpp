/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 #include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
  //laser_proxy es el puntero y tlaserdata es el tipo de dato de laser.
  TLaserData laserdata = laser_proxy->getLaserData();
  
  /*for ( int i = 0; i<laserdata.size(); i++){
    qDebug() << "Datos LaserData" << laserdata[i].dist << laserdata[i].angle; 
  }
  */
  try{
    
    for(auto i: laserdata){
      //qDebug() << "Datos LaserData" << i.dist << i.angle; 
      if(i.dist < 100){
	differentialrobot_proxy->stopBase();
      }
    }
  
  }catch(const Ice::Exception &e){
    std::cout<<e<<std::endl;
  }

  
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};