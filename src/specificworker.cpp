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
#include <qt4/QtCore/QDate>

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
  S=STATE::I;
  T.restart();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
  
  switch(S){
    case STATE::I:
        iniciar();
      break;
    case STATE::A:
        avanzar();
      break;
    case STATE::C:
        chocar();
      break;
    case STATE::R:
        rotar();
      break;
    case STATE::IR:
        iniciarrotar();
      break;
  }
  
}

bool SpecificWorker::chocar()
{
   qDebug()<<__FUNCTION__;
   //laser_proxy es el puntero y tlaserdata es el tipo de dato de laser.
   differentialrobot_proxy->stopBase();
   T.restart();
   intervalo=qrand()*2200.f/RAND_MAX -1100;
   S=STATE::IR;
   
   return true;
}

bool SpecificWorker::rotar()
{
   
   qDebug()<<__FUNCTION__;
   if(T.elapsed()>intervalo){ 
     differentialrobot_proxy->setSpeedBase(0,0);
     S=STATE::I;
   }
   return true;
}

void SpecificWorker::iniciarrotar()
{
   qDebug()<<__FUNCTION__;
   differentialrobot_proxy->setSpeedBase(0,1);
   S=STATE::R;
}


bool SpecificWorker::avanzar()
{

  qDebug()<<__FUNCTION__;
  TLaserData laserdata = laser_proxy->getLaserData();
  
  try{
    for(auto i: laserdata){
      //qDebug() << "Datos LaserData" << i.dist << i.angle; 
      if(i.dist < 300){
	S=STATE::C;
	break;
      }
    }
  }catch(const Ice::Exception &e){
      std::cout<<e<<std::endl;
  }
  return true;
}


void SpecificWorker::iniciar()
{
  qDebug()<<__FUNCTION__;
  differentialrobot_proxy->setSpeedBase(500,0);
  S=STATE::A;
}



bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};