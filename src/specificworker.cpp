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
  S=STATE::IR;
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
    case STATE::P:
        parar();
      break;
  }
  
}

/*void SpecificWorker::compute2( )
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
*/

///OJO SE EJECUTA EN OTRO HILO

void SpecificWorker::newAprilTag(const tagsList& tags)
{
   qDebug()<<__FUNCTION__;
   tagslocal.clear();
   tagslocal=tags;
   for(auto i: tagslocal){
     i.tx=i.tx*1000.f;
     i.tz=i.tz*1000.f;
   }
}

bool SpecificWorker::chocar()
{
   float angulo;
   qDebug()<<__FUNCTION__;
   //laser_proxy es el puntero y tlaserdata es el tipo de dato de laser.
   differentialrobot_proxy->stopBase();
   T.restart();
   intervalo=qrand()*2200.f/RAND_MAX -1100;
   if(rotando==false){
      angulo=qrand()*2.f/RAND_MAX -1;
      if(angulo>=0)
	  angulo=1;
      else
	  angulo=-1;
   }
   S=STATE::IR;
   
   return true;
}

void SpecificWorker::parar()
{
  qDebug()<<__FUNCTION__;
  differentialrobot_proxy->setSpeedBase(0,0);
}


void SpecificWorker::iniciarrotar()
{
   qDebug()<<__FUNCTION__;
   if(T.elapsed()>intervalo){ 
     differentialrobot_proxy->setSpeedBase(0,1);
     S=STATE::R;
   }
}

bool SpecificWorker::rotar()
{
   float angulo, velocidad;
   qDebug()<<__FUNCTION__;
   for(auto i: tagslocal){
     if(i.id==3){
       qDebug()<< i.tz ;
       if(i.tz*1000.f<600){
	 S=STATE::P; 
	 return true;
       }
       // en vez de uno son dos constantes p y q, p para el angulo y q para la velocidad pero no se cuales son.
       angulo=1.f*i.tx;
       velocidad=100.f*i.tz;
  
       try{
	differentialrobot_proxy->setSpeedBase(velocidad,angulo);
      }catch(const Ice::Exception &e){
	  std::cout<<e<<std::endl;
      }
      
     }
   }
      
  return true;
}


bool SpecificWorker::avanzar()
{

  qDebug()<<__FUNCTION__;
  TLaserData laserdata = laser_proxy->getLaserData();
  
  try{
    for(auto i: laserdata){
      //qDebug() << "Datos LaserData" << i.dist << i.angle; 
      if(i.dist < 500 && i.angle<1.2 && i.angle >-1.2){
	S=STATE::C;
	break;
      }else{
	S=STATE::I;
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
  rotando=false;
}



bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};