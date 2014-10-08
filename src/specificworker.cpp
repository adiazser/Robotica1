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
    case STATE::AM:
        avanzarMarca();
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
   //qDebug()<<__FUNCTION__;
   tagslocal.update(tags);
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

void SpecificWorker::iniciarrotar()
{
   qDebug()<<__FUNCTION__;
   try
   {
      differentialrobot_proxy->setSpeedBase (0, 1);
      S=STATE::R;
   } catch ( const Ice::Exception &E )
     { 
	std::cout<< E << endl; 
     }
}

bool SpecificWorker::rotar()
{
  tag t;
  if(tagslocal.existsId(3,t))
  {
    S=STATE::P;
    marencontrada=true;
    return true;
  }
  return false;
}


void SpecificWorker::parar()
{
  qDebug()<<__FUNCTION__;
  differentialrobot_proxy->setSpeedBase(0,0);
  if(marencontrada==true){
    S=STATE::AM;
  }
  
}

bool SpecificWorker::avanzarMarca()
{
  qDebug()<<__FUNCTION__;
  float angulo, velocidad;
  tag t;
  if(tagslocal.existsId(3,t))
  {
      qDebug()<< t.tz ;
      
      
      
      
      
      Rot2D m(t.ry);
      m.print("m");
      QVec punto(2);
      punto[0]=0;
      punto[1]=600;
      QVec T= QVec::vec2(t.tx, t.tz);
      QVec r = m*punto + T;
      r.print("r");
      //qFatal(".");
      
      if(r[1]<0)
      {
	 S=STATE::P; 
	 marencontrada=false;
	 return true;
       }
      
      
       angulo=0.001*r[0];
       if (angulo>0.8)
	 angulo=0.8;
       velocidad=0.5*r[1];
       if(velocidad>500)
	 velocidad=500;
       try{
	differentialrobot_proxy->setSpeedBase(velocidad,angulo);
      }catch(const Ice::Exception &e){	  std::cout<<e<<std::endl;      }
  }
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