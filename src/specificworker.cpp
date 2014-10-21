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
  inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorld2.xml");
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
  try{
    differentialrobot_proxy->getBaseState(basestate);
  }
  catch(const Ice::Exception &e){	  std::cout<<e<<std::endl; }
  inner->updateTransformValues("base", basestate.x,0,basestate.z, 0, -basestate.alpha,0);
  
  qDebug()<< "BASESTATE" << basestate.x << basestate.z;
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
  static QVec memory=QVec::zeros(3);
  QVec r2(3);
  QVec imagine;
  
  if(tagslocal.existsId(3,t) )
  {
      QVec punto(2);
      punto[0]=0;
      punto[1]=500;
      memory = inner->transform("world", QVec::vec3(t.tx,0,t.tz), "camera");
      memory.print("memory");
      qDebug()<< "TZ" << t.tz;
      imagine=QVec::vec3(t.tx,0,t.tz);      
   }   
    else
    {
      imagine = inner->transform("base", memory, "world");
      imagine.print("imagine");
     }
  
      angulo=0.001*imagine.x();
      if (angulo>0.7)
	angulo=0.7;
      if (angulo < -0.7)
	angulo=-0.7;
      velocidad=0.5*imagine.z();
      if(velocidad>300){
	velocidad=300;
      if (angulo>0.7)
	angulo=0.7;
      if (angulo < -0.7)
	angulo=-0.7;
      }
    
      if(imagine.z() < 350)
      {
	S=STATE::P;
	marencontrada=false;
	return true;
      } try{
	differentialrobot_proxy->setSpeedBase(velocidad,angulo);
    }catch(const Ice::Exception &e){	  std::cout<<e<<std::endl;   } 

    return true;
 }
      //Qvec r = inner->transform("world", QVec::vec3(t.tx,0,t.tz), "camerargbd");
    
      /*Rot2D m(t.ry);
      Rot2DC mt(t.ry);
      QVec punto(2);
      punto[0]=0;
      punto[1]=500;
      QVec T= QVec::vec2(t.tx, t.tz);
      QVec f = m*(-(punto-T));
   //   f.print("f");
      puntoencontrado=true;
      marca=f;
      Rot2D mm(-basestate.alpha);
      qDebug()<<basestate.alpha;
      QVec puntom(2);
      puntom[0]=marca[0];
      puntom[1]=marca[1];
      QVec Tm= QVec::vec2(basestate.x, basestate.z);
      prm = mm*puntom + Tm;
      
     // Rot2D m2(-basestate.alpha);
    //  qDebug()<<basestate.alpha;
    //  QVec punto(2);
    //  QVec T2= QVec::vec2(basestate.x, basestate.z);
      QVec f2 = mm*prm + Tm;
    //QVec f= m*(punto-T);
      prm.print("prm: ");
      f2.print("resultado f");
    
      angulo=0.001*f2[0];
      if (angulo>0.8)
	angulo=0.8;
      if (angulo < -0.8)
	angulo=-0.8;
      velocidad=0.5*f2[1];
      if(velocidad>500)
	velocidad=500;
    
      if(f2[1]>-80 && f2[1]<80)
      {
	S=STATE::P;
	return true;
      } try{
	differentialrobot_proxy->setSpeedBase(velocidad,angulo);
    }catch(const Ice::Exception &e){	  std::cout<<e<<std::endl;      }
      
       * r[0] es la variable de giro el angulo
       * r[1] es la distancia a la pared 
       angulo=0.001*f[0];
       if (angulo>0.8)
	 angulo=0.8;
       velocidad=0.5*f[1];
       if(velocidad>500)
	 velocidad=500;
      if( f[1]>-1 && f[1]<1)
      {
	    S=STATE::P;
	    marencontrada=false;
	    return true;
      }
      
       try{
	differentialrobot_proxy->setSpeedBase(velocidad,angulo);
      }catch(const Ice::Exception &e){	  std::cout<<e<<std::endl;      }
      
  }
  
  else{
    //mundo 
    Rot2D m(-basestate.alpha);
    qDebug()<<basestate.alpha;
    QVec punto(2);
    punto[0]=prm[0];
    punto[1]=prm[1];
    QVec T= QVec::vec2(basestate.x, basestate.z);
    QVec f = m*punto - T;
    //QVec f= m*(punto-T);
    prm.print("prm: ");
    f.print("resultado f");
    
    angulo=0.001*f[0];
    if (angulo>0.8)
      angulo=0.8;
    velocidad=0.5*f[1];
    if(velocidad>500)
      velocidad=300;
    
    if(f[1]>-100 && f[1]<100)
    {
      S=STATE::P;
      return true;
     }
    try{
	differentialrobot_proxy->setSpeedBase(velocidad,angulo);
    }catch(const Ice::Exception &e){	  std::cout<<e<<std::endl;      }
    
    */


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