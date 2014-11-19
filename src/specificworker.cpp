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
  
 
  
  poseAndar.push_back(std::make_pair<std::string,float>("shoulder_right_1", 3));
  poseAndar.push_back(std::make_pair<std::string,float>("elbow_right", 1)); 
  poseAndar.push_back(std::make_pair<std::string,float>("wrist_right_2", 1.1));
  poseAndar.push_back(std::make_pair<std::string,float>("wrist_right_giro", 0.0));
  
  poseCoger.push_back(std::make_pair<std::string,float>("shoulder_right_1", 0));
  poseCoger.push_back(std::make_pair<std::string,float>("shoulder_right_2", -0.7));
  poseCoger.push_back(std::make_pair<std::string,float>("elbow_right", 1)); 
  poseCoger.push_back(std::make_pair<std::string,float>("wrist_right_giro", 0));
  poseCoger.push_back(std::make_pair<std::string,float>("wrist_right_2", 1.1));
  poseCoger.push_back(std::make_pair<std::string,float>("finger_right_1", 0.0));
  poseCoger.push_back(std::make_pair<std::string,float>("finger_right_2", 0.0));
  
  ponerBrazo( poseAndar );
  timer.start(Period);
  
  //S=STATE::IDLE;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}


void SpecificWorker::compute( )
{ 
  static QTime reloj= QTime::currentTime();
  static QTime reloj2= QTime::currentTime();
  try
  {
    differentialrobot_proxy->getBaseState(basestate);
    laserdata = laser_proxy->getLaserData();
  }
  catch(const Ice::Exception &e){	  std::cout<<e<<std::endl; }
  inner->updateTransformValues("robot", basestate.x,0,basestate.z, 0, basestate.alpha,0);
  
  
  try
  {
    tagslocal0.update(getapriltags0_proxy->checkMarcas());
    //tagslocal0.print();
    tagslocal1.update(getapriltags1_proxy->checkMarcas());    
    //tagslocal1.print();
  }
  catch(const Ice::Exception &e){	  std::cout<<e<<std::endl; }
  
  /* try
  {
    RoboCompJointMotor::MotorStateMap motorMap;
    jointmotor_proxy->getAllMotorState(motorMap);
  }
  catch(const Ice::Exception &e){	  std::cout<<e<<std::endl; }
  */
  
//   readBaseState();
//   readTags();
//   readArmState();
  
  switch(S){
   /* case STATE::C:
        chocar();
      break;
      */
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
        reloj2.restart();
        avanzarMarca();
	 qDebug()<< "Reloj AVANZAR" << reloj2.restart();
      break;
    case STATE::O:
        orientar();
      break;
    case STATE::CC:
       cogerCaja();
       break;
    case STATE::IDLE:
      break;
  }
  qDebug()<< reloj.restart();
  
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





/*bool SpecificWorker::chocar()
{
   
   qDebug()<<__FUNCTION__;
   //laser_proxy es el puntero y tlaserdata es el tipo de dato de laser.
   differentialrobot_proxy->stopBase();
   T.restart();
   intervalo=qrand()*2200.f/RAND_MAX -1100;
   if(rotando==false){
      angulochoque=qrand()*2.f/RAND_MAX -1;
      if(angulochoque>=0)
	  angulochoque=1;
      else
	  angulochoque=-1;
   }
   S=STATE::IR;
   
   return true;
}
*/


void SpecificWorker::iniciarrotar()
{
   qDebug()<<__FUNCTION__;
   try
   {
      differentialrobot_proxy->setSpeedBase (0, 0.5);
      S=STATE::R;
   } catch ( const Ice::Exception &E )
     { 
	std::cout<< E << endl; 
     }
}

bool SpecificWorker::rotar()
{
  tag t;
  if(tagslocal0.existsId(10,t))
  {
    S=STATE::P;
    if(enmarca==false){
      marencontrada=true;
    }
    else{
      S=STATE::O; 
    }
    
    return true;
  }
  return false;
}


void SpecificWorker::parar()
{
  qDebug()<<__FUNCTION__;
  differentialrobot_proxy->setSpeedBase(0,0);
  ponerBrazo( poseCoger );
  if(marencontrada==true){
    S=STATE::AM;
  }
  if(enmarca==true && marencontrada==false){
    S=STATE::O;
  }
  
}

bool SpecificWorker::avanzarMarca()
{
  qDebug()<<__FUNCTION__;
  tag t;
  static QVec memory=QVec::zeros(3);
  QVec r2(3);
  QVec imagine;
  
  if(tagslocal0.existsId(10,t) && tagslocal1.existsId(10 , t) ){
    	S=STATE::P;
	marencontrada=false;
	enmarca=true;
    return true;
  }
  else if(tagslocal1.existsId(10, t)){
    	S=STATE::P;
	marencontrada=false;
	enmarca=true;
    return true;
  }
  else if(tagslocal0.existsId(10,t) )
  {
      QVec punto(3);
      punto[0]=0;
      punto[1]=0;
      punto[2]=0;
      
      addTransformInnerModel("marca-desde-camara", "camera", t.getPose());
      memory = inner->transform("world", punto, "marca-desde-camara");
      imagine= inner->transform("camera", punto, "marca-desde-camara");
      //imagine.print("PUNTO VISTO DESDE LA CAMARA IF");
    }   
    else
    {
      imagine = inner->transform("camera", memory, "world");
      //imagine.print("PUNTO VISTO DESDE LA CAMARA ELSE");
     }
     
     QVec fuerzas = fuerzasRepulsion();
     QVec resultante = fuerzas + QVec::vec2(imagine.x(),imagine.z());
     controlador(resultante, imagine);
     
    
    return true;
 }
 
void SpecificWorker::controlador(const QVec &resultante, const QVec &target)
{
      qDebug()<<__FUNCTION__;
      
      float angulo = atan2(resultante.x(),resultante.y());
      if (angulo>0.7){
	angulo=0.7;
      }else if (angulo< -0.7)
	angulo =-0.7;
      angulochoque=angulo;
      float velocidad=0.2*resultante.norm2();
      if(velocidad <50){
	  velocidad=50;
      }
      if(velocidad >200){
	 velocidad=200;
      }
      
     if (target.z() < 50)
      {
	S=STATE::P;
	marencontrada=false;
	enmarca=true;
      } 
      
       try{
 	differentialrobot_proxy->setSpeedBase(velocidad,angulo);
       }catch(const Ice::Exception &e){	  std::cout<<e<<std::endl;   } 

}


void SpecificWorker::orientar()
{

  qDebug()<<__FUNCTION__;
  tag t;
  
      if(rotando==false){
	if (angulochoque <0){
	   angulochoque = 0.2;
	   rotando=true;
	}
	else{
	  angulochoque=-0.2;
	  rotando=true;
	}
      }
  
  differentialrobot_proxy->setSpeedBase(0,angulochoque);
  if(tagslocal1.existsId(10,t) )
  {
   // qDebug()<< "TX" << t.tx;
    if(t.tx<70 && t.tx>-70){
      differentialrobot_proxy->setSpeedBase(0,0);
      enmarca=false;
      rotando=false;
      //S=STATE::IDLE;
      S=STATE::CC;
    }
    
  }
  
}

void SpecificWorker::cogerCaja()
{
  qDebug()<<__FUNCTION__;
  tag t;
  if(tagslocal1.existsId(10,t) )
  {
    
    if(fabs(t.tx)>10){
      Axis ax;
      ax.x= 1; 
      ax.y= 0;
      ax.z= 0;
      qDebug() << "enviado al brazo a x";
      bodyinversekinematics_proxy->advanceAlongAxis("ARM", ax, 0.8*t.tx);
      sleep(2);
    }
    else if (fabs(t.ty)>30)
    {
      Axis ax;
      ax.x= 0; 
      ax.y= 1;
      ax.z= 0;
      qDebug() << "enviado al brazo en Y";
      bodyinversekinematics_proxy->advanceAlongAxis("ARM", ax, 0.8*t.ty);
      sleep(2);
    }
    else
    {      
     S=STATE::IDLE; 
    }
  }
}


void SpecificWorker::addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D)
{
		InnerModelNode *nodeParent = inner->getNode(parent);
		if( inner->getNode(name) == NULL)
		{
			InnerModelTransform *node = inner->newTransform(name, "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
			nodeParent->addChild(node);
		}
		inner->updateTransformValues(name, pose6D.x(), pose6D.y(), pose6D.z(), pose6D.rx(), pose6D.ry(), pose6D.rz());	
}

 
QVec SpecificWorker::fuerzasRepulsion()
{

  //qDebug()<<__FUNCTION__;
  QVec fuerza;
  QVec expulsion = QVec::zeros(2);
  try{
    
    for(auto i: laserdata)
    {
      if (i.angle>-1.5 and i.angle< 1.5)
      {
	fuerza = QVec::vec2(-sin(i.angle) * i.dist, -cos(i.angle) * i.dist);
	float mod = fuerza.norm2();
	if (i.dist < 1000){
	  expulsion = expulsion + (fuerza.normalize() * (float)(1.0/(mod))); 
	}
	
      }
    }
    
  }catch(const Ice::Exception &e){
      std::cout<<e<<std::endl;
  }
  return expulsion*100000;
}

void SpecificWorker::ponerBrazo(const std::vector< std::pair<std::string, float> > & listaPose)
{
  try
  {
    RoboCompJointMotor::MotorGoalPosition mg;
    for(auto i : listaPose)
    {  
      mg.name = i.first;
      mg.position = i.second;
      mg.maxSpeed = 1.f;
      jointmotor_proxy->setPosition( mg );
    }
  }
  catch(const Ice::Exception &ex)
  { std::cout << ex << std::endl;}
}



bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	
	return true;
};
 
