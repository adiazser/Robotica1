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
#include </home/salabeta/robocomp/components/robocomp-robolab/components/apriltagsComp/src/specificworker.h>
#include <qt4/QtCore/QDate>
 
/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
  
  S=STATE::IR;
  inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorld.xml");
  T.restart();
  
  vectorCajas[0]=10;
  vectorCajas[1]=11;
  vectorCajas[2]=23;
  numMarca=vectorCajas[icajas];
  
  poseCoger.push_back(std::make_pair<std::string,float>("shoulder_right_1", 0));
  //poseCoger.push_back(std::make_pair<std::string,float>("shoulder_right_2", -0.7));
 // poseCoger.push_back(std::make_pair<std::string,float>("elbow_right", 1)); 
  poseCoger.push_back(std::make_pair<std::string,float>("shoulder_right_2", -0.9));
  poseCoger.push_back(std::make_pair<std::string,float>("elbow_right", 1.05)); 
  poseCoger.push_back(std::make_pair<std::string,float>("wrist_right_giro", 0));
  poseCoger.push_back(std::make_pair<std::string,float>("wrist_right_2", 1.1));
  poseCoger.push_back(std::make_pair<std::string,float>("finger_right_1", 0.0));
  poseCoger.push_back(std::make_pair<std::string,float>("finger_right_2", 0.0));
  
  
  poseCaja.push_back(std::make_pair<std::string, float>("shoulder_right_1", 0));
  poseCaja.push_back(std::make_pair<std::string, float>("shoulder_right_2", -0.7));
  poseCaja.push_back(std::make_pair<std::string,float>("elbow_right", 1));
  poseCaja.push_back(std::make_pair<std::string, float>("wrist_right_2", 1.3));
  poseCaja.push_back(std::make_pair<std::string, float>("wrist_right_giro", 0));
  
  poseChepa.push_back(std::make_pair<std::string, float>("shoulder_right_1", 3));
  poseChepa.push_back(std::make_pair<std::string, float>("shoulder_right_2", -0.7));
  poseChepa.push_back(std::make_pair<std::string,float>("elbow_right", 1));
  poseChepa.push_back(std::make_pair<std::string, float>("wrist_right_2", 1.3));
  poseChepa.push_back(std::make_pair<std::string, float>("wrist_right_giro", 0));
  
  
  soltando.push_back(std::make_pair<std::string, float>("finger_right_1", 0));
  soltando.push_back(std::make_pair<std::string, float>("finger_right_2", 0));
  
  
  ponerBrazo( poseChepa );
  timer.start(Period);
  
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}


void SpecificWorker::compute( )
{ 
	
  try
  {
    differentialrobot_proxy->getBaseState(basestate);
    laserdata = laser_proxy->getLaserData();
  }
  catch(const Ice::Exception &e){ std::cout<< "en diff" <<e<<std::endl; }
  inner->updateTransformValues("robot", basestate.x,0,basestate.z, 0, basestate.alpha,0);
  
  
  try
  {
    tagslocal0.update(getapriltags0_proxy->checkMarcas());
    tagslocal1.update(getapriltags1_proxy->checkMarcas());    
  }
  catch(const Ice::Exception &e){std::cout<< "error en april " <<e<<std::endl; }
  
  
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
        avanzarMarca();
      break;
    case STATE::O:
        orientar();
      break;
	case STATE::OM:
		orientarMarca();
		break;
    case STATE::CC:
       cogerCaja();
       break;
	case STATE::BB:
		bajarBrazo();
		break;
	case STATE::AC:
		 agarrarCaja();
		 break;
	case STATE::SC:
		soltarCaja();
		break;
    case STATE::IDLE:
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
  qDebug()<<__FUNCTION__;
  tag t;
  if(tagslocal0.existsId(numMarca,t))
  {
    S=STATE::P;
    if(enmarca==false){
	  qDebug()<<"CAMBIO A MARCA "<< numMarca;
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
  if(numMarca>=10)
	 ponerBrazo( poseCoger );
     sleep(2);
  if(marencontrada==true){
    S=STATE::AM;
  }
  if(enmarca==true && marencontrada==false){
	  if(numMarca<10){
		  S=STATE::OM; 
			
		}else{
			S=STATE::O;
		}
  }
  
}

bool SpecificWorker::avanzarMarca()
{
  qDebug()<<__FUNCTION__;
  tag t;
  static QVec memory=QVec::zeros(3);
  QVec r2(3);
  QVec imagine;
  
  if(tagslocal0.existsId(numMarca,t) && tagslocal1.existsId(numMarca , t) ){
    	S=STATE::P;
	marencontrada=false;
	enmarca=true;
    differentialrobot_proxy->setSpeedBase(0,0);
	return true;
  }
  else if(tagslocal1.existsId(numMarca, t)){
    	S=STATE::P;
	marencontrada=false;
	enmarca=true;
    differentialrobot_proxy->setSpeedBase(0,0);
	return true;
  }
  else if(tagslocal0.existsId(numMarca,t) )
  {
	  qDebug()<<"MARCAA "<< t.id;
	  QVec punto(3);
      punto[0]=0;
      punto[1]=0;
	  punto[2]=0;
	  
      addTransformInnerModel("marca-desde-camara", "camera", t.getPose());
      memory = inner->transform("world", punto, "marca-desde-camara");
      imagine= inner->transform("camera", memory, "world");
    }   
    else
    {
      imagine = inner->transform("camera", memory, "world");
     }
     
     QVec fuerzas = fuerzasRepulsion(imagine);
     controlador(fuerzas, imagine);
     
    
    return true;
 }
 
bool SpecificWorker::controlador(const QVec &resultante, const QVec &target)
{
      qDebug()<<__FUNCTION__;
      
      float angulo = atan2(resultante.x(),resultante.y());
	  
		if (angulo>0.4){
			angulo=0.4;
		}else if (angulo< -0.4)
			angulo =-0.4;
      angulochoque=angulo;
      float velocidad=0.2*resultante.norm2();
	  
      if(velocidad >150){
		  if(target.z()<1000){
			  velocidad=50;
		  }
		  if(target.z()<500){
			  velocidad=0;  
		  }
		  else
			velocidad=150;
      }
	   if(numMarca>=10){
			if (fabs(target.z()) < 650)
			{
		 
				S=STATE::P;
				marencontrada=false;
				enmarca=true;
				differentialrobot_proxy->setSpeedBase(0,0);
				return true;
			} 
	   }
	   else{
		   if(fabs(target.z())<800){
		   
			   S=STATE::P;
			marencontrada=false;
			enmarca=true;
			differentialrobot_proxy->setSpeedBase(0,0);
			return true;
		}
	}
      
       try{
 	differentialrobot_proxy->setSpeedBase(velocidad,angulo);
       }catch(const Ice::Exception &e){	  std::cout<<e<<std::endl;   } 
       
  return true;
}

QVec SpecificWorker::fuerzasRepulsion(const QVec & objectivo)
{
	
  QVec fuerza;
  QVec expulsion = QVec::zeros(2);
  try{
    
    for(auto i: laserdata)
    {
      if (i.angle>-1.5 and i.angle< 1.5)
      {
	fuerza = QVec::vec2(-sin(i.angle) * i.dist, -cos(i.angle) * i.dist);
	float mod = fuerza.norm2();
	if(objectivo.norm2()>700){
		if (i.dist < 550){
			expulsion = expulsion + (fuerza.normalize() * (float)(1.0/(mod))); 
		}
	}
	
	
	}
	
      }
  }catch(const Ice::Exception &e){
      std::cout<<e<<std::endl;
  }
  expulsion=expulsion*100000;
  
  QVec resultante = expulsion + QVec::vec2(objectivo.x(),objectivo.z());
  
  return resultante;
}

void SpecificWorker::orientar()
{

  qDebug()<<__FUNCTION__;
  tag t;
      if(rotando==false){
	if (angulochoque < 0){
	   angulochoque = 0.2;
	   rotando=true;
	}
	else{
	  angulochoque=-0.2;
	  rotando=true;
	}
      }
  differentialrobot_proxy->setSpeedBase(0,angulochoque);
  if(tagslocal1.existsId(numMarca,t) )
  {
		if(fabs(t.tx)<130){
		differentialrobot_proxy->setSpeedBase(0,0);
		enmarca=false;
		rotando=false;
		S=STATE::CC;
		}
    
  }
  
}


void SpecificWorker::orientarMarca()
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
      
  qDebug()<< "NUMERO MARCA ORIENTAR" << numMarca;
  differentialrobot_proxy->setSpeedBase(0,angulochoque);
  if(tagslocal0.existsId(numMarca,t) )
  {
	  
		 differentialrobot_proxy->setSpeedBase(0,0);
		 ponerBrazo(poseCaja);
		 sleep(2);
		 S=STATE::SC;
  }
  
}


void SpecificWorker::cogerCaja()
{
  qDebug()<<__FUNCTION__;
  tag t;
  if(tagslocal1.existsId(numMarca,t) )
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
	  if(fabs(t.rz)!=0){
		   qDebug() << "Ajustando el giro";
		MotorGoalPosition posicion;
		try 
		{
			posicion.name = "wrist_right_giro";
			posicion.position = t.rz;
			posicion.maxSpeed = 1.f;
			jointmotor_proxy->setPosition(posicion);
			sleep(1);
			S=STATE::BB;
			//NOS VA MAL LAS FUERZAS POR ESO TENGO COMENTADO QUE VAYA A BAJAR CAJA !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			//SI SE DESCOMENTA TODO VA BIEN PERO ALGUNA VEZ ENCONTRARA LA CAJA BIEN OTRAS NO!!!!!!!!!!!!!!!!!!!!!!!!!!!
			//S=STATE::IDLE;
		}catch(Ice::Exception &ex){std::cout << ex << std::cout;}
	  }
    }
  }
}


void SpecificWorker::soltarCaja()
{
	 qDebug()<<__FUNCTION__;
	 tag t;
	 //TRANSFORMACION PORQUE COMO NO VE LA CAJA CON EL TAGS NO SE PUEDE
	 //ENTONCES TRANSFORMAMOS DEL BRAZO QUE ES EL PUNTO DONDE ESTA LA CAJA COGIDA
	 vectorSuelo = inner->transform("world", "arm_right_7");
	    try{
		Axis ax;
		ax.x= 0; 
		ax.y= 0;
		ax.z= 1;
		qDebug() << "Bajando Brazo Para soltar Caja";
		//VECTORSUELO[1] por que queremos bajar el brazo
		bodyinversekinematics_proxy->advanceAlongAxis("ARM", ax, vectorSuelo[1]-80);
		sleep(2);
		}
		catch( const Ice::Exception &ex){std::cout << ex << std::endl;}
		
	ponerBrazo(soltando);
	sleep(1);
	pasarCajaASuelo();
	qDebug() << "Caja Soltada";
	sleep(2);
	ponerBrazo(poseChepa);
	sleep(2);
	icajas++;
	numMarca=vectorCajas[icajas];
	if(icajas>2){
		S = STATE::IDLE;
		qDebug()<<"FINISH";
		
	}
	else{
		enmarca=false;
		marencontrada=false;
		S = STATE::IR;
	}
}


	
void SpecificWorker::bajarBrazo()
{
	 qDebug()<<__FUNCTION__;
	 tag t;
	 if(tagslocal1.existsId(numMarca,t) )
	 {	
		try{
	    Axis ax;
		ax.x= 0; 
		ax.y= 0;
		ax.z= 1;
		qDebug() << "Bajando el brazo";
		bodyinversekinematics_proxy->advanceAlongAxis("ARM", ax, t.tz-200);
		sleep(2);
		S=STATE::AC;
		}
		catch( const Ice::Exception &ex){std::cout << ex << std::endl;}
	}
}



void SpecificWorker::agarrarCaja()
{
	qDebug()<<__FUNCTION__;
	try{
		bodyinversekinematics_proxy->setFingers(90.0f);
	}
	catch( const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
	sleep(1);
	pasarCajaAMano();
	ponerBrazo( poseCaja );
	sleep(2);
	ponerBrazo( poseChepa );
	sleep(2);
	marcaMano=numMarca;
	//MARCA DE LA PARED A LA QUE LO QUEREMOS LLEVAR	
	numMarca=3;
	S = STATE::IR;
	
}

//caja es el id (string) de la caja que se va a mover. Las cajas se llaman: "C10" "C11" etc
void SpecificWorker::pasarCajaAMano()
{
    try{
        QString marca = "C" + QString::number(numMarca,10);
        string marcaString = marca.toStdString();
        innermodelmanager_proxy->removeNode(marcaString);
        Pose3D posCaja;
        posCaja.x = -25; posCaja.y = 0; posCaja.z = 80; posCaja.rx = 0; posCaja.ry = 0; posCaja.rz = 0;
        innermodelmanager_proxy->addTransform(marcaString, "static", "arm_right_7", posCaja);
        Plane3D planoCaja;
        planoCaja.px = 0; planoCaja.py = 0; planoCaja.pz = 0; planoCaja.nx = 0; planoCaja.ny = 0; planoCaja.nz = 0;
        planoCaja.width = 100; planoCaja.height =100; planoCaja.thickness = 100;
        QString textura = "/home/robocomp/robocomp/files/innermodel/tar36h11-"+QString::number(numMarca,10)+".png";
        string texturaString = textura.toStdString();
        planoCaja.texture = texturaString;
        QString marcaPlano = "Plano" + marca;
        string marcaPlanoString = marcaPlano.toStdString();
        innermodelmanager_proxy->addPlane(marcaPlanoString, marcaString, planoCaja);
    }
    catch (Ice::Exception &ex)
    {std::cout << ex << std::cout;}
}

// El vector suelo se saca de la siguiente transformación 'vectorSuelo = inner->transform("world", "arm_right_7");'

void SpecificWorker::pasarCajaASuelo()
{
    try{
        QString cajaCogida = "C" + QString::number(marcaMano,10);
        string cajaCogidaString = cajaCogida.toStdString();
        innermodelmanager_proxy->removeNode(cajaCogidaString);
        Pose3D posCaja;
        posCaja.x = vectorSuelo[0]; posCaja.y = 50; posCaja.z = vectorSuelo[2]; posCaja.rx = 0; posCaja.ry = 0; posCaja.rz = 0;
        innermodelmanager_proxy->addTransform(cajaCogidaString, "static", "world", posCaja);
        Plane3D planoCaja;
        planoCaja.px = 0; planoCaja.py = 0; planoCaja.pz = 0; planoCaja.nx = 0; planoCaja.ny = 0; planoCaja.nz = 0;
        planoCaja.width = 100; planoCaja.height =100; planoCaja.thickness = 100;
        QString textura = "/home/robocomp/robocomp/files/innermodel/tar36h11-"+QString::number(marcaMano,10)+".png";
        string texturaString = textura.toStdString();
        planoCaja.texture = texturaString;
        QString cajaCogidaPlano = "Plano" + cajaCogida;
        string cajaCogidaPlanoString = cajaCogidaPlano.toStdString();
        innermodelmanager_proxy->addPlane(cajaCogidaPlanoString, cajaCogidaString, planoCaja);
    }
    catch (Ice::Exception &ex)
    {std::cout << ex << std::cout;}
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
 
