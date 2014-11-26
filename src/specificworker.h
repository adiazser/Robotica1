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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT

enum class STATE {CC, R, IR, P, AM, O, BB, AC, SC ,IDLE};
//enum class STATE {CC, C, R,IR, P, AM, O, IDLE};
STATE S;
QTime T;
float intervalo;
bool rotando=false;
bool marencontrada=false;
bool puntoencontrado=false;
bool enmarca=false;
int numMarca;
QVec marca;
QVec prm;
TBaseState basestate;
InnerModel *inner;
float angulochoque;
 TLaserData laserdata;


struct tag
{
  int id;
  float tx,ty,tz,ry,rz;	
  tag(){};
    tag( int id_, float tx_, float ty_, float tz_, float ry_, float rz_)
  {
      tx = tx_*1000; ty = ty_*1000; tz = tz_*1000; ry = ry_; id = id_; rz=rz_;
      pose.resize(6);
      pose[0] = tx;     pose[1] = ty;     pose[2] = tz;     pose[3] = 0;     pose[4] = ry;     pose[5] = rz;
  }
  QVec getPose()
  {
    return pose;
  }
  void print()
  {
    qDebug() << "	" << tx << ty << tz << ry;
  }
  QVec pose;
};

struct tagslocalT
{
  QMutex mutex;
  void update( const RoboCompGetAprilTags::listaMarcas &t)
  {
    QMutexLocker m(&mutex);
    tags.clear();
    for(auto i: t)
    {
      tag myT(i.id, i.tx, i.ty, i.tz, i.ry, i.rz);
      tags.push_back(myT);
    }
    
  }

  bool existsId(int id_, tag &tt)
  {
    QMutexLocker m(&mutex);
    for(auto i: tags)
      if( i.id == id_)
      {
	tt=i;
	return true;
      }
    return false;
  }
  void print()
  {
    qDebug() << "---------------PRINTING TAGS--------------";
    for(auto i: tags)
      i.print();
    qDebug() << "---------------END TAGS--------------";
  }
  std::vector<tag> tags;
};

tagslocalT tagslocal0, tagslocal1;


private:
	bool chocar();
	bool rotar();
	QVec fuerzasRepulsion();
	void iniciarrotar();
	void parar();
	bool avanzarMarca();
	void orientar();
	void controlador(const QVec& resultante, const QVec& target);
	void addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D);
	
	typedef std::pair<std::string, float> Tp;
	typedef std::vector<Tp> TPose;
	TPose poseCoger;
	TPose poseCaja;
	TPose poseChepa;
	TPose soltando;
	void ponerBrazo( const TPose &listaPose );
	void cogerCaja();
	void bajarBrazo();
	void agarrarCaja();
	void soltarCaja();
	void pasarCajaAMano();
	void pasarCajaASuelo();
	
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void  newAprilTag(const RoboCompGetAprilTags::listaMarcas& tags);

public slots:
 	void compute(); 	
};

#endif