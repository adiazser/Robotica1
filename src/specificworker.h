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

enum class STATE {FR, C, R, I, IR, P, AM, O, IDLE};
STATE S;
QTime T;
float intervalo;
bool rotando=false;
bool marencontrada=false;
bool puntoencontrado=false;
bool enmarca=false;
QVec marca;
QVec prm;
TBaseState basestate;
InnerModel *inner;
float angulochoque;

 


struct tag
{
  int id;
  float tx,ty,tz,ry;	
  tag(){};
    tag( int id_, float tx_, float ty_, float tz_, float ry_)
  {
      tx = tx_*1000; ty = ty_*1000; tz = tz_*1000; ry = ry_; id = id_;
      pose.resize(6);
      pose[0] = tx;     pose[1] = ty;     pose[2] = tz;     pose[3] = 0;     pose[4] = ry;     pose[5] = 0;
  }
  QVec getPose()
  {
    return pose;
  }
  QVec pose;
};

struct tagslocalT
{
  QMutex mutex;
  void update( const tagsList &t)
  {
    QMutexLocker m(&mutex);
    tags.clear();
    for(auto i: t)
    {
      tag myT(i.id, i.tx, i.ty, i.tz, i.ry);
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
  std::vector<tag> tags;
};

tagslocalT tagslocal;


private:
	bool chocar();
	bool rotar();
	QVec fuerzasRepulsion();
	void iniciar();
	void iniciarrotar();
	void parar();
	bool avanzarMarca();
	void orientar();
	void controlador(const QVec& resultante, const QVec& target);
	void addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D);
	
	typedef std::pair<std::string, float> Tp;
	typedef std::vector<Tp> TPose;
	 TPose poseAndar;
    TPose poseCoger;
	void ponerBrazo( const TPose &listaPose );
	
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void  newAprilTag(const tagsList& tags);

public slots:
 	void compute(); 	
};

#endif