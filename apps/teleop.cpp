#include "mrcore/mrcore.h"


#include <iostream>
#include "glutapp.h"
#include "reactivecontrol.h"
#include "trajcontrol.h"

using namespace mr;
using namespace std;

class MyGlutApp: public GlutApp
{
public:
	MyGlutApp(string name,MobileRobot* r):GlutApp(name),robot(r)
	{
		world+=robot;
		scene.addObject(&world);
		scene.SetViewPoint(35,160,25);	
		va=vg=0;

		float x=-7,y=-5;
		vector<Vector2D> path;
		path.push_back(Vector2D(x,y));
		path.push_back(Vector2D(x+20,y+0));
		path.push_back(Vector2D(x+20,y+10));
		path.push_back(Vector2D(x,y+10));
		path.push_back(Vector2D(x,y));
		traj.setPath(path);

		//path Building.world
		/*path.push_back(Vector2D(8,8));
		path.push_back(Vector2D(8,1));
		path.push_back(Vector2D(1,1));
		path.push_back(Vector2D(1,8));
		//Primer piso
		path.push_back(Vector2D(8,8));
		path.push_back(Vector2D(8,1));
		path.push_back(Vector2D(1,1));
		path.push_back(Vector2D(1,8));
		//Segundo piso
		path.push_back(Vector2D(8,8));
		path.push_back(Vector2D(8,1));
		path.push_back(Vector2D(1,1));
		path.push_back(Vector2D(1,8));
		//Tercer piso
		path.push_back(Vector2D(8,8));*/

		//path DisamLab.world
		/*float x=6.0, y=1.5;
		path.push_back(Vector2D(6-x,1.5-y));
		path.push_back(Vector2D(8-x,1.5-y));
		path.push_back(Vector2D(8.3-x,4-y));
		path.push_back(Vector2D(8.3-x,3-y));
		path.push_back(Vector2D(6-x,2.5-y));
		path.push_back(Vector2D(6-x,1.5-y));*/


		manual=true;
		robot->startLogging("log/disamlog1");
	}
/*	bool reactiveControl(LaserData laser,double distMin=0.4)
	{
		vector<double> ranges=laser.getRanges();
		for(int i=0;i<ranges.size();i++)
		{
			if(ranges[i]<distMin)
				return false;
		}
		return true;
	}
	void modeAutomatic(Pose2D& robotPose, float& sp, float& rt)
	{
		Vector2D error=path.at(0)-robotPose.position();
		double angle=error.argument();
		//standardization of angles between -PI y +PI
		if(robotPose.theta>PI)
			robotPose.theta=robotPose.theta-2*PI;
		else if(robotPose.theta<-PI)
			robotPose.theta=robotPose.theta+2*PI;
		if(angle>PI)
			angle-=2*PI;
		else if (angle<-PI)
			angle+=2*PI;
		double angDiff=angle-robotPose.theta.getValue();
		if(angDiff>PI)
			angDiff-=2*PI;
		else if (angDiff<-PI)
			angDiff+=2*PI;

		//Master and slave control.
		if (abs(angDiff)>=2*DEG2RAD)	//too much error in orientation
		{
			sp=0.0;
			if(angle>robotPose.theta.getValue())
				rt=0.1;
			else
				rt=-0.1;
		}
		else
		{
			rt=0.5*angDiff;
			cout<<error.module()<<endl;
			if(error.module()>0.1)	//too much error in distance
			{
				sp=0.1;
			}
			else	//Near final point
			{
				//First point of the path is going to be erased
				path.erase(path.begin(),path.begin()+1);
				if(path.size()==0)	//list is empty
				{
					rt=0;
					sp=0;
					manual=true;
				}
			}

		}
	}*/
	void Draw(void)
	{
		scene.Draw();
		traj.drawGL();
		control.drawGL();
	}
	void Timer(float time)
	{
		Odometry odom;
		LaserData laserData;

		robot->getOdometry(odom);
		Pose3D realPose;
		robot->getPose3D(realPose);
		robot->getLaserData(laserData);

		//The odometry is full 3D, lets handle it only in 2D, as a Pose (x, y, theta)
		Transformation3D pose=odom.pose;
		double roll,pitch,yaw;
		pose.orientation.getRPY(roll,pitch,yaw);
		Pose2D robotPose(pose.position.x,pose.position.y,yaw);

		if(manual)
			robot->move(va,vg);
		else
		{
			traj.setData(robotPose);
			traj.getSpeed(va,vg);

			control.setCommand(va,vg);
			control.setData(laserData);
			float va2=va,vg2=vg;
			control.getSpeed(va2,vg2);	

			robot->move(va2,vg2);
		}
	}
	void Key(unsigned char key)
	{
		if(key=='m')
			manual=!manual;
		else if(key=='a')
			vg+=0.05;
		else if(key=='d')
			vg-=0.05;
		else if(key=='s')
			va-=0.05;
		else if(key=='w')
			va+=0.05;
		else 
		{
			va=vg=0;
		}
	}
	void MouseMove(int x,int y)
	{
		scene.MouseMove(x,y);
		glutPostRedisplay();
	}
	void MouseClick(int b,int state, int x,int y)
	{
		bool down=(state==GLUT_DOWN);
		int button;
		if(b==GLUT_LEFT_BUTTON)
			button=MOUSE_LEFT_BUTTON;
		if(b==GLUT_RIGHT_BUTTON)
			button=MOUSE_RIGHT_BUTTON;
			
		int specialKey = glutGetModifiers();
		bool ctrlKey= (specialKey & GLUT_ACTIVE_CTRL)?true:false ;
		bool sKey= specialKey&GLUT_ACTIVE_SHIFT ;
		
		scene.MouseButton(x,y,b,down,sKey,ctrlKey);
		glutPostRedisplay();
	}
private:
	bool manual;
	float vg,va;
	GLScene scene;
	World world;
	MobileRobot* robot;
	ReactiveControl control;
	TrajControl traj;
};

void printUsage();

int main(int argc,char* argv[])
{
/*	if(argc!=2)
	{
		printUsage();
		return -1;
	}
	string configFile(argv[1]);

	int port=-1;
	if(robotname=="nemo")
		robot=new Nemo;*/
	mrcoreInit();
	MobileRobot* robot=new Nemo();
	robot->connectClients("192.168.0.150",13000);
//	robot->connectLog("log/columns");
	MyGlutApp myApp("teleop",robot);
	myApp.Run();
	return 0;   
}

void printUsage()
{
	cout<<"-------- Usage -----------------"<<endl;
	cout<<"> teleop config.txt    "<<endl;
	cout<<"example:    "<<endl;
	cout<<"> teleop neo 127.0.0.1 13000    "<<endl;
}