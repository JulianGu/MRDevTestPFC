#include "mrcore/mrcore.h"

void CreateEmptyWorld(string filename)
{
	World world;
	FaceSetPart *building=new FaceSetPart;
	Face ground;
	ground.setBase(Transformation3D(0,0,0));
	ground.addVertex(0,0);
	ground.addVertex(20.0,0);
	ground.addVertex(20.0,20.0);
	ground.addVertex(0,20.0);
	ground.setColor(0.6f, 0.4f, 0.4f, 1.0f);
	building->addFace(ground);

	Face wall1;
	wall1.setBase(Transformation3D(0,0,0,X_AXIS,PI/2));
	wall1.addVertex(0,0);
	wall1.addVertex(20.0,0);
	wall1.addVertex(20.0,2.0);
	wall1.addVertex(0,2.0);
	wall1.setColor(0.9f, 0.9f, 0.5f, 1.0f);
	building->addFace(wall1);

	Face wall2(wall1);
	wall2.setBase(Transformation3D(0,20.0,0,X_AXIS,PI/2));
	building->addFace(wall2);

	Face wall3;
	wall3.setBase(Transformation3D(0,0,0,Y_AXIS,-PI/2));
	wall3.addVertex(0,0);
	wall3.addVertex(2.0,0);
	wall3.addVertex(2.0,20.0);
	wall3.addVertex(0,20.0);
	wall3.setColor(0.9f, 0.9f, 0.5f, 1.0f);
	building->addFace(wall3);

	Face wall4(wall3);
	wall4.setBase(Transformation3D(20.0,0,0,Y_AXIS,-PI/2));
	building->addFace(wall4);

	world+=building;
	StreamFile myfile(filename,false);
	myfile.write(&world);
}