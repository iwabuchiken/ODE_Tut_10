#include <iostream>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

using namespace std;

// things that the user controls

static dReal speed=0,steer=0;	// user commands



// dynamics  and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dJointID joint[3];	// joint[0] is the front wheel
static dSpaceID space;
static dJointGroupID contactgroup;

static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);

	static float xyz[3] = {0.8317f,-0.9817f,0.8000f};
	static float hpr[3] = {121.0000f,-27.5000f,0.0000f};
	dsSetViewpoint (xyz,hpr);
	printf ("Press:\t'a' to increase speed.\n"
		  "\t'z' to decrease speed.\n"
		  "\t',' to steer left.\n"
		  "\t'.' to steer right.\n"
		  "\t' ' to reset speed and steering.\n"
		  "\t'1' to save the current state to 'state.dif'.\n");

}

static void command (int cmd)
{

	switch (cmd) {
	case 'a': case 'A':
		speed += 0.3;
		break;
	case 'z': case 'Z':
		speed -= 0.3;
		break;
	case ',':
		steer -= 0.5;
		break;
	case '.':
		steer += 0.5;
		break;
	case ' ':
		speed = 0;
		steer = 0;
		break;
	case '1': {
			FILE *f = fopen ("state.dif","wt");
			if (f) {
			  dWorldExportDIF (world,f,"");
			  fclose (f);
			}
		}
	}

}//command (int cmd)

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
}

static void simLoop (int pause)
{

	  int i;
	  if (!pause) {
	    // motor
	    dJointSetHinge2Param (joint[0],dParamVel2,-speed);
	    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

	    // steering
	    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
	    if (v > 0.1) v = 0.1;
	    if (v < -0.1) v = -0.1;
	    v *= 10.0;
	    dJointSetHinge2Param (joint[0],dParamVel,v);
	    dJointSetHinge2Param (joint[0],dParamFMax,0.2);
	    dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
	    dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
	    dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);

	    dSpaceCollide (space,0,&nearCallback);
	    dWorldStep (world,0.05);

	    // remove all contact joints
	    dJointGroupEmpty (contactgroup);

	  }

}

int main()
{

	int i;
	dMass m;


	// setup pointers to drawstuff callback functions
	dsFunctions fn;



	cout << "Hello world!" << endl;
	cout << "YES! RUN! JUMP! AND SIT DOWN, FEEL" << endl;
	return 0;
}
