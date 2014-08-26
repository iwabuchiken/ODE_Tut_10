#include <iostream>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

using namespace std;

// things that the user controls

static dReal speed=0,steer=0;	// user commands

// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;

static void start()
{
    //debug
    speed =0;
}

static void command (int cmd)
{
    //debug
    speed =0;

}


static void simLoop (int pause)
{
    //debug
    speed =0;

}

int main()
{

	int i;
	dMass m;

	// setup pointers to drawstuff callback functions
//	dsFunctions fn;



    cout << "Hello world!" << endl;
    cout << "YES! RUN! JUMP! AND SIT DOWN, FEEL" << endl;
    return 0;
}
