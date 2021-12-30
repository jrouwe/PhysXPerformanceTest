#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"

using namespace physx;

extern void initPhysics(int inNumThreads, bool inCCD);
extern void stepPhysics();
extern void cleanupPhysics();

Snippets::Camera *gCamera = nullptr;

void motionCallback(int x, int y)
{
	gCamera->handleMotion(x, y);
}

void keyboardCallback(unsigned char key, int x, int y)
{
	if(key==27)
		exit(0);

	gCamera->handleKey(key, x, y);
}

void mouseCallback(int button, int state, int x, int y)
{
	gCamera->handleMouse(button, state, x, y);
}

void idleCallback()
{
	glutPostRedisplay();
}

void renderCallback()
{
	stepPhysics();

	Snippets::startRender(gCamera->getEye(), gCamera->getDir());

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if (nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
	}

	Snippets::finishRender();
}

void exitCallback(void)
{
	delete gCamera;

	cleanupPhysics();
}

void renderLoop()
{
	gCamera = new Snippets::Camera(PxVec3(50.0f, 50.0f, 50.0f), PxVec3(-0.6f,-0.2f,-0.7f));

	Snippets::setupDefaultWindow("PhysX Snippet Performance Test");
	Snippets::setupDefaultRenderState();

	glutIdleFunc(idleCallback);
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	motionCallback(0,0);

	atexit(exitCallback);

	initPhysics(2, true);
	glutMainLoop();
}

#endif
