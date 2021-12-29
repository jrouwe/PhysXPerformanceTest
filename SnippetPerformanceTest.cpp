#include <ctype.h>

#include "PxPhysicsAPI.h"

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"

// STL includes
#include <iostream>
#include <thread>
#include <chrono>

using namespace physx;
using namespace std;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;
PxFoundation*			gFoundation = nullptr;
PxPhysics*				gPhysics	= nullptr;
PxCooking*				gCooking	= nullptr;
PxDefaultCpuDispatcher*	gDispatcher = nullptr;
PxScene*				gScene		= nullptr;
PxMaterial*				gMaterial	= nullptr;

void createTerrain()
{
	const int n = 100;
	const float cell_size = 3.0f;
	const float max_height = 5.0f;
	float center = n * cell_size / 2;

	// Create vertices
	const int num_vertices = (n + 1) * (n + 1);
	PxVec3 *vertices = new PxVec3[num_vertices];
	for (int x = 0; x <= n; ++x)
		for (int z = 0; z <= n; ++z)
		{
			float height = sin(float(x) * 50.0f / n) * cos(float(z) * 50.0f / n);
			vertices[z * (n + 1) + x] = PxVec3(cell_size * x, max_height * height, cell_size * z);
		}

	// Create regular grid of triangles
	const int num_triangles = n * n * 2;
	PxU32 *indices = new PxU32[num_triangles * 3];
	PxU32 *next = indices;
	for (int x = 0; x < n; ++x)
		for (int z = 0; z < n; ++z)
		{
			int start = (n + 1) * z + x;

			*next++ = start;
			*next++ = start + n + 1;
			*next++ = start + 1;

			*next++ = start + 1;
			*next++ = start + n + 1;
			*next++ = start + n + 2;
		}

	// Cook the mesh
	PxTriangleMeshDesc desc;
	desc.points.count = num_vertices;
	desc.points.data = vertices;
	desc.points.stride = sizeof(PxVec3);
	desc.triangles.count = num_triangles;
	desc.triangles.data = indices;
	desc.triangles.stride = 3 * sizeof(PxU32);

	PxCookingParams params = gCooking->getParams();
	params.midphaseDesc = PxMeshMidPhase::eBVH34;
	gCooking->setParams(params);

	PxTriangleMesh *mesh = gCooking->createTriangleMesh(desc, gPhysics->getPhysicsInsertionCallback());

	// Create an actor
	PxRigidStatic *actor = gPhysics->createRigidStatic(PxTransform(PxVec3(-center, max_height, -center)));

	// Attach the mesh
	PxTriangleMeshGeometry geometry(mesh);
	PxRigidActorExt::createExclusiveShape(*actor, geometry, *gMaterial);

	// Add the actor to the scene
	gScene->addActor(*actor);

	mesh->release();
}

void createDynamic()
{
	// Create convex mesh
	const PxVec3 convex_points[] = { PxVec3(0, 1, 0), PxVec3(1, 0, 0), PxVec3(-1, 0, 0), PxVec3(0, 0, 1), PxVec3(0, 0, -1) };

	PxConvexMeshDesc convex_desc;
	convex_desc.points.count = 5;
	convex_desc.points.stride = sizeof(PxVec3);
	convex_desc.points.data = convex_points;
	convex_desc.flags = PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eDISABLE_MESH_VALIDATION | PxConvexFlag::eFAST_INERTIA_COMPUTATION;
	
	PxConvexMesh *convex_mesh = gCooking->createConvexMesh(convex_desc, gPhysics->getPhysicsInsertionCallback());

	// Create shapes
	PxShape *shapes[] = { 
		gPhysics->createShape(PxBoxGeometry(0.5f, 0.75f, 1.0f), *gMaterial),
		gPhysics->createShape(PxSphereGeometry(0.5f), *gMaterial),
		gPhysics->createShape(PxCapsuleGeometry(0.5f, 0.75f), *gMaterial),
		gPhysics->createShape(PxConvexMeshGeometry(convex_mesh), *gMaterial),
	};
	const int num_shapes = 4;

	convex_mesh->release();

	// Construct bodies
	for (int x = -10; x <= 10; ++x)
		for (int y = 0; y < num_shapes; ++y)
			for (int z = -10; z <= 10; ++z)
			{
				PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(7.5f * x, 20.0f + 2.0f * y, 7.5f * z));
				body->attachShape(*shapes[y]);
				PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
				gScene->addActor(*body);
			}

	// Release shapes
	for (int i = 0; i < num_shapes; ++i)
		shapes[i]->release();
}

void initPhysics(int inNumThreads)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,nullptr);
	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));
	gDispatcher = PxDefaultCpuDispatcherCreate(inNumThreads);

	// Create scene
	PxSceneDesc desc(gPhysics->getTolerancesScale());
	desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	desc.cpuDispatcher	= gDispatcher;
	desc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(desc);

	// Create material
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	// Create terrain
	createTerrain();

	// Create dynamic objects
	createDynamic();
}

void stepPhysics()
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}
	
void cleanupPhysics()
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gCooking);
	PX_RELEASE(gPhysics);
	PX_RELEASE(gFoundation);
}

int snippetMain(int, const char*const*)
{
	// Trace header
	cout << "Thread Count, Steps / Second" << endl;

	for (int num_threads = 1; num_threads <= (int)thread::hardware_concurrency(); ++num_threads)
	{
		// Init physics, use 0 threads when num_threads is 1 to avoid communication overhead between main thread and worker pool (all work will be done on main thread in this case)
		initPhysics(num_threads == 1? 0 : num_threads);

		constexpr int cMaxIterations = 500;

		// Start measuring
		chrono::high_resolution_clock::time_point clock_start = chrono::high_resolution_clock::now();
				
		// Step the world for a fixed amount of iterations
		for (int iterations = 0; iterations < cMaxIterations; ++iterations)
			stepPhysics();

		// Stop measuring
		chrono::high_resolution_clock::time_point clock_end = chrono::high_resolution_clock::now();
		chrono::nanoseconds total_duration = chrono::duration_cast<chrono::nanoseconds>(clock_end - clock_start);

		// Trace stat line
		cout << num_threads << ", " << double(cMaxIterations) / (1.0e-9 * total_duration.count()) << endl;

		cleanupPhysics();
	}

#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#endif

	return 0;
}