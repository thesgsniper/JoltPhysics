// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

// The Jolt headers don't include Jolt.h. Always include Jolt.h before including any other Jolt header.
// You can use Jolt.h in your precompiled header to speed up compilation.
#include <Jolt/Jolt.h>

// Jolt includes
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

// STL includes
#include <iostream>
#include <cstdarg>
#include <thread>

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

// All Jolt symbols are in the JPH namespace
using namespace JPH;

// If you want your code to compile using single or double precision write 0.0_r to get a Real value that compiles to double or float depending if JPH_DOUBLE_PRECISION is set or not.
using namespace JPH::literals;

// We're also using STL classes in this example
using namespace std;

// Callback for traces, connect this to your own trace function if you have one
static void TraceImpl(const char *inFMT, ...)
{
	// Format the message
	va_list list;
	va_start(list, inFMT);
	char buffer[1024];
	vsnprintf(buffer, sizeof(buffer), inFMT, list);
	va_end(list);

	// Print to the TTY
	cout << buffer << endl;
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{
	// Print to the TTY
	cout << inFile << ":" << inLine << ": (" << inExpression << ") " << (inMessage != nullptr? inMessage : "") << endl;

	// Breakpoint
	return true;
};

#endif // JPH_ENABLE_ASSERTS

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but you can have more
// layers if you want. E.g. you could have a layer for high detail collision (which is not used by the physics simulation
// but only if you do collision testing).
namespace Layers
{
	static constexpr ObjectLayer NON_MOVING = 0;
	static constexpr ObjectLayer MOVING = 1;
	static constexpr ObjectLayer NUM_LAYERS = 2;
};

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
	virtual bool					ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
	{
		switch (inObject1)
		{
		case Layers::NON_MOVING:
			return inObject2 == Layers::MOVING; // Non moving only collides with moving
		case Layers::MOVING:
			return true; // Moving collides with everything
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least want to have
// a layer for non-moving and moving objects to avoid having to update a tree full of static objects every frame.
// You can have a 1-on-1 mapping between object layers and broadphase layers (like in this case) but if you have
// many object layers you'll be creating many broad phase trees, which is not efficient. If you want to fine tune
// your broadphase layers define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace BroadPhaseLayers
{
	static constexpr BroadPhaseLayer NON_MOVING(0);
	static constexpr BroadPhaseLayer MOVING(1);
	static constexpr uint NUM_LAYERS(2);
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
									BPLayerInterfaceImpl()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
	}

	virtual uint					GetNumBroadPhaseLayers() const override
	{
		return BroadPhaseLayers::NUM_LAYERS;
	}

	virtual BroadPhaseLayer			GetBroadPhaseLayer(ObjectLayer inLayer) const override
	{
		JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
		return mObjectToBroadPhase[inLayer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *			GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
	{
		switch ((BroadPhaseLayer::Type)inLayer)
		{
		case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:	return "NON_MOVING";
		case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:		return "MOVING";
		default:													JPH_ASSERT(false); return "INVALID";
		}
	}
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	BroadPhaseLayer					mObjectToBroadPhase[Layers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
	virtual bool				ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
	{
		switch (inLayer1)
		{
		case Layers::NON_MOVING:
			return inLayer2 == BroadPhaseLayers::MOVING;
		case Layers::MOVING:
			return true;
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

// An example contact listener
class MyContactListener : public ContactListener
{
public:
	// See: ContactListener
	virtual ValidateResult	OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inCollisionResult) override
	{
		cout << "Contact validate callback" << endl;

		// Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
		return ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	virtual void			OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		cout << "A contact was added" << endl;
	}

	virtual void			OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		cout << "A contact was persisted" << endl;
	}

	virtual void			OnContactRemoved(const SubShapeIDPair &inSubShapePair) override
	{
		cout << "A contact was removed" << endl;
	}
};

// An example activation listener
class MyBodyActivationListener : public BodyActivationListener
{
public:
	virtual void		OnBodyActivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		cout << "A body got activated" << endl;
	}

	virtual void		OnBodyDeactivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		cout << "A body went to sleep" << endl;
	}
};

	class Physics
	{
	private:
		TempAllocatorImpl* temp_allocator = nullptr;
		JobSystemThreadPool* job_system = nullptr;

		const uint cMaxBodies = 1024; // Max rigidbodies
		const uint cNumBodyMutexes = 0; // 0 is default. 
		const uint cMaxBodyPairs = 1024; // How many contact pairs allowed
		const uint cMaxContactConstraints = 1024; // Max contact before things phase through each other

		BPLayerInterfaceImpl broad_phase_layer_interface; // Map obj layer to broadphase layer
		ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter; // filter object by broadphase
		ObjectLayerPairFilterImpl object_vs_object_layer_filter; // 

		PhysicsSystem* physics_system; // actual physics system
		MyBodyActivationListener body_activation_listener;
		MyContactListener contact_listener;

		const float cDeltaTime = 1.0f / 60.0f; // physics dt

		//Test objects, to be deleted later.
		Body* floor = nullptr;
		BodyID sphere_id;

	public:
		Physics();
		void Init();
		void Update();
		void Exit();
	};

	Physics::Physics() 
	{

		
		Init();
	}

	void Physics::Init()
	{
		RegisterDefaultAllocator(); // Register allocation hook

		Trace = TraceImpl; // Install callbacks
		JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)

			Factory::sInstance = new Factory(); // Create a factory
		RegisterTypes(); // Register all Jolt physics types

		// THESE NEED TO BE ALLOCATED IN THE SAME ORDER, OTHERWISE JOLT's
		// OVERRIDE NEW AND DELETE GETS REALLY FUSSY
		temp_allocator = new TempAllocatorImpl(10 * 1024 * 1024);
		job_system = new JobSystemThreadPool(cMaxPhysicsJobs,
			cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);
		physics_system = new PhysicsSystem();

		physics_system->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs,
			cMaxContactConstraints, broad_phase_layer_interface,
			object_vs_broadphase_layer_filter, object_vs_object_layer_filter);

		physics_system->SetBodyActivationListener(&body_activation_listener);
		physics_system->SetContactListener(&contact_listener);

		//anytime you want to access rigidbodies, need this!
		BodyInterface& body_interface = physics_system->GetBodyInterface();

		// Test Objects, to be deleted later.

		// Create the floor 
		BoxShapeSettings floor_shape_settings(Vec3(100.0f, 1.0f, 100.0f));

		// Create the shape
		ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();

		// We don't expect an error here, but you can check floor_shape_result 
		// for HasError() / GetError()
		ShapeRefC floor_shape = floor_shape_result.Get();

		// Create the settings for the body itself. Note that here you can also set 
		// other properties like the restitution / friction.
		BodyCreationSettings floor_settings(floor_shape,
			RVec3(0.0_r, -1.0_r, 0.0_r),
			Quat::sIdentity(),
			EMotionType::Static,
			Layers::NON_MOVING);

		// Create the actual rigid body
		// Note that if we run out of bodies this can return nullptr
		floor = body_interface.CreateBody(floor_settings);

		// Add it to the world
		body_interface.AddBody(floor->GetID(), EActivation::DontActivate);

		// Create the ball
		BodyCreationSettings sphere_settings(new SphereShape(0.5f),
			RVec3(0.0_r, 2.0_r, 0.0_r),
			Quat::sIdentity(),
			EMotionType::Dynamic,
			Layers::MOVING);

		sphere_id = body_interface.CreateAndAddBody(
			sphere_settings, EActivation::Activate); // Return id instead of pointer

		// Now you can interact with the dynamic body, 
		// in this case we're going to give it a velocity.
		// (note that if we had used CreateBody then we could 
		// have set the velocity straight on the body before adding it to the physics system)
		body_interface.SetLinearVelocity(sphere_id, Vec3(0.0f, -5.0f, 0.0f));

		// ideally you do this after initialising a lot of physics objects.
		physics_system->OptimizeBroadPhase();
	}
	void Physics::Update()
	{
		uint step = 0;
		//anytime you want to access rigidbodies, need this!
		BodyInterface& body_interface = physics_system->GetBodyInterface();

		while (body_interface.IsActive(sphere_id))
		{
			// Next step
			++step;

			// Output current position and velocity of the sphere
			RVec3 position = body_interface.GetCenterOfMassPosition(sphere_id);
			Vec3 velocity = body_interface.GetLinearVelocity(sphere_id);
			cout << "Step " << step
				<< ": Position = (" << position.GetX() << ", "
				<< position.GetY() << ", " << position.GetZ()
				<< "), Velocity = (" << velocity.GetX() << ", "
				<< velocity.GetY() << ", " << velocity.GetZ() << ")" << endl;

			// If you take larger steps than 1 / 60th of a second you need to do 
			// multiple collision steps in order to keep the simulation stable. 
			// Do 1 collision step per 1 / 60th of a second (round up).
			const int cCollisionSteps = 1;

			// Step the world
			physics_system->Update(cDeltaTime, cCollisionSteps, temp_allocator, job_system);
		}
	}
	void Physics::Exit()
	{
		BodyInterface& body_interface = physics_system->GetBodyInterface();

		// Remove the sphere from the physics system. Note that the sphere itself keeps all of its state and can be re-added at any time.
		body_interface.RemoveBody(sphere_id);

		// Destroy the sphere. After this the sphere ID is no longer valid.
		body_interface.DestroyBody(sphere_id);

		// Remove and destroy the floor
		body_interface.RemoveBody(floor->GetID());
		body_interface.DestroyBody(floor->GetID());

		// Unregisters all types with the factory and cleans up the default material
		UnregisterTypes();

		// Destroy the factory
		delete Factory::sInstance;
		Factory::sInstance = nullptr;

		delete this->temp_allocator;
		delete this->job_system;
		delete this->physics_system;
	}



// Program entry point
int main(int argc, char** argv)
{
	Physics physics;
	physics.Init();
	physics.Update();
	physics.Exit();

	return 0;
}
