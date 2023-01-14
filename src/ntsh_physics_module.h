#pragma once
#include "../external/Common/module_interfaces/ntsh_physics_module_interface.h"
#include "../external/nml/include/nml.h"

class NutshellPhysicsModule : public NutshellPhysicsModuleInterface {
public:
	NutshellPhysicsModule() : NutshellPhysicsModuleInterface("Nutshell Physics Test Module") {}

	void init();
	void update(double dt);
	void destroy();

	// Returns true if the two shapes are intersecting with each other, else, returns false
	bool intersect(Ntsh::ColliderShape* shape1, Ntsh::ColliderShape* shape2);

private:
	bool intersect(Ntsh::ColliderSphere* sphere1, Ntsh::ColliderSphere* sphere2);
	bool intersect(Ntsh::ColliderSphere* sphere, Ntsh::ColliderAABB* aabb);
	bool intersect(Ntsh::ColliderSphere* sphere, Ntsh::ColliderCapsule* capsule);
	bool intersect(Ntsh::ColliderAABB* aabb1, Ntsh::ColliderAABB* aabb2);
	bool intersect(Ntsh::ColliderAABB* aabb, Ntsh::ColliderCapsule* capsule);
	bool intersect(Ntsh::ColliderCapsule* capsule1, Ntsh::ColliderCapsule* capsule2);

	bool intersect(Ntsh::ColliderAABB* aabb, Ntsh::ColliderSphere* sphere);
	bool intersect(Ntsh::ColliderCapsule* capsule, Ntsh::ColliderSphere* sphere);
	bool intersect(Ntsh::ColliderCapsule* capsule, Ntsh::ColliderAABB* aabb);

	float squareDistancePointSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB);
	nml::vec3 closestPointOnSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB);
};