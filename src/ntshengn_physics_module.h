#pragma once
#include "../external/Common/module_interfaces/ntshengn_physics_module_interface.h"
#include "../external/nml/include/nml.h"

namespace NtshEngn {

	class PhysicsModule : public PhysicsModuleInterface {
	public:
		PhysicsModule() : PhysicsModuleInterface("NutshellEngine Physics Simple Module") {}

		void init();
		void update(double dt);
		void destroy();

		// Returns true if the two shapes are intersecting with each other, else, returns false
		bool intersect(NtshEngn::ColliderShape* shape1, NtshEngn::ColliderShape* shape2);

	private:
		bool intersect(NtshEngn::ColliderSphere* sphere1, NtshEngn::ColliderSphere* sphere2);
		bool intersect(NtshEngn::ColliderSphere* sphere, NtshEngn::ColliderAABB* aabb);
		bool intersect(NtshEngn::ColliderSphere* sphere, NtshEngn::ColliderCapsule* capsule);
		bool intersect(NtshEngn::ColliderAABB* aabb1, NtshEngn::ColliderAABB* aabb2);
		bool intersect(NtshEngn::ColliderAABB* aabb, NtshEngn::ColliderCapsule* capsule);
		bool intersect(NtshEngn::ColliderCapsule* capsule1, NtshEngn::ColliderCapsule* capsule2);

		bool intersect(NtshEngn::ColliderAABB* aabb, NtshEngn::ColliderSphere* sphere);
		bool intersect(NtshEngn::ColliderCapsule* capsule, NtshEngn::ColliderSphere* sphere);
		bool intersect(NtshEngn::ColliderCapsule* capsule, NtshEngn::ColliderAABB* aabb);

		float squareDistancePointSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB);
		nml::vec3 closestPointOnSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB);
	};

}