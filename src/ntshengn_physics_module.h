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
		bool intersect(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2);

	private:
		bool intersect(const NtshEngn::ColliderSphere* sphere1, const NtshEngn::ColliderSphere* sphere2);
		bool intersect(const NtshEngn::ColliderSphere* sphere, const NtshEngn::ColliderAABB* aabb);
		bool intersect(const NtshEngn::ColliderSphere* sphere, const NtshEngn::ColliderCapsule* capsule);
		bool intersect(const NtshEngn::ColliderAABB* aabb1, const NtshEngn::ColliderAABB* aabb2);
		bool intersect(const NtshEngn::ColliderAABB* aabb, const NtshEngn::ColliderCapsule* capsule);
		bool intersect(const NtshEngn::ColliderCapsule* capsule1, const NtshEngn::ColliderCapsule* capsule2);

		bool intersect(const NtshEngn::ColliderAABB* aabb, const NtshEngn::ColliderSphere* sphere);
		bool intersect(const NtshEngn::ColliderCapsule* capsule, const NtshEngn::ColliderSphere* sphere);
		bool intersect(const NtshEngn::ColliderCapsule* capsule, const NtshEngn::ColliderAABB* aabb);

		float squareDistancePointSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB);
		nml::vec3 closestPointOnSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB);
	};

}