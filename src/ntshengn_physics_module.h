#pragma once
#include "../external/Common/module_interfaces/ntshengn_physics_module_interface.h"
#include "../external/Common/ecs/ntshengn_ecs.h"
#include "../external/nml/include/nml.h"
#include <unordered_map>

namespace NtshEngn {

	struct RigidbodyState {
		nml::vec3 acceleration = nml::vec3(0.0f, 0.0f, 0.0f);
		nml::vec3 velocity = nml::vec3(0.0f, 0.0f, 0.0f);
	};

	class PhysicsModule : public PhysicsModuleInterface {
	public:
		PhysicsModule() : PhysicsModuleInterface("NutshellEngine Physics Simple Module") {}

		void init();
		void update(double dt);
		void destroy();

		// Returns true if the two shapes are intersecting with each other, else, returns false
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2);

	public:
		const ComponentMask getComponentMask() const;

		void onEntityComponentAdded(Entity entity, Component componentID);
		void onEntityComponentRemoved(Entity entity, Component componentID);

	private:
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderSphere* sphere1, const NtshEngn::ColliderSphere* sphere2);
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderSphere* sphere, const NtshEngn::ColliderAABB* aabb);
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderSphere* sphere, const NtshEngn::ColliderCapsule* capsule);
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderAABB* aabb1, const NtshEngn::ColliderAABB* aabb2);
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderAABB* aabb, const NtshEngn::ColliderCapsule* capsule);
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderCapsule* capsule1, const NtshEngn::ColliderCapsule* capsule2);

		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderAABB* aabb, const NtshEngn::ColliderSphere* sphere);
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderCapsule* capsule, const NtshEngn::ColliderSphere* sphere);
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderCapsule* capsule, const NtshEngn::ColliderAABB* aabb);

		float squareDistancePointSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB);
		nml::vec3 closestPointOnSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB);

	private:
		const nml::vec3 m_gravity = nml::vec3(0.0f, -1.0f, 0.0f);

		std::unordered_map<Entity, RigidbodyState> m_rigidbodyStates;
	};

}