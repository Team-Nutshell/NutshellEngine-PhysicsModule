#pragma once
#include "../Common/module_interfaces/ntshengn_physics_module_interface.h"
#include "../Common/ecs/ntshengn_ecs.h"
#include "../Common/utils/ntshengn_utils_math.h"
#include <set>
#include <vector>
#include <unordered_map>

struct RigidbodyState {
	NtshEngn::Math::vec3 linearAcceleration = NtshEngn::Math::vec3(0.0f, 0.0f, 0.0f);
	NtshEngn::Math::vec3 linearVelocity = NtshEngn::Math::vec3(0.0f, 0.0f, 0.0f);

	NtshEngn::Math::vec3 angularAcceleration = NtshEngn::Math::vec3(0.0f, 0.0f, 0.0f);
	NtshEngn::Math::vec3 angularVelocity = NtshEngn::Math::vec3(0.0f, 0.0f, 0.0f);
};

struct ObjectDuringCollisionResponseState {
	NtshEngn::Math::vec3 position = NtshEngn::Math::vec3(0.0f, 0.0f, 0.0f);
	NtshEngn::Math::vec3 linearVelocity = NtshEngn::Math::vec3(0.0f, 0.0f, 0.0f);
	NtshEngn::Math::vec3 angularVelocity = NtshEngn::Math::vec3(0.0f, 0.0f, 0.0f);
};

struct AABB {
	NtshEngn::Math::vec3 position;
	NtshEngn::Math::vec3 size;
};

struct BroadphaseCollision {
	NtshEngn::Entity entity1;
	NtshEngn::Entity entity2;

	bool operator<(const BroadphaseCollision& other) const {
		size_t hash = static_cast<size_t>(entity1) + (static_cast<size_t>(entity2) << 8);
		size_t otherHash = static_cast<size_t>(other.entity1) + (static_cast<size_t>(other.entity2) << 8);

		return hash < otherHash;
	}
};

struct NarrowphaseCollision {
	NtshEngn::Entity entity1;
	NtshEngn::Entity entity2;
	NtshEngn::Math::vec3 intersectionNormal;
	float intersectionDepth;
	std::vector<NtshEngn::Math::vec3> intersectionPoints;
};

namespace NtshEngn {

	class PhysicsModule : public PhysicsModuleInterface {
	public:
		PhysicsModule() : PhysicsModuleInterface("NutshellEngine Euler Physics Module") {}

		void init();
		void update(double dt);
		void destroy();

		// Returns an IntersectionInformation structure containing information about the intersection
		IntersectionInformation intersect(const ColliderShape* shape1, const ColliderShape* shape2);

		// Returns a list of RaycastInformation structures containing information about the hit entities
		std::vector<RaycastInformation> raycast(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax);

	public:
		const ComponentMask getComponentMask() const;

		void onEntityComponentAdded(Entity entity, Component componentID);
		void onEntityComponentRemoved(Entity entity, Component componentID);

	private:
		void eulerIntegrator(float dtSeconds);
		void collisionsDetection();
		void collisionsResponse();

		void collisionsBroadphase();
		void collisionsNarrowphase();

		IntersectionInformation intersect(const ColliderBox* box1, const ColliderBox* box2);
		IntersectionInformation intersect(const ColliderBox* box, const ColliderSphere* sphere);
		IntersectionInformation intersect(const ColliderSphere* sphere1, const ColliderSphere* sphere2);
		IntersectionInformation intersect(const ColliderSphere* sphere, const ColliderCapsule* capsule);
		IntersectionInformation intersect(const ColliderBox* box, const ColliderCapsule* capsule);
		IntersectionInformation intersect(const ColliderCapsule* capsule1, const ColliderCapsule* capsule2);

		IntersectionInformation intersect(const ColliderSphere* sphere, const ColliderBox* box);
		IntersectionInformation intersect(const ColliderCapsule* capsule, const ColliderBox* box);
		IntersectionInformation intersect(const ColliderCapsule* capsule, const ColliderSphere* sphere);

		Math::vec3 getCenter(const ColliderShape* shape);
		Math::vec3 getCenter(const ColliderSphere* sphere);
		Math::vec3 getCenter(const ColliderBox* box);
		Math::vec3 getCenter(const ColliderCapsule* capsule);

		void transform(ColliderShape* shape, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale);
		void transform(ColliderSphere* sphere, const Math::vec3& translation, const Math::vec3& scale);
		void transform(ColliderBox* box, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale);
		void transform(ColliderCapsule* capsule, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale);

		Math::vec3 closestPointOnSegment(const Math::vec3& point, const Math::vec3& segmentA, const Math::vec3& segmentB);
		std::pair<Math::vec3, Math::vec3> closestPointSegmentSegment(const Math::vec3& segmentA1, const Math::vec3& segmentA2, const Math::vec3& segmentB1, const Math::vec3& segmentB2);

		std::vector<Math::vec3> clipEdgesToBox(const std::array<std::pair<Math::vec3, Math::vec3>, 12>& edges, const ColliderBox* box, const Math::mat4& boxRotation);

	private:
		const uint32_t m_maxIterations = 60;
		const double m_maxDeltaTime = 1000.0 / 60.0;
		double m_timeAccumulator = 0.0;

		const Math::vec3 m_gravity = Math::vec3(0.0f, -9.81f, 0.0f);

		std::set<BroadphaseCollision> m_broadphaseCollisions;
		std::vector<NarrowphaseCollision> m_narrowphaseCollisions;

		std::unordered_map<Entity, RigidbodyState> m_rigidbodyStates;
	};

}