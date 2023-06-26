#pragma once
#include "../external/Common/module_interfaces/ntshengn_physics_module_interface.h"
#include "../external/Common/ecs/ntshengn_ecs.h"
#include "../external/nml/include/nml.h"
#include <unordered_map>

struct RigidbodyState {
	nml::vec3 acceleration = nml::vec3(0.0f, 0.0f, 0.0f);
	nml::vec3 velocity = nml::vec3(0.0f, 0.0f, 0.0f);
	nml::vec3 angularVelocity = nml::vec3(0.0f, 0.0f, 0.0f);
};

struct Collision {
	NtshEngn::Entity entity1;
	NtshEngn::Entity entity2;
	nml::vec3 intersectionNormal;
	float intersectionDepth;
};

class GJKSimplex {
public:
	GJKSimplex& operator=(std::initializer_list<nml::vec3> list) {
		for (std::initializer_list<nml::vec3>::iterator it = list.begin(); it != list.end(); it++) {
			m_points[std::distance(list.begin(), it)] = *it;
		}
		m_size = list.size();

		return *this;
	}

	void push_front(const nml::vec3& point) {
		m_points = { point, m_points[0], m_points[1], m_points[2] };
		m_size = std::min(m_size + 1, static_cast<size_t>(4));
	}

	nml::vec3& operator[](size_t i) {
		return m_points[i];
	}

	size_t size() {
		return m_size;
	}

	std::array<nml::vec3, 4>::iterator begin() {
		return m_points.begin();
	}

	std::array<nml::vec3, 4>::iterator end() {
		return m_points.end() - (4 - m_size);
	}

private:
	std::array<nml::vec3, 4> m_points = { nml::vec3(0.0f, 0.0f, 0.0f),
		nml::vec3(0.0f, 0.0f, 0.0f),
		nml::vec3(0.0f, 0.0f, 0.0f),
		nml::vec3(0.0f, 0.0f, 0.0f) };
	size_t m_size = 0;
};

namespace NtshEngn {

	class PhysicsModule : public PhysicsModuleInterface {
	public:
		PhysicsModule() : PhysicsModuleInterface("NutshellEngine Physics Euler Module") {}

		void init();
		void update(double dt);
		void destroy();

		// Returns an IntersectionInformation structure containing information about the intersection
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2);

	public:
		const ComponentMask getComponentMask() const;

		void onEntityComponentAdded(Entity entity, Component componentID);
		void onEntityComponentRemoved(Entity entity, Component componentID);

	private:
		void eulerIntegrator(float dtSeconds);
		void collisionsDetection();
		void collisionsResponse();

		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderSphere* sphere1, const NtshEngn::ColliderSphere* sphere2);
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderSphere* sphere, const NtshEngn::ColliderAABB* aabb);
		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderAABB* aabb1, const NtshEngn::ColliderAABB* aabb2);

		NtshEngn::IntersectionInformation intersect(const NtshEngn::ColliderAABB* aabb, const NtshEngn::ColliderSphere* sphere);

		NtshEngn::IntersectionInformation gjk(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2);

		bool simplexContainsOrigin(GJKSimplex& simplex, nml::vec3& direction);

		bool gjkLine(GJKSimplex& simplex, nml::vec3& direction);
		bool gjkTriangle(GJKSimplex& simplex, nml::vec3& direction);
		bool gjkTetrahedron(GJKSimplex& simplex, nml::vec3& direction);

		bool sameDirection(const nml::vec3& a, const nml::vec3& b);

		nml::vec3 getCenter(const NtshEngn::ColliderShape* shape);
		nml::vec3 getCenter(const NtshEngn::ColliderSphere* sphere);
		nml::vec3 getCenter(const NtshEngn::ColliderAABB* aabb);
		nml::vec3 getCenter(const NtshEngn::ColliderCapsule* capsule);

		nml::vec3 support(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2, const nml::vec3& direction);

		nml::vec3 getFarthestPointInDirection(const NtshEngn::ColliderShape* shape, const nml::vec3& direction);
		nml::vec3 getFarthestPointInDirection(const NtshEngn::ColliderSphere* sphere, const nml::vec3& direction);
		nml::vec3 getFarthestPointInDirection(const NtshEngn::ColliderAABB* aabb, const nml::vec3& direction);
		nml::vec3 getFarthestPointInDirection(const NtshEngn::ColliderCapsule* capsule, const nml::vec3& direction);

		std::pair<nml::vec3, float> epa(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2, GJKSimplex& simplex);

		std::pair<std::vector<nml::vec4>, size_t> getPolytopeNormals(const std::vector<nml::vec3>& polytope, const std::vector<size_t>& faces);
		void addIfUniqueEdge(std::vector<std::pair<size_t, size_t>>& edges, const std::vector<size_t>& polytopeIndices, size_t a, size_t b);

		void transform(NtshEngn::ColliderShape* shape, const nml::vec3& translation, const nml::vec3& rotation, const nml::vec3& scale);
		void transform(NtshEngn::ColliderSphere* sphere, const nml::vec3& translation, const nml::vec3& scale);
		void transform(NtshEngn::ColliderAABB* aabb, const nml::vec3& translation, const nml::vec3& rotation, const nml::vec3& scale);
		void transform(NtshEngn::ColliderCapsule* capsule, const nml::vec3& translation, const nml::vec3& rotation, const nml::vec3& scale);

	private:
		const nml::vec3 m_gravity = nml::vec3(0.0f, -9.81f, 0.0f);

		std::vector<Collision> m_collisions;

		std::unordered_map<Entity, RigidbodyState> m_rigidbodyStates;
	};

}