#include "ntshengn_physics_module.h"
#include "../external/Module/utils/ntshengn_module_defines.h"
#include "../external/Module/utils/ntshengn_dynamic_library.h"
#include "../external/Common/utils/ntshengn_defines.h"
#include "../external/Common/utils/ntshengn_enums.h"
#include <algorithm>
#include <limits>
#include <cmath>

void NtshEngn::PhysicsModule::init() {
}

void NtshEngn::PhysicsModule::update(double dt) {
	const float dtSeconds = static_cast<float>(dt / 1000.0);

	// Euler integrator
	eulerIntegrator(dtSeconds);

	// Collisions detection
	collisionsDetection();

	// Collisions response
	collisionsResponse();
}

void NtshEngn::PhysicsModule::destroy() {
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2) {
	if ((shape1->getType() == NtshEngn::ColliderShapeType::Sphere) && (shape2->getType() == NtshEngn::ColliderShapeType::Sphere)) {
		return intersect(static_cast<const NtshEngn::ColliderSphere*>(shape1), static_cast<const NtshEngn::ColliderSphere*>(shape2));
	}
	else {
		return gjk(shape1, shape2);
	}
}

const NtshEngn::ComponentMask NtshEngn::PhysicsModule::getComponentMask() const {
	ComponentMask componentMask;
	componentMask.set(ecs->getComponentId<Rigidbody>());

	return componentMask;
}

void NtshEngn::PhysicsModule::onEntityComponentAdded(Entity entity, Component componentID) {
	if (componentID == ecs->getComponentId<Rigidbody>()) {
		m_rigidbodyStates[entity] = RigidbodyState();
	}
}

void NtshEngn::PhysicsModule::onEntityComponentRemoved(Entity entity, Component componentID) {
	if (componentID == ecs->getComponentId<Rigidbody>()) {
		m_rigidbodyStates.erase(entity);
	}
}

void NtshEngn::PhysicsModule::eulerIntegrator(float dtSeconds) {
	for (Entity entity : entities) {
		Rigidbody& entityRigidbody = ecs->getComponent<Rigidbody>(entity);
		RigidbodyState& entityRigidbodyState = m_rigidbodyStates[entity];
		if (!entityRigidbody.isStatic) {

			Transform& entityTransform = ecs->getComponent<Transform>(entity);

			entityRigidbodyState.acceleration = nml::vec3(entityRigidbody.force.data()) / entityRigidbody.mass;
			if (entityRigidbody.isAffectedByConstants) {
				entityRigidbodyState.acceleration += m_gravity;
			}

			entityRigidbodyState.velocity += entityRigidbodyState.acceleration * dtSeconds;
			entityRigidbodyState.angularVelocity += nml::vec3(entityRigidbody.torque.data()) * entityRigidbody.inertia * dtSeconds;

			entityTransform.position[0] += entityRigidbodyState.velocity.x * dtSeconds;
			entityTransform.position[1] += entityRigidbodyState.velocity.y * dtSeconds;
			entityTransform.position[2] += entityRigidbodyState.velocity.z * dtSeconds;
		}
		else {
			entityRigidbodyState.velocity = { 0.0f, 0.0f, 0.0f };
		}

		entityRigidbody.force = { 0.0f, 0.0f, 0.0f };
	}
}

void NtshEngn::PhysicsModule::collisionsDetection() {
	std::set<Entity>::iterator it = entities.begin();
	while (it != entities.end()) {
		Entity entity = *it;
		NtshEngn::ColliderShape* colliderShape = nullptr;

		NtshEngn::ColliderSphere colliderSphere;
		NtshEngn::ColliderAABB colliderAABB;
		NtshEngn::ColliderCapsule colliderCapsule;
		if (ecs->hasComponent<SphereCollidable>(entity)) {
			colliderSphere = ecs->getComponent<SphereCollidable>(entity).collider;
			colliderShape = &colliderSphere;
		}
		else if (ecs->hasComponent<AABBCollidable>(entity)) {
			colliderAABB = ecs->getComponent<AABBCollidable>(entity).collider;
			colliderShape = &colliderAABB;
		}
		else if (ecs->hasComponent<CapsuleCollidable>(entity)) {
			colliderCapsule = ecs->getComponent<CapsuleCollidable>(entity).collider;
			colliderShape = &colliderCapsule;
		}

		if (colliderShape) {
			const Transform& entityTransform = ecs->getComponent<Transform>(entity);
			transform(colliderShape, nml::vec3(entityTransform.position.data()), nml::vec3(entityTransform.rotation.data()), nml::vec3(entityTransform.scale.data()));

			std::set<Entity>::iterator otherIt = entities.begin();
			std::advance(otherIt, std::distance(entities.begin(), it));
			while (otherIt != entities.end()) {
				Entity otherEntity = *otherIt;
				if (otherEntity != entity) {
					NtshEngn::ColliderShape* otherColliderShape = nullptr;

					NtshEngn::ColliderSphere otherColliderSphere;
					NtshEngn::ColliderAABB otherColliderAABB;
					NtshEngn::ColliderCapsule otherColliderCapsule;
					if (ecs->hasComponent<SphereCollidable>(otherEntity)) {
						otherColliderSphere = ecs->getComponent<SphereCollidable>(otherEntity).collider;
						otherColliderShape = &otherColliderSphere;
					}
					else if (ecs->hasComponent<AABBCollidable>(otherEntity)) {
						otherColliderAABB = ecs->getComponent<AABBCollidable>(otherEntity).collider;
						otherColliderShape = &otherColliderAABB;
					}
					else if (ecs->hasComponent<CapsuleCollidable>(otherEntity)) {
						otherColliderCapsule = ecs->getComponent<CapsuleCollidable>(otherEntity).collider;
						otherColliderShape = &otherColliderCapsule;
					}

					if (otherColliderShape) {
						const Transform& otherEntityTransform = ecs->getComponent<Transform>(otherEntity);
						transform(otherColliderShape, nml::vec3(otherEntityTransform.position.data()), nml::vec3(otherEntityTransform.rotation.data()), nml::vec3(otherEntityTransform.scale.data()));

						IntersectionInformation intersectionInformation = intersect(colliderShape, otherColliderShape);
						if (intersectionInformation.hasIntersected) {
							Collision collision;
							collision.entity1 = entity;
							collision.entity2 = otherEntity;
							collision.intersectionNormal = nml::vec3(intersectionInformation.intersectionNormal.data());
							collision.intersectionDepth = intersectionInformation.intersectionDepth;
							m_collisions.push_back(collision);
						}
					}
				}

				otherIt++;
			}
		}

		it++;
	}
}

void NtshEngn::PhysicsModule::collisionsResponse() {
	for (const Collision& collision : m_collisions) {
		const Rigidbody& entity1Rigidbody = ecs->getComponent<Rigidbody>(collision.entity1);
		RigidbodyState& entity1RigidbodyState = m_rigidbodyStates[collision.entity1];

		Transform& entity1Transform = ecs->getComponent<Transform>(collision.entity1);

		const Rigidbody& entity2Rigidbody = ecs->getComponent<Rigidbody>(collision.entity2);
		RigidbodyState& entity2RigidbodyState = m_rigidbodyStates[collision.entity2];

		Transform& entity2Transform = ecs->getComponent<Transform>(collision.entity2);

		if (entity1Rigidbody.isStatic && entity2Rigidbody.isStatic) {
			continue;
		}

		const nml::vec3 relativeVelocity = entity2RigidbodyState.velocity - entity1RigidbodyState.velocity;
		const float rVdotN = nml::dot(relativeVelocity, collision.intersectionNormal);

		if (rVdotN >= 0.0f) {
			continue;
		}

		const float e = entity1Rigidbody.restitution * entity2Rigidbody.restitution;
		const float invMass1 = 1.0f / entity1Rigidbody.mass;
		const float invMass2 = 1.0f / entity2Rigidbody.mass;
		const float j = (-(1.0f + e) * rVdotN) / (invMass1 + invMass2);
		const nml::vec3 impulse = j * collision.intersectionNormal;

		const nml::vec3 correction = (std::max(collision.intersectionDepth - 0.01f, 0.0f) * 0.8f * collision.intersectionNormal / (invMass1 + invMass2));

		if (!entity1Rigidbody.isStatic) {
			entity1RigidbodyState.velocity -= impulse * invMass1;

			const nml::vec3 entity1PositionCorrection = correction * invMass1;

			entity1Transform.position[0] -= entity1PositionCorrection.x;
			entity1Transform.position[1] -= entity1PositionCorrection.y;
			entity1Transform.position[2] -= entity1PositionCorrection.z;
		}

		if (!entity2Rigidbody.isStatic) {
			entity2RigidbodyState.velocity += impulse * invMass2;

			const nml::vec3 entity2PositionCorrected = correction * invMass2;

			entity2Transform.position[0] += entity2PositionCorrected.x;
			entity2Transform.position[1] += entity2PositionCorrected.y;
			entity2Transform.position[2] += entity2PositionCorrected.z;
		}
	}
	m_collisions.clear();
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const NtshEngn::ColliderSphere* sphere1, const NtshEngn::ColliderSphere* sphere2) {
	IntersectionInformation intersectionInformation;

	const nml::vec3 sphere1Center = nml::vec3(sphere1->center.data());
	const nml::vec3 sphere2Center = nml::vec3(sphere2->center.data());
	const nml::vec3 centerDiff = sphere2Center - sphere1Center;
	const float centerDiffLength = centerDiff.length();

	if ((centerDiffLength < 0.000001f) || (centerDiffLength >= (sphere1->radius + sphere2->radius))) {
		intersectionInformation.hasIntersected = false;
		
		return intersectionInformation;
	}

	const nml::vec3 intersectionNormal = nml::normalize(centerDiff);

	intersectionInformation.hasIntersected = true;
	intersectionInformation.intersectionNormal = { intersectionNormal[0], intersectionNormal[1], intersectionNormal[2] };
	intersectionInformation.intersectionDepth = (sphere1->radius + sphere2->radius) - centerDiffLength;

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::gjk(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2) {
	IntersectionInformation intersectionInformation;

	GJKSimplex simplex;

	nml::vec3 direction = getCenter(shape2) - getCenter(shape1);

	nml::vec3 supp = support(shape1, shape2, direction);
	simplex.push_front(supp);

	direction = supp * -1.0f;

	while (true) {
		supp = support(shape1, shape2, direction);

		if (nml::dot(supp, direction) <= 0.0f) { // No intersection
			intersectionInformation.hasIntersected = false;

			return intersectionInformation;
		}

		simplex.push_front(supp);

		if (simplexContainsOrigin(simplex, direction)) { // Intersection
			intersectionInformation.hasIntersected = true;

			const std::pair<nml::vec3, float> intersectionNormalAndDepth = epa(shape1, shape2, simplex);
			intersectionInformation.intersectionNormal = { intersectionNormalAndDepth.first[0], intersectionNormalAndDepth.first[1], intersectionNormalAndDepth.first[2] };
			intersectionInformation.intersectionDepth = intersectionNormalAndDepth.second + 0.001f;

			return intersectionInformation;
		}
	}

	NTSHENGN_MODULE_ERROR("Reached impossible path.", NtshEngn::Result::ModuleError);
}

bool NtshEngn::PhysicsModule::simplexContainsOrigin(GJKSimplex& simplex, nml::vec3& direction) {
	if (simplex.size() == 2) {
		return gjkLine(simplex, direction);
	}
	else if (simplex.size() == 3) {
		return gjkTriangle(simplex, direction);
	}
	else { // simplex.size() == 4
		return gjkTetrahedron(simplex, direction);
	}
}

bool NtshEngn::PhysicsModule::gjkLine(GJKSimplex& simplex, nml::vec3& direction) {
	const nml::vec3 a = simplex[0];
	const nml::vec3 b = simplex[1];

	const nml::vec3 ab = b - a;
	const nml::vec3 ao = a * -1.0f;

	if (sameDirection(ab, ao)) {
		direction = nml::cross(nml::cross(ab, ao), ab);
	}
	else {
		simplex = { a };
		direction = ao;
	}

	return false;
}

bool NtshEngn::PhysicsModule::gjkTriangle(GJKSimplex& simplex, nml::vec3& direction) {
	const nml::vec3 a = simplex[0];
	const nml::vec3 b = simplex[1];
	const nml::vec3 c = simplex[2];

	const nml::vec3 ab = b - a;
	const nml::vec3 ac = c - a;
	const nml::vec3 ao = a * -1.0f;

	const nml::vec3 abc = nml::cross(ab, ac);

	if (sameDirection(nml::cross(abc, ac), ao)) {
		if (sameDirection(ac, ao)) {
			simplex = { a, c };
			direction = nml::cross(nml::cross(ac, ao), ac);
		}
		else {
			simplex = { a, b };
			return gjkLine(simplex, direction);
		}
	}
	else {
		if (sameDirection(nml::cross(ab, abc), ao)) {
			simplex = { a, b };
			return gjkLine(simplex, direction);
		}
		else {
			if (sameDirection(abc, ao)) {
				direction = abc;
			}
			else {
				simplex = { a, c, b };
				direction = abc * -1.0f;
			}
		}
	}

	return false;
}

bool NtshEngn::PhysicsModule::gjkTetrahedron(GJKSimplex& simplex, nml::vec3& direction) {
	const nml::vec3 a = simplex[0];
	const nml::vec3 b = simplex[1];
	const nml::vec3 c = simplex[2];
	const nml::vec3 d = simplex[3];

	const nml::vec3 ab = b - a;
	const nml::vec3 ac = c - a;
	const nml::vec3 ad = d - a;
	const nml::vec3 ao = a * -1.0f;

	const nml::vec3 abc = nml::cross(ab, ac);
	const nml::vec3 acd = nml::cross(ac, ad);
	const nml::vec3 adb = nml::cross(ad, ab);

	if (sameDirection(abc, ao)) {
		simplex = { a, b, c };
		return gjkTriangle(simplex, direction);
	}

	if (sameDirection(acd, ao)) {
		simplex = { a, c, d };
		return gjkTriangle(simplex, direction);
	}

	if (sameDirection(adb, ao)) {
		simplex = { a, d, b };
		return gjkTriangle(simplex, direction);
	}

	return true;
}

bool NtshEngn::PhysicsModule::sameDirection(const nml::vec3& a, const nml::vec3& b) {
	return nml::dot(a, b) > 0.0f;
}

nml::vec3 NtshEngn::PhysicsModule::getCenter(const NtshEngn::ColliderShape* shape) {
	if (shape->getType() == NtshEngn::ColliderShapeType::Sphere) {
		return getCenter(static_cast<const NtshEngn::ColliderSphere*>(shape));
	}
	else if (shape->getType() == NtshEngn::ColliderShapeType::AABB) {
		return getCenter(static_cast<const NtshEngn::ColliderAABB*>(shape));
	}
	else if (shape->getType() == NtshEngn::ColliderShapeType::Capsule) {
		return getCenter(static_cast<const NtshEngn::ColliderCapsule*>(shape));
	}

	return nml::vec3(0.0f, 0.0f, 0.0f);
}

nml::vec3 NtshEngn::PhysicsModule::getCenter(const NtshEngn::ColliderSphere* sphere) {
	return nml::vec3(sphere->center.data());
}

nml::vec3 NtshEngn::PhysicsModule::getCenter(const NtshEngn::ColliderAABB* aabb) {
	return nml::vec3((aabb->min[0] + aabb->max[0]) / 2.0f, (aabb->min[1] + aabb->max[1]) / 2.0f, (aabb->min[2] + aabb->max[2]) / 2.0f);
}

nml::vec3 NtshEngn::PhysicsModule::getCenter(const NtshEngn::ColliderCapsule* capsule) {
	return nml::vec3((capsule->base[0] + capsule->tip[0]) / 2.0f, (capsule->base[1] + capsule->tip[1]) / 2.0f, (capsule->base[2] + capsule->tip[2]) / 2.0f);
}

nml::vec3 NtshEngn::PhysicsModule::support(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2, const nml::vec3& direction) {
	const nml::vec3 p1 = getFarthestPointInDirection(shape1, direction);
	const nml::vec3 p2 = getFarthestPointInDirection(shape2, direction * -1.0f);

	return p1 - p2;
}

nml::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const NtshEngn::ColliderShape* shape, const nml::vec3& direction) {
	if (shape->getType() == NtshEngn::ColliderShapeType::Sphere) {
		return getFarthestPointInDirection(static_cast<const NtshEngn::ColliderSphere*>(shape), direction);
	}
	else if (shape->getType() == NtshEngn::ColliderShapeType::AABB) {
		return getFarthestPointInDirection(static_cast<const NtshEngn::ColliderAABB*>(shape), direction);
	}
	else if (shape->getType() == NtshEngn::ColliderShapeType::Capsule) {
		return getFarthestPointInDirection(static_cast<const NtshEngn::ColliderCapsule*>(shape), direction);
	}

	return nml::vec3(0.0f, 0.0f, 0.0f);
}

nml::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const NtshEngn::ColliderSphere* sphere, const nml::vec3& direction) {
	const float directionLength = direction.length();

	return nml::vec3(sphere->center.data()) + direction * (sphere->radius / directionLength);
}

nml::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const NtshEngn::ColliderAABB* aabb, const nml::vec3& direction) {
	return nml::vec3(direction.x >= 0.0f ? aabb->max[0] : aabb->min[0],
		direction.y >= 0.0f ? aabb->max[1] : aabb->min[1],
		direction.z >= 0.0f ? aabb->max[2] : aabb->min[2]);
}

nml::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const NtshEngn::ColliderCapsule* capsule, const nml::vec3& direction) {
	const float directionLength = direction.length();
	const nml::vec3 capsuleBase = nml::vec3(capsule->base.data());
	const nml::vec3 capsuleTip = nml::vec3(capsule->tip.data());

	return ((nml::dot(direction, nml::vec3(capsuleTip - capsuleBase)) >= 0.0) ? capsuleTip : capsuleBase) + direction * (capsule->radius / directionLength);
}

std::pair<nml::vec3, float> NtshEngn::PhysicsModule::epa(const NtshEngn::ColliderShape* shape1, const NtshEngn::ColliderShape* shape2, GJKSimplex& simplex) {
	std::vector<nml::vec3> polytope(simplex.begin(), simplex.end());
	std::vector<size_t> faces = {
		0, 1, 2,
		0, 3, 1,
		0, 2, 3,
		1, 3, 2
	};
	std::pair<std::vector<nml::vec4>, size_t> polytopeNormals = getPolytopeNormals(polytope, faces);
	std::vector<nml::vec4> normals = polytopeNormals.first;
	size_t minTriangle = polytopeNormals.second;

	nml::vec3 minNormal;
	float minDistance = std::numeric_limits<float>::max();
	while (minDistance == std::numeric_limits<float>::max()) {
		minNormal = nml::vec3(normals[minTriangle]);
		minDistance = normals[minTriangle].w;

		const nml::vec3 supp = support(shape1, shape2, minNormal);
		const float sDistance = nml::dot(minNormal, supp);
		if (std::abs(sDistance - minDistance) > 0.001f) {
			minDistance = std::numeric_limits<float>::max();

			std::vector<std::pair<size_t, size_t>> uniqueEdges;
			for (size_t i = 0; i < normals.size(); i++) {
				size_t f = i * 3;
				if (sameDirection(nml::vec3(normals[i]), supp - polytope[faces[f]])) {
					addIfUniqueEdge(uniqueEdges, faces, f, f + 1);
					addIfUniqueEdge(uniqueEdges, faces, f + 1, f + 2);
					addIfUniqueEdge(uniqueEdges, faces, f + 2, f);

					faces[f + 2] = faces.back();
					faces.pop_back();
					faces[f + 1] = faces.back();
					faces.pop_back();
					faces[f] = faces.back();
					faces.pop_back();

					normals[i] = normals.back();
					normals.pop_back();

					i--;
				}
			}

			std::vector<size_t> newFaces;
			for (std::pair<size_t, size_t> uniqueEdge : uniqueEdges) {
				newFaces.push_back(uniqueEdge.first);
				newFaces.push_back(uniqueEdge.second);
				newFaces.push_back(polytope.size());
			}

			polytope.push_back(supp);

			std::pair<std::vector<nml::vec4>, size_t> newFacesNormals = getPolytopeNormals(polytope, newFaces);
			const std::vector<nml::vec4> newNormals = newFacesNormals.first;
			const size_t newMinTriangle = newFacesNormals.second;

			float oldMinDistance = std::numeric_limits<float>::max();
			for (size_t i = 0; i < normals.size(); i++) {
				if (normals[i].w < oldMinDistance) {
					oldMinDistance = normals[i].w;
					minTriangle = i;
				}
			}

			if (newNormals[newMinTriangle].w < oldMinDistance) {
				minTriangle = newMinTriangle + static_cast<size_t>(normals.size());
			}

			faces.insert(faces.end(), newFaces.begin(), newFaces.end());
			normals.insert(normals.end(), newNormals.begin(), newNormals.end());
		}
	}

	return { minNormal, minDistance };
}

std::pair<std::vector<nml::vec4>, size_t> NtshEngn::PhysicsModule::getPolytopeNormals(const std::vector<nml::vec3>& polytope, const std::vector<size_t>& faces) {
	std::vector<nml::vec4> normals;
	size_t minTriangle = 0;
	float minDistance = std::numeric_limits<float>::max();

	for (size_t i = 0; i < faces.size(); i += 3) {
		const nml::vec3 a = polytope[faces[i]];
		const nml::vec3 b = polytope[faces[i + 1]];
		const nml::vec3 c = polytope[faces[i + 2]];

		const nml::vec3 ab = b - a;
		const nml::vec3 ac = c - a;

		nml::vec3 n = nml::normalize(nml::cross(ab, ac));
		float distance = nml::dot(n, a);
		if (distance < 0.0f) {
			n *= -1;
			distance *= -1;
		}

		normals.push_back(nml::vec4(n, distance));

		if (distance < minDistance) {
			minDistance = distance;
			minTriangle = i / 3;
		}
	}

	return { normals, minTriangle };
}

void NtshEngn::PhysicsModule::addIfUniqueEdge(std::vector<std::pair<size_t, size_t>>& edges, const std::vector<size_t>& polytopeIndices, size_t a, size_t b) {
	std::vector<std::pair<size_t, size_t>>::iterator reverse = std::find(edges.begin(), edges.end(), std::pair<size_t, size_t>(polytopeIndices[b], polytopeIndices[a]));
	if (reverse != edges.end()) {
		edges.erase(reverse);
	}
	else {
		edges.push_back(std::pair<size_t, size_t>(polytopeIndices[a], polytopeIndices[b]));
	}
}

void NtshEngn::PhysicsModule::transform(NtshEngn::ColliderShape* shape, const nml::vec3& translation, const nml::vec3& rotation, const nml::vec3& scale) {
	if (shape->getType() == NtshEngn::ColliderShapeType::Sphere) {
		transform(static_cast<NtshEngn::ColliderSphere*>(shape), translation, scale);
	}
	else if (shape->getType() == NtshEngn::ColliderShapeType::AABB) {
		transform(static_cast<NtshEngn::ColliderAABB*>(shape), translation, rotation, scale);
	}
	else if (shape->getType() == NtshEngn::ColliderShapeType::Capsule) {
		transform(static_cast<NtshEngn::ColliderCapsule*>(shape), translation, rotation, scale);
	}
}

void NtshEngn::PhysicsModule::transform(NtshEngn::ColliderSphere* sphere, const nml::vec3& translation, const nml::vec3& scale) {
	sphere->center = { translation.x, translation.y, translation.z };
	sphere->radius *= std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z)));
}

void NtshEngn::PhysicsModule::transform(NtshEngn::ColliderAABB* aabb, const nml::vec3& translation, const nml::vec3& rotation, const nml::vec3& scale) {
	NtshEngn::ColliderAABB newAABB;
	newAABB.min = { translation.x, translation.y, translation.z };
	newAABB.max = { translation.x, translation.y, translation.z };

	float a;
	float b;

	const nml::mat4 rotationMatrix = nml::rotate(rotation[0], nml::vec3(1.0f, 0.0f, 0.0f)) *
		nml::rotate(rotation[1], nml::vec3(0.0f, 1.0f, 0.0f)) *
		nml::rotate(rotation[2], nml::vec3(0.0f, 0.0f, 1.0f));

	for (uint8_t i = 0; i < 3; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			a = rotationMatrix[i][j] * aabb->min[j] * std::abs(scale[i]);
			b = rotationMatrix[i][j] * aabb->max[j] * std::abs(scale[i]);
			
			newAABB.min[i] += (a < b) ? a : b;
			newAABB.max[i] += (a < b) ? b : a;
		}
	}

	aabb->min = newAABB.min;
	aabb->max = newAABB.max;
}

void NtshEngn::PhysicsModule::transform(NtshEngn::ColliderCapsule* capsule, const nml::vec3& translation, const nml::vec3& rotation, const nml::vec3& scale) {
	capsule->base = { capsule->base[0] + translation.x, capsule->base[1] + translation.y, capsule->base[2] + translation.z };
	capsule->tip = { capsule->tip[0] + translation.x, capsule->tip[1] + translation.y, capsule->tip[2] + translation.z };

	const nml::mat4 rotationMatrix = nml::rotate(rotation[0], nml::vec3(1.0f, 0.0f, 0.0f)) *
		nml::rotate(rotation[1], nml::vec3(0.0f, 1.0f, 0.0f)) *
		nml::rotate(rotation[2], nml::vec3(0.0f, 0.0f, 1.0f));
	const nml::vec3 tipMinusBase = nml::vec3(capsule->tip.data()) - nml::vec3(capsule->base.data());

	const nml::vec3 tipRotation = nml::vec3(rotationMatrix * nml::vec4(tipMinusBase, 1.0f));

	capsule->tip = { tipRotation.x + capsule->base[0], tipRotation.y + capsule->base[1], tipRotation.z + capsule->base[2]};
	capsule->radius *= std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z)));
}

extern "C" NTSHENGN_MODULE_API NtshEngn::PhysicsModuleInterface* createModule() {
	return new NtshEngn::PhysicsModule;
}

extern "C" NTSHENGN_MODULE_API void destroyModule(NtshEngn::PhysicsModuleInterface* m) {
	delete m;
}