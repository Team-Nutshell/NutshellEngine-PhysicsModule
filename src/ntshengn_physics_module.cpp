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

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderShape* shape1, const ColliderShape* shape2) {
	if ((shape1->getType() == ColliderShapeType::Sphere) && (shape2->getType() == ColliderShapeType::Sphere)) {
		return intersect(static_cast<const ColliderSphere*>(shape1), static_cast<const ColliderSphere*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Sphere) && (shape2->getType() == ColliderShapeType::AABB)) {
		return intersect(static_cast<const ColliderSphere*>(shape1), static_cast<const ColliderAABB*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Sphere) && (shape2->getType() == ColliderShapeType::Capsule)) {
		return intersect(static_cast<const ColliderSphere*>(shape1), static_cast<const ColliderCapsule*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Capsule) && (shape2->getType() == ColliderShapeType::Sphere)) {
		return intersect(static_cast<const ColliderCapsule*>(shape1), static_cast<const ColliderSphere*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::AABB) && (shape2->getType() == ColliderShapeType::Sphere)) {
		return intersect(static_cast<const ColliderAABB*>(shape1), static_cast<const ColliderSphere*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::AABB) && (shape2->getType() == ColliderShapeType::AABB)) {
		return intersect(static_cast<const ColliderAABB*>(shape1), static_cast<const ColliderAABB*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::AABB) && (shape2->getType() == ColliderShapeType::Capsule)) {
		return intersect(static_cast<const ColliderAABB*>(shape1), static_cast<const ColliderCapsule*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Capsule) && (shape2->getType() == ColliderShapeType::AABB)) {
		return intersect(static_cast<const ColliderCapsule*>(shape1), static_cast<const ColliderAABB*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Capsule) && (shape2->getType() == ColliderShapeType::Capsule)) {
		return intersect(static_cast<const ColliderCapsule*>(shape1), static_cast<const ColliderCapsule*>(shape2));
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
	std::mutex mutex;

	jobSystem->dispatch(static_cast<uint32_t>(entities.size()), (static_cast<uint32_t>(entities.size()) / jobSystem->getNumThreads()) + 1, [this, &mutex](JobDispatchArguments args) {
		std::set<Entity>::iterator it = entities.begin();
		std::advance(it, args.jobIndex);

		Entity entity = *it;

		const Rigidbody& entityRigidbody = ecs->getComponent<Rigidbody>(entity);

		ColliderShape* colliderShape = nullptr;

		ColliderSphere colliderSphere;
		ColliderAABB colliderAABB;
		ColliderCapsule colliderCapsule;
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
					const Rigidbody& otherEntityRigidbody = ecs->getComponent<Rigidbody>(otherEntity);
					if (entityRigidbody.isStatic && otherEntityRigidbody.isStatic) {
						otherIt++;

						continue;
					}

					ColliderShape* otherColliderShape = nullptr;

					ColliderSphere otherColliderSphere;
					ColliderAABB otherColliderAABB;
					ColliderCapsule otherColliderCapsule;
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

							std::unique_lock<std::mutex> lock(mutex);
							m_collisions.push_back(collision);
							lock.unlock();
						}
					}
				}

				otherIt++;
			}
		}
		});

	jobSystem->wait();
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

		// Impulse
		nml::vec3 relativeVelocity = entity2RigidbodyState.velocity - entity1RigidbodyState.velocity;
		float rVdotN = nml::dot(relativeVelocity, collision.intersectionNormal);

		if (rVdotN >= 0.0f) {
			continue;
		}

		const float e = entity1Rigidbody.restitution * entity2Rigidbody.restitution;
		const float invMass1 = 1.0f / entity1Rigidbody.mass;
		const float invMass2 = 1.0f / entity2Rigidbody.mass;
		const float j = (-(1.0f + e) * rVdotN) / (invMass1 + invMass2);
		const nml::vec3 impulse = j * collision.intersectionNormal;

		if (!entity1Rigidbody.isStatic) {
			entity1RigidbodyState.velocity -= impulse * invMass1;
		}

		if (!entity2Rigidbody.isStatic) {
			entity2RigidbodyState.velocity += impulse * invMass2;
		}

		// Friction
		relativeVelocity = entity2RigidbodyState.velocity - entity1RigidbodyState.velocity;
		rVdotN = nml::dot(relativeVelocity, collision.intersectionNormal);

		nml::vec3 tangent = relativeVelocity - (rVdotN * collision.intersectionNormal);
		if (tangent.length() > 0.0001f) {
			tangent = nml::normalize(tangent);
		}

		const float fVelocity = nml::dot(relativeVelocity, tangent);

		float mu = nml::vec2(entity1Rigidbody.staticFriction, entity2Rigidbody.staticFriction).length();
		const float f = -fVelocity / (invMass1 + invMass2);

		nml::vec3 friction;
		if (std::abs(f) < (j * mu)) {
			friction = f * tangent;
		}
		else {
			mu = nml::vec2(entity1Rigidbody.dynamicFriction, entity2Rigidbody.dynamicFriction).length();
			friction = -j * tangent * mu;
		}

		if (!entity1Rigidbody.isStatic) {
			entity1RigidbodyState.velocity -= friction * invMass1;
		}

		if (!entity2Rigidbody.isStatic) {
			entity2RigidbodyState.velocity += friction * invMass2;
		}

		// Position correction
		const nml::vec3 correction = std::max(collision.intersectionDepth, 0.0f) * collision.intersectionNormal;

		if (!entity1Rigidbody.isStatic) {
			entity1Transform.position[0] -= correction.x;
			entity1Transform.position[1] -= correction.y;
			entity1Transform.position[2] -= correction.z;
		}

		if (!entity2Rigidbody.isStatic) {
			entity2Transform.position[0] += correction.x;
			entity2Transform.position[1] += correction.y;
			entity2Transform.position[2] += correction.z;
		}
	}
	m_collisions.clear();
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderSphere* sphere1, const ColliderSphere* sphere2) {
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
	intersectionInformation.intersectionNormal = { intersectionNormal.x, intersectionNormal.y, intersectionNormal.z };
	intersectionInformation.intersectionDepth = (sphere1->radius + sphere2->radius) - centerDiffLength;

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderSphere* sphere, const ColliderAABB* aabb) {
	IntersectionInformation intersectionInformation;

	const float x = std::max(aabb->min[0], std::min(sphere->center[0], aabb->max[0]));
	const float y = std::max(aabb->min[1], std::min(sphere->center[1], aabb->max[1]));
	const float z = std::max(aabb->min[2], std::min(sphere->center[2], aabb->max[2]));

	const float distance = std::sqrt((x - sphere->center[0]) * (x - sphere->center[0]) +
		(y - sphere->center[1]) * (y - sphere->center[1]) +
		(z - sphere->center[2]) * (z - sphere->center[2]));

	if ((distance < 0.000001f) || (distance >= sphere->radius)) {
		intersectionInformation.hasIntersected = false;

		return intersectionInformation;
	}

	const nml::vec3 intersectionNormal = nml::normalize(nml::vec3(x, y, z) - nml::vec3(sphere->center.data()));

	intersectionInformation.hasIntersected = true;
	intersectionInformation.intersectionNormal = { intersectionNormal.x, intersectionNormal.y, intersectionNormal.z };
	intersectionInformation.intersectionDepth = sphere->radius - distance;

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderSphere* sphere, const ColliderCapsule* capsule) {
	IntersectionInformation intersectionInformation;

	const nml::vec3 sphereCenter = nml::vec3(sphere->center.data());
	const nml::vec3 capsuleBase = nml::vec3(capsule->base.data());
	const nml::vec3 capsuleTip = nml::vec3(capsule->tip.data());

	const nml::vec3 closestPointOnCapsule = closestPointOnSegment(sphereCenter, capsuleBase, capsuleTip);

	ColliderSphere sphereFromCapsule;
	sphereFromCapsule.center = { closestPointOnCapsule.x, closestPointOnCapsule.y, closestPointOnCapsule.z };
	sphereFromCapsule.radius = capsule->radius;

	return intersect(sphere, &sphereFromCapsule);
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderAABB* aabb1, const ColliderAABB* aabb2) {
	IntersectionInformation intersectionInformation;

	const std::array<std::array<float, 3>, 6> normals = {
		std::array<float, 3> { -1.0f, 0.0f, 0.0f },
		{ 1.0f, 0.0f, 0.0f },
		{ 0.0f, -1.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ 0.0f, 0.0f, -1.0f },
		{ 0.0f, 0.0f, 1.0f }
	};

	const std::array<float, 6> distances = {
		aabb2->max[0] - aabb1->min[0],
		aabb1->max[0] - aabb2->min[0],
		aabb2->max[1] - aabb1->min[1],
		aabb1->max[1] - aabb2->min[1],
		aabb2->max[2] - aabb1->min[2],
		aabb1->max[2] - aabb2->min[2]
	};

	uint8_t collidedFace = 0;

	for (uint8_t i = 0; i < 6; i++) {
		if (distances[i] <= 0.0f) {
			intersectionInformation.hasIntersected = false;

			return intersectionInformation;
		}

		if (distances[i] < distances[collidedFace]) {
			collidedFace = i;
		}
	}

	intersectionInformation.hasIntersected = true;
	intersectionInformation.intersectionNormal = normals[collidedFace];
	intersectionInformation.intersectionDepth = distances[collidedFace];

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderAABB* aabb, const ColliderCapsule* capsule) {
	IntersectionInformation intersectionInformation;

	const nml::vec3 aabbCenter = getCenter(aabb);
	const nml::vec3 capsuleBase = nml::vec3(capsule->base.data());
	const nml::vec3 capsuleTip = nml::vec3(capsule->tip.data());

	const nml::vec3 closestPointOnCapsule = closestPointOnSegment(aabbCenter, capsuleBase, capsuleTip);

	ColliderSphere sphereFromCapsule;
	sphereFromCapsule.center = { closestPointOnCapsule.x, closestPointOnCapsule.y, closestPointOnCapsule.z };
	sphereFromCapsule.radius = capsule->radius;

	return intersect(aabb, &sphereFromCapsule);
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule1, const ColliderCapsule* capsule2) {
	IntersectionInformation intersectionInformation;

	const nml::vec3 capsule1Base = nml::vec3(capsule1->base.data());
	const nml::vec3 capsule1Tip = nml::vec3(capsule1->tip.data());
	const nml::vec3 capsule2Base = nml::vec3(capsule2->base.data());
	const nml::vec3 capsule2Tip = nml::vec3(capsule2->tip.data());

	const std::pair<nml::vec3, nml::vec3> bestOnCapsules = closestPointSegmentSegment(capsule1Base, capsule1Tip, capsule2Base, capsule2Tip);

	ColliderSphere sphereFromCapsule1;
	sphereFromCapsule1.center = { bestOnCapsules.first.x, bestOnCapsules.first.y, bestOnCapsules.first.z };
	sphereFromCapsule1.radius = capsule1->radius;

	ColliderSphere sphereFromCapsule2;
	sphereFromCapsule2.center = { bestOnCapsules.second.x, bestOnCapsules.second.y, bestOnCapsules.second.z };
	sphereFromCapsule2.radius = capsule2->radius;

	return intersect(&sphereFromCapsule1, &sphereFromCapsule2);
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule, const ColliderSphere* sphere) {
	IntersectionInformation intersectionInformation = intersect(sphere, capsule);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.intersectionNormal = { intersectionInformation.intersectionNormal[0] * -1.0f, intersectionInformation.intersectionNormal[1] * -1.0f, intersectionInformation.intersectionNormal[2] * -1.0f };
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderAABB* aabb, const ColliderSphere* sphere) {
	IntersectionInformation intersectionInformation = intersect(sphere, aabb);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.intersectionNormal = { intersectionInformation.intersectionNormal[0] * -1.0f, intersectionInformation.intersectionNormal[1] * -1.0f, intersectionInformation.intersectionNormal[2] * -1.0f };
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule, const ColliderAABB* aabb) {
	IntersectionInformation intersectionInformation = intersect(aabb, capsule);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.intersectionNormal = { intersectionInformation.intersectionNormal[0] * -1.0f, intersectionInformation.intersectionNormal[1] * -1.0f, intersectionInformation.intersectionNormal[2] * -1.0f };
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::gjk(const ColliderShape* shape1, const ColliderShape* shape2) {
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

	NTSHENGN_MODULE_ERROR("Reached impossible path.", Result::ModuleError);
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

nml::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderShape* shape) {
	if (shape->getType() == ColliderShapeType::Sphere) {
		return getCenter(static_cast<const ColliderSphere*>(shape));
	}
	else if (shape->getType() == ColliderShapeType::AABB) {
		return getCenter(static_cast<const ColliderAABB*>(shape));
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		return getCenter(static_cast<const ColliderCapsule*>(shape));
	}

	return nml::vec3(0.0f, 0.0f, 0.0f);
}

nml::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderSphere* sphere) {
	return nml::vec3(sphere->center.data());
}

nml::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderAABB* aabb) {
	return nml::vec3((aabb->min[0] + aabb->max[0]) / 2.0f, (aabb->min[1] + aabb->max[1]) / 2.0f, (aabb->min[2] + aabb->max[2]) / 2.0f);
}

nml::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderCapsule* capsule) {
	return nml::vec3((capsule->base[0] + capsule->tip[0]) / 2.0f, (capsule->base[1] + capsule->tip[1]) / 2.0f, (capsule->base[2] + capsule->tip[2]) / 2.0f);
}

nml::vec3 NtshEngn::PhysicsModule::support(const ColliderShape* shape1, const ColliderShape* shape2, const nml::vec3& direction) {
	const nml::vec3 p1 = getFarthestPointInDirection(shape1, direction);
	const nml::vec3 p2 = getFarthestPointInDirection(shape2, direction * -1.0f);

	return p1 - p2;
}

nml::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderShape* shape, const nml::vec3& direction) {
	if (shape->getType() == ColliderShapeType::Sphere) {
		return getFarthestPointInDirection(static_cast<const ColliderSphere*>(shape), direction);
	}
	else if (shape->getType() == ColliderShapeType::AABB) {
		return getFarthestPointInDirection(static_cast<const ColliderAABB*>(shape), direction);
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		return getFarthestPointInDirection(static_cast<const ColliderCapsule*>(shape), direction);
	}

	return nml::vec3(0.0f, 0.0f, 0.0f);
}

nml::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderSphere* sphere, const nml::vec3& direction) {
	const float directionLength = direction.length();

	return nml::vec3(sphere->center.data()) + direction * (sphere->radius / directionLength);
}

nml::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderAABB* aabb, const nml::vec3& direction) {
	return nml::vec3(direction.x >= 0.0f ? aabb->max[0] : aabb->min[0],
		direction.y >= 0.0f ? aabb->max[1] : aabb->min[1],
		direction.z >= 0.0f ? aabb->max[2] : aabb->min[2]);
}

nml::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderCapsule* capsule, const nml::vec3& direction) {
	const float directionLength = direction.length();
	const nml::vec3 capsuleBase = nml::vec3(capsule->base.data());
	const nml::vec3 capsuleTip = nml::vec3(capsule->tip.data());

	return ((nml::dot(direction, nml::vec3(capsuleTip - capsuleBase)) >= 0.0) ? capsuleTip : capsuleBase) + direction * (capsule->radius / directionLength);
}

std::pair<nml::vec3, float> NtshEngn::PhysicsModule::epa(const ColliderShape* shape1, const ColliderShape* shape2, GJKSimplex& simplex) {
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

void NtshEngn::PhysicsModule::transform(ColliderShape* shape, const nml::vec3& translation, const nml::vec3& rotation, const nml::vec3& scale) {
	if (shape->getType() == ColliderShapeType::Sphere) {
		transform(static_cast<ColliderSphere*>(shape), translation, scale);
	}
	else if (shape->getType() == ColliderShapeType::AABB) {
		transform(static_cast<ColliderAABB*>(shape), translation, rotation, scale);
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		transform(static_cast<ColliderCapsule*>(shape), translation, rotation, scale);
	}
}

void NtshEngn::PhysicsModule::transform(ColliderSphere* sphere, const nml::vec3& translation, const nml::vec3& scale) {
	sphere->center = { translation.x, translation.y, translation.z };
	sphere->radius *= std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z)));
}

void NtshEngn::PhysicsModule::transform(ColliderAABB* aabb, const nml::vec3& translation, const nml::vec3& rotation, const nml::vec3& scale) {
	ColliderAABB newAABB;
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

void NtshEngn::PhysicsModule::transform(ColliderCapsule* capsule, const nml::vec3& translation, const nml::vec3& rotation, const nml::vec3& scale) {
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

nml::vec3 NtshEngn::PhysicsModule::closestPointOnSegment(const nml::vec3& point, const nml::vec3& segmentA, const nml::vec3& segmentB) {
	const nml::vec3 ab = segmentB - segmentA;
	const nml::vec3 ap = point - segmentA;

	const float e = dot(ap, ab) / dot(ab, ab);

	return segmentA + (std::min(std::max(e, 0.0f), 1.0f) * ab);
}

std::pair<nml::vec3, nml::vec3> NtshEngn::PhysicsModule::closestPointSegmentSegment(const nml::vec3& segmentA1, const nml::vec3& segmentA2, const nml::vec3& segmentB1, const nml::vec3& segmentB2) {
	const nml::vec3 segmentA = segmentA2 - segmentA1;
	const nml::vec3 segmentB = segmentB2 - segmentB1;
	const nml::vec3 r = segmentA1 - segmentB1;
	const float segmentASqLength = dot(segmentA, segmentA);
	const float segmentBSqLength = dot(segmentB, segmentB);
	const float f = dot(segmentB, r);

	float s = 0.0f;
	float t = 0.0f;

	if ((segmentASqLength <= 0.000001f) && (segmentBSqLength <= 0.000001f)) {
		return std::pair<nml::vec3, nml::vec3>(segmentA1, segmentB1);
	}

	if (segmentASqLength <= 0.000001f) {
		s = 0.0f;
		t = f / segmentBSqLength;
		t = std::max(std::min(t, 1.0f), 0.0f);
	}
	else {
		const float c = nml::dot(segmentA, r);

		if (segmentBSqLength <= 0.000001f) {
			t = 0.0f;
			s = -c / segmentASqLength;
			s = std::max(std::min(s, 1.0f), 0.0f);
		}
		else {
			const float b = nml::dot(segmentA, segmentB);

			const float denom = (segmentASqLength * segmentBSqLength) - (b * b);
			if (denom != 0.0f) {
				s = ((b * f) - (c * segmentBSqLength)) / denom;
				s = std::max(std::min(s, 1.0f), 0.0f);
			}
			else {
				s = 0.0f;
			}

			t = ((b * s) + f) / segmentBSqLength;
			if (t < 0.0f) {
				t = 0.0f;
				s = -c / segmentASqLength;
				s = std::max(std::min(s, 1.0f), 0.0f);
			}
			else if (t > 1.0f) {
				t = 1.0f;
				s = (b - c) / segmentASqLength;
				s = std::max(std::min(s, 1.0f), 0.0f);
			}
		}
	}

	return std::pair<nml::vec3, nml::vec3>(segmentA1 + (segmentA * s), segmentB1 + (segmentB * t));
}

extern "C" NTSHENGN_MODULE_API NtshEngn::PhysicsModuleInterface* createModule() {
	return new NtshEngn::PhysicsModule;
}

extern "C" NTSHENGN_MODULE_API void destroyModule(NtshEngn::PhysicsModuleInterface* m) {
	delete m;
}