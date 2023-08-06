#include "ntshengn_physics_module.h"
#include "../Module/utils/ntshengn_module_defines.h"
#include "../Module/utils/ntshengn_dynamic_library.h"
#include "../Common/utils/ntshengn_defines.h"
#include "../Common/utils/ntshengn_enums.h"
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

			entityRigidbodyState.acceleration = Math::vec3(entityRigidbody.force.data()) / entityRigidbody.mass;
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
			transform(colliderShape, entityTransform.position, entityTransform.rotation, entityTransform.scale);

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
						transform(otherColliderShape, otherEntityTransform.position, otherEntityTransform.rotation, otherEntityTransform.scale);

						IntersectionInformation intersectionInformation = intersect(colliderShape, otherColliderShape);
						if (intersectionInformation.hasIntersected) {
							Collision collision;
							collision.entity1 = entity;
							collision.entity2 = otherEntity;
							collision.intersectionNormal = Math::vec3(intersectionInformation.intersectionNormal.data());
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
		Math::vec3 relativeVelocity = entity2RigidbodyState.velocity - entity1RigidbodyState.velocity;
		float rVdotN = Math::dot(relativeVelocity, collision.intersectionNormal);

		if (rVdotN >= 0.0f) {
			continue;
		}

		const float e = entity1Rigidbody.restitution * entity2Rigidbody.restitution;
		const float invMass1 = 1.0f / entity1Rigidbody.mass;
		const float invMass2 = 1.0f / entity2Rigidbody.mass;
		const float j = (-(1.0f + e) * rVdotN) / (invMass1 + invMass2);
		const Math::vec3 impulse = j * collision.intersectionNormal;

		if (!entity1Rigidbody.isStatic) {
			entity1RigidbodyState.velocity -= impulse * invMass1;
		}

		if (!entity2Rigidbody.isStatic) {
			entity2RigidbodyState.velocity += impulse * invMass2;
		}

		// Friction
		relativeVelocity = entity2RigidbodyState.velocity - entity1RigidbodyState.velocity;
		rVdotN = Math::dot(relativeVelocity, collision.intersectionNormal);

		Math::vec3 tangent = relativeVelocity - (rVdotN * collision.intersectionNormal);
		if (tangent.length() > 0.0001f) {
			tangent = Math::normalize(tangent);
		}

		const float fVelocity = Math::dot(relativeVelocity, tangent);

		float mu = Math::vec2(entity1Rigidbody.staticFriction, entity2Rigidbody.staticFriction).length();
		const float f = -fVelocity / (invMass1 + invMass2);

		Math::vec3 friction;
		if (std::abs(f) < (j * mu)) {
			friction = f * tangent;
		}
		else {
			mu = Math::vec2(entity1Rigidbody.dynamicFriction, entity2Rigidbody.dynamicFriction).length();
			friction = -j * tangent * mu;
		}

		if (!entity1Rigidbody.isStatic) {
			entity1RigidbodyState.velocity -= friction * invMass1;
		}

		if (!entity2Rigidbody.isStatic) {
			entity2RigidbodyState.velocity += friction * invMass2;
		}

		// Position correction
		const Math::vec3 correction = std::max(collision.intersectionDepth, 0.0f) * collision.intersectionNormal;

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

	const Math::vec3 centerDiff = sphere2->center - sphere1->center;
	const float centerDiffLength = centerDiff.length();

	if ((centerDiffLength < 0.000001f) || (centerDiffLength >= (sphere1->radius + sphere2->radius))) {
		intersectionInformation.hasIntersected = false;
		
		return intersectionInformation;
	}

	intersectionInformation.hasIntersected = true;
	intersectionInformation.intersectionNormal = Math::normalize(centerDiff);
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

	intersectionInformation.hasIntersected = true;
	intersectionInformation.intersectionNormal = Math::normalize(Math::vec3(x, y, z) - sphere->center);
	intersectionInformation.intersectionDepth = sphere->radius - distance;

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderSphere* sphere, const ColliderCapsule* capsule) {
	IntersectionInformation intersectionInformation;

	ColliderSphere sphereFromCapsule;
	sphereFromCapsule.center = closestPointOnSegment(sphere->center, capsule->base, capsule->tip);
	sphereFromCapsule.radius = capsule->radius;

	return intersect(sphere, &sphereFromCapsule);
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderAABB* aabb1, const ColliderAABB* aabb2) {
	IntersectionInformation intersectionInformation;

	const std::array<Math::vec3, 6> normals = {
		Math::vec3(-1.0f, 0.0f, 0.0f),
		Math::vec3(1.0f, 0.0f, 0.0f),
		Math::vec3(0.0f, -1.0f, 0.0f),
		Math::vec3(0.0f, 1.0f, 0.0f),
		Math::vec3(0.0f, 0.0f, -1.0f),
		Math::vec3(0.0f, 0.0f, 1.0f)
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

	const Math::vec3 aabbCenter = getCenter(aabb);

	const Math::vec3 closestPointOnCapsule = closestPointOnSegment(aabbCenter, capsule->base, capsule->tip);

	ColliderSphere sphereFromCapsule;
	sphereFromCapsule.center = { closestPointOnCapsule.x, closestPointOnCapsule.y, closestPointOnCapsule.z };
	sphereFromCapsule.radius = capsule->radius;

	return intersect(aabb, &sphereFromCapsule);
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule1, const ColliderCapsule* capsule2) {
	IntersectionInformation intersectionInformation;

	const std::pair<Math::vec3, Math::vec3> bestOnCapsules = closestPointSegmentSegment(capsule1->base, capsule1->tip, capsule2->base, capsule2->tip);

	ColliderSphere sphereFromCapsule1;
	sphereFromCapsule1.center = bestOnCapsules.first;
	sphereFromCapsule1.radius = capsule1->radius;

	ColliderSphere sphereFromCapsule2;
	sphereFromCapsule2.center = bestOnCapsules.second;
	sphereFromCapsule2.radius = capsule2->radius;

	return intersect(&sphereFromCapsule1, &sphereFromCapsule2);
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule, const ColliderSphere* sphere) {
	IntersectionInformation intersectionInformation = intersect(sphere, capsule);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.intersectionNormal = intersectionInformation.intersectionNormal * -1.0f;
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderAABB* aabb, const ColliderSphere* sphere) {
	IntersectionInformation intersectionInformation = intersect(sphere, aabb);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.intersectionNormal = intersectionInformation.intersectionNormal * -1.0;
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule, const ColliderAABB* aabb) {
	IntersectionInformation intersectionInformation = intersect(aabb, capsule);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.intersectionNormal = intersectionInformation.intersectionNormal * -1.0;
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::gjk(const ColliderShape* shape1, const ColliderShape* shape2) {
	IntersectionInformation intersectionInformation;

	GJKSimplex simplex;

	Math::vec3 direction = getCenter(shape2) - getCenter(shape1);

	Math::vec3 supp = support(shape1, shape2, direction);
	simplex.push_front(supp);

	direction = supp * -1.0f;

	while (true) {
		supp = support(shape1, shape2, direction);

		if (Math::dot(supp, direction) <= 0.0f) { // No intersection
			intersectionInformation.hasIntersected = false;

			return intersectionInformation;
		}

		simplex.push_front(supp);

		if (simplexContainsOrigin(simplex, direction)) { // Intersection
			intersectionInformation.hasIntersected = true;

			const std::pair<Math::vec3, float> intersectionNormalAndDepth = epa(shape1, shape2, simplex);
			intersectionInformation.intersectionNormal = intersectionNormalAndDepth.first;
			intersectionInformation.intersectionDepth = intersectionNormalAndDepth.second + 0.001f;

			return intersectionInformation;
		}
	}

	NTSHENGN_MODULE_ERROR("Reached impossible path in GJK.", Result::ModuleError);
}

bool NtshEngn::PhysicsModule::simplexContainsOrigin(GJKSimplex& simplex, Math::vec3& direction) {
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

bool NtshEngn::PhysicsModule::gjkLine(GJKSimplex& simplex, Math::vec3& direction) {
	const Math::vec3 a = simplex[0];
	const Math::vec3 b = simplex[1];

	const Math::vec3 ab = b - a;
	const Math::vec3 ao = a * -1.0f;

	if (sameDirection(ab, ao)) {
		direction = Math::cross(Math::cross(ab, ao), ab);
	}
	else {
		simplex = { a };
		direction = ao;
	}

	return false;
}

bool NtshEngn::PhysicsModule::gjkTriangle(GJKSimplex& simplex, Math::vec3& direction) {
	const Math::vec3 a = simplex[0];
	const Math::vec3 b = simplex[1];
	const Math::vec3 c = simplex[2];

	const Math::vec3 ab = b - a;
	const Math::vec3 ac = c - a;
	const Math::vec3 ao = a * -1.0f;

	const Math::vec3 abc = Math::cross(ab, ac);

	if (sameDirection(Math::cross(abc, ac), ao)) {
		if (sameDirection(ac, ao)) {
			simplex = { a, c };
			direction = Math::cross(Math::cross(ac, ao), ac);
		}
		else {
			simplex = { a, b };
			return gjkLine(simplex, direction);
		}
	}
	else {
		if (sameDirection(Math::cross(ab, abc), ao)) {
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

bool NtshEngn::PhysicsModule::gjkTetrahedron(GJKSimplex& simplex, Math::vec3& direction) {
	const Math::vec3 a = simplex[0];
	const Math::vec3 b = simplex[1];
	const Math::vec3 c = simplex[2];
	const Math::vec3 d = simplex[3];

	const Math::vec3 ab = b - a;
	const Math::vec3 ac = c - a;
	const Math::vec3 ad = d - a;
	const Math::vec3 ao = a * -1.0f;

	const Math::vec3 abc = Math::cross(ab, ac);
	const Math::vec3 acd = Math::cross(ac, ad);
	const Math::vec3 adb = Math::cross(ad, ab);

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

bool NtshEngn::PhysicsModule::sameDirection(const Math::vec3& a, const Math::vec3& b) {
	return Math::dot(a, b) > 0.0f;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderShape* shape) {
	if (shape->getType() == ColliderShapeType::Sphere) {
		return getCenter(static_cast<const ColliderSphere*>(shape));
	}
	else if (shape->getType() == ColliderShapeType::AABB) {
		return getCenter(static_cast<const ColliderAABB*>(shape));
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		return getCenter(static_cast<const ColliderCapsule*>(shape));
	}

	return Math::vec3(0.0f, 0.0f, 0.0f);
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderSphere* sphere) {
	return sphere->center;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderAABB* aabb) {
	return (aabb->min + aabb->max) / 2.0f;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderCapsule* capsule) {
	return (capsule->base + capsule->tip) / 2.0f;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::support(const ColliderShape* shape1, const ColliderShape* shape2, const Math::vec3& direction) {
	const Math::vec3 p1 = getFarthestPointInDirection(shape1, direction);
	const Math::vec3 p2 = getFarthestPointInDirection(shape2, direction * -1.0f);

	return p1 - p2;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderShape* shape, const Math::vec3& direction) {
	if (shape->getType() == ColliderShapeType::Sphere) {
		return getFarthestPointInDirection(static_cast<const ColliderSphere*>(shape), direction);
	}
	else if (shape->getType() == ColliderShapeType::AABB) {
		return getFarthestPointInDirection(static_cast<const ColliderAABB*>(shape), direction);
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		return getFarthestPointInDirection(static_cast<const ColliderCapsule*>(shape), direction);
	}

	return Math::vec3(0.0f, 0.0f, 0.0f);
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderSphere* sphere, const Math::vec3& direction) {
	const float directionLength = direction.length();

	return sphere->center + direction * (sphere->radius / directionLength);
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderAABB* aabb, const Math::vec3& direction) {
	return Math::vec3(direction.x >= 0.0f ? aabb->max.x : aabb->min.x,
		direction.y >= 0.0f ? aabb->max.y : aabb->min.y,
		direction.z >= 0.0f ? aabb->max.z : aabb->min.z);
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderCapsule* capsule, const Math::vec3& direction) {
	const float directionLength = direction.length();

	return ((Math::dot(direction, Math::vec3(capsule->tip - capsule->base)) >= 0.0) ? capsule->tip : capsule->base) + direction * (capsule->radius / directionLength);
}

std::pair<NtshEngn::Math::vec3, float> NtshEngn::PhysicsModule::epa(const ColliderShape* shape1, const ColliderShape* shape2, GJKSimplex& simplex) {
	std::vector<Math::vec3> polytope(simplex.begin(), simplex.end());
	std::vector<size_t> faces = {
		0, 1, 2,
		0, 3, 1,
		0, 2, 3,
		1, 3, 2
	};
	std::pair<std::vector<Math::vec4>, size_t> polytopeNormals = getPolytopeNormals(polytope, faces);
	std::vector<Math::vec4> normals = polytopeNormals.first;
	size_t minTriangle = polytopeNormals.second;

	Math::vec3 minNormal;
	float minDistance = std::numeric_limits<float>::max();
	while (minDistance == std::numeric_limits<float>::max()) {
		minNormal = Math::vec3(normals[minTriangle]);
		minDistance = normals[minTriangle].w;

		const Math::vec3 supp = support(shape1, shape2, minNormal);
		const float sDistance = Math::dot(minNormal, supp);
		if (std::abs(sDistance - minDistance) > 0.001f) {
			minDistance = std::numeric_limits<float>::max();

			std::vector<std::pair<size_t, size_t>> uniqueEdges;
			for (size_t i = 0; i < normals.size(); i++) {
				size_t f = i * 3;
				if (sameDirection(Math::vec3(normals[i]), supp - polytope[faces[f]])) {
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

			std::pair<std::vector<Math::vec4>, size_t> newFacesNormals = getPolytopeNormals(polytope, newFaces);
			const std::vector<Math::vec4> newNormals = newFacesNormals.first;
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

std::pair<std::vector<NtshEngn::Math::vec4>, size_t> NtshEngn::PhysicsModule::getPolytopeNormals(const std::vector<Math::vec3>& polytope, const std::vector<size_t>& faces) {
	std::vector<Math::vec4> normals;
	size_t minTriangle = 0;
	float minDistance = std::numeric_limits<float>::max();

	for (size_t i = 0; i < faces.size(); i += 3) {
		const Math::vec3 a = polytope[faces[i]];
		const Math::vec3 b = polytope[faces[i + 1]];
		const Math::vec3 c = polytope[faces[i + 2]];

		const Math::vec3 ab = b - a;
		const Math::vec3 ac = c - a;

		Math::vec3 n = Math::normalize(Math::cross(ab, ac));
		float distance = Math::dot(n, a);
		if (distance < 0.0f) {
			n *= -1;
			distance *= -1;
		}

		normals.push_back(Math::vec4(n, distance));

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

void NtshEngn::PhysicsModule::transform(ColliderShape* shape, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
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

void NtshEngn::PhysicsModule::transform(ColliderSphere* sphere, const Math::vec3& translation, const Math::vec3& scale) {
	sphere->center = translation;
	sphere->radius *= std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z)));
}

void NtshEngn::PhysicsModule::transform(ColliderAABB* aabb, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
	ColliderAABB newAABB;
	newAABB.min = translation;
	newAABB.max = translation;

	float a;
	float b;

	const Math::mat4 rotationMatrix = Math::rotate(rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(rotation.z, Math::vec3(0.0f, 0.0f, 1.0f));

	for (uint8_t i = 0; i < 3; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			a = rotationMatrix[j][i] * aabb->min[j] * std::abs(scale[i]);
			b = rotationMatrix[j][i] * aabb->max[j] * std::abs(scale[i]);
			
			newAABB.min[i] += (a < b) ? a : b;
			newAABB.max[i] += (a < b) ? b : a;
		}
	}

	aabb->min = newAABB.min;
	aabb->max = newAABB.max;
}

void NtshEngn::PhysicsModule::transform(ColliderCapsule* capsule, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
	capsule->base += translation;
	capsule->tip += translation;

	const Math::mat4 rotationMatrix = Math::rotate(rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(rotation.z, Math::vec3(0.0f, 0.0f, 1.0f));
	const Math::vec3 tipMinusBase = Math::vec3(capsule->tip.data()) - Math::vec3(capsule->base.data());

	const Math::vec3 tipRotation = Math::vec3(rotationMatrix * Math::vec4(tipMinusBase, 1.0f));

	capsule->tip = tipRotation + capsule->base;
	capsule->radius *= std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z)));
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::closestPointOnSegment(const Math::vec3& point, const Math::vec3& segmentA, const Math::vec3& segmentB) {
	const Math::vec3 ab = segmentB - segmentA;
	const Math::vec3 ap = point - segmentA;

	const float e = dot(ap, ab) / dot(ab, ab);

	return segmentA + (std::min(std::max(e, 0.0f), 1.0f) * ab);
}

std::pair<NtshEngn::Math::vec3, NtshEngn::Math::vec3> NtshEngn::PhysicsModule::closestPointSegmentSegment(const Math::vec3& segmentA1, const Math::vec3& segmentA2, const Math::vec3& segmentB1, const Math::vec3& segmentB2) {
	const Math::vec3 segmentA = segmentA2 - segmentA1;
	const Math::vec3 segmentB = segmentB2 - segmentB1;
	const Math::vec3 r = segmentA1 - segmentB1;
	const float segmentASqLength = dot(segmentA, segmentA);
	const float segmentBSqLength = dot(segmentB, segmentB);
	const float f = dot(segmentB, r);

	float s = 0.0f;
	float t = 0.0f;

	if ((segmentASqLength <= 0.000001f) && (segmentBSqLength <= 0.000001f)) {
		return std::pair<Math::vec3, Math::vec3>(segmentA1, segmentB1);
	}

	if (segmentASqLength <= 0.000001f) {
		s = 0.0f;
		t = f / segmentBSqLength;
		t = std::max(std::min(t, 1.0f), 0.0f);
	}
	else {
		const float c = Math::dot(segmentA, r);

		if (segmentBSqLength <= 0.000001f) {
			t = 0.0f;
			s = -c / segmentASqLength;
			s = std::max(std::min(s, 1.0f), 0.0f);
		}
		else {
			const float b = Math::dot(segmentA, segmentB);

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

	return std::pair<Math::vec3, Math::vec3>(segmentA1 + (segmentA * s), segmentB1 + (segmentB * t));
}

extern "C" NTSHENGN_MODULE_API NtshEngn::PhysicsModuleInterface* createModule() {
	return new NtshEngn::PhysicsModule;
}

extern "C" NTSHENGN_MODULE_API void destroyModule(NtshEngn::PhysicsModuleInterface* m) {
	delete m;
}