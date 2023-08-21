#include "ntshengn_physics_module.h"
#include "../Module/utils/ntshengn_module_defines.h"
#include "../Module/utils/ntshengn_dynamic_library.h"
#include "../Common/utils/ntshengn_defines.h"
#include "../Common/utils/ntshengn_enums.h"
#include "../Common/utils/ntshengn_utils_octree.h"
#include <algorithm>
#include <limits>
#include <cmath>

void NtshEngn::PhysicsModule::init() {
}

void NtshEngn::PhysicsModule::update(double dt) {
	if (dt > (1000.0 / 60.0)) {
		dt = 1000.0 / 60.0;
	}
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

std::vector<NtshEngn::RaycastInformation> NtshEngn::PhysicsModule::raycast(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax) {
	std::vector<RaycastInformation> raycastInformations;

	const Math::vec3 invRayDirection = Math::vec3(1.0f / rayDirection.x, 1.0f / rayDirection.y, 1.0f / rayDirection.z);

	for (Entity entity : entities) {
		const Transform& entityTransform = ecs->getComponent<Transform>(entity);

		if (ecs->hasComponent<SphereCollidable>(entity)) {
			ColliderSphere colliderSphere = ecs->getComponent<SphereCollidable>(entity).collider;
			transform(&colliderSphere, entityTransform.position, entityTransform.rotation, entityTransform.scale);

			const Math::vec3 co = rayOrigin - colliderSphere.center;
			const float a = Math::dot(rayDirection, rayDirection);
			const float b = 2.0f * Math::dot(co, rayDirection);
			const float c = Math::dot(co, co) - (colliderSphere.radius * colliderSphere.radius);
			const float discriminant = (b * b) - (4.0f * a * c);

			if (discriminant < 0.0f) {
				continue;
			}

			const float distance = (-b - (std::sqrt(discriminant))) / (2.0f * a);
			if ((distance >= tMin) && (distance <= tMax)) {
				RaycastInformation raycastInformation;
				raycastInformation.entity = entity;
				raycastInformation.distance = distance;
				raycastInformation.normal = Math::normalize((rayOrigin + (rayDirection * distance)) - colliderSphere.center);
				raycastInformations.push_back(raycastInformation);
			}
		}
		else if (ecs->hasComponent<AABBCollidable>(entity)) {
			ColliderAABB colliderAABB = ecs->getComponent<AABBCollidable>(entity).collider;
			transform(&colliderAABB, entityTransform.position, entityTransform.rotation, entityTransform.scale);

			const float t1 = (colliderAABB.min.x - rayOrigin.x) * invRayDirection.x;
			const float t2 = (colliderAABB.max.x - rayOrigin.x) * invRayDirection.x;
			const float t3 = (colliderAABB.min.y - rayOrigin.y) * invRayDirection.y;
			const float t4 = (colliderAABB.max.y - rayOrigin.y) * invRayDirection.y;
			const float t5 = (colliderAABB.min.z - rayOrigin.z) * invRayDirection.z;
			const float t6 = (colliderAABB.max.z - rayOrigin.z) * invRayDirection.z;

			const float distanceMin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
			const float distanceMax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));
			if ((distanceMax >= 0.0f) && (distanceMin <= distanceMax) && ((distanceMin >= tMin) && (distanceMin <= tMax))) {
				Math::vec3 normal;
				if (distanceMin == t1) {
					normal = Math::vec3(-1.0f, 0.0f, 0.0f);
				}
				else if (distanceMin == t2) {
					normal = Math::vec3(1.0f, 0.0f, 0.0f);
				}
				else if (distanceMin == t3) {
					normal = Math::vec3(0.0f, -1.0f, 0.0f);
				}
				else if (distanceMin == t4) {
					normal = Math::vec3(0.0f, 1.0f, 0.0f);
				}
				else if (distanceMin == t5) {
					normal = Math::vec3(0.0f, 0.0f, -1.0f);
				}
				else if (distanceMin == t6) {
					normal = Math::vec3(0.0f, 0.0f, 1.0f);
				}

				RaycastInformation raycastInformation;
				raycastInformation.entity = entity;
				raycastInformation.distance = distanceMin;
				raycastInformation.normal = normal;
				raycastInformations.push_back(raycastInformation);
			}
		}
		else if (ecs->hasComponent<CapsuleCollidable>(entity)) {
			ColliderCapsule colliderCapsule = ecs->getComponent<CapsuleCollidable>(entity).collider;
			transform(&colliderCapsule, entityTransform.position, entityTransform.rotation, entityTransform.scale);

			const Math::vec3 ab = colliderCapsule.tip - colliderCapsule.base;
			const Math::vec3 ao = rayOrigin - colliderCapsule.base;

			const float abab = Math::dot(ab, ab);
			const float aoao = Math::dot(ao, ao);
			const float abrd = Math::dot(ab, rayDirection);
			const float abao = Math::dot(ab, ao);
			const float rdao = Math::dot(rayDirection, ao);

			const float a = abab - (abrd * abrd);
			float b = (abab * rdao) - (abao * abrd);
			float c = (abab * aoao) - (abao * abao) - (colliderCapsule.radius * colliderCapsule.radius * abab);
			float h = (b * b) - (a * c);
			if (h >= 0.0f) {
				float distance = (-b - std::sqrt(h)) / a;
				const float y = abao + (distance * abrd);
				if ((y > 0.0) && (y < abab) && ((distance >= tMin) && (distance <= tMax))) {
					const Math::vec3 position = rayOrigin + (rayDirection * distance);
					const Math::vec3 ap = position - colliderCapsule.base;

					RaycastInformation raycastInformation;
					raycastInformation.entity = entity;
					raycastInformation.distance = distance;
					raycastInformation.normal = (ap - (ab * std::clamp(Math::dot(ap, ab) / Math::dot(ab, ab), 0.0f, 1.0f))) / colliderCapsule.radius;
					raycastInformations.push_back(raycastInformation);

					continue;
				}

				const Math::vec3 co = (y <= 0.0f) ? ao : (rayOrigin - colliderCapsule.tip);
				b = Math::dot(rayDirection, co);
				c = Math::dot(co, co) - (colliderCapsule.radius * colliderCapsule.radius);

				h = (b * b) - c;
				distance = -b - std::sqrt(h);
				if ((h > 0.0f) && ((distance >= tMin) && (distance <= tMax))) {
					const Math::vec3 position = rayOrigin + (rayDirection * distance);
					const Math::vec3 ap = position - colliderCapsule.base;

					RaycastInformation raycastInformation;
					raycastInformation.entity = entity;
					raycastInformation.distance = distance;
					raycastInformation.normal = (ap - (ab * std::clamp(Math::dot(ap, ab) / Math::dot(ab, ab), 0.0f, 1.0f))) / colliderCapsule.radius;
					raycastInformations.push_back(raycastInformation);
				}
			}
		}
	}

	std::sort(raycastInformations.begin(), raycastInformations.end(), [](RaycastInformation raycastInformationA, RaycastInformation raycastInformationB) {
		return raycastInformationA.distance < raycastInformationB.distance;
		});

	return raycastInformations;
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

			entityRigidbodyState.acceleration = entityRigidbody.force / entityRigidbody.mass;
			if (entityRigidbody.isAffectedByConstants) {
				entityRigidbodyState.acceleration += m_gravity;
			}

			entityRigidbodyState.velocity += entityRigidbodyState.acceleration * dtSeconds;

			entityTransform.position += entityRigidbodyState.velocity * dtSeconds;
		}
		else {
			entityRigidbodyState.velocity = { 0.0f, 0.0f, 0.0f };
		}

		entityRigidbody.force = { 0.0f, 0.0f, 0.0f };
	}
}

void NtshEngn::PhysicsModule::collisionsDetection() {
	collisionsBroadphase();
	collisionsNarrowphase();
}

void NtshEngn::PhysicsModule::collisionsResponse() {
	for (const NarrowphaseCollision& collision : m_narrowphaseCollisions) {
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
			entity1Transform.position -= correction;
		}

		if (!entity2Rigidbody.isStatic) {
			entity2Transform.position += correction;
		}
	}
}

void NtshEngn::PhysicsModule::collisionsBroadphase() {
	m_broadphaseCollisions.clear();

	ColliderAABB sceneAABB;

	std::unordered_map<Entity, EntityAABB> entityAABBs;
	for (const Entity& entity : entities) {
		EntityAABB entityAABB;

		const Transform& entityTransform = ecs->getComponent<Transform>(entity);

		if (ecs->hasComponent<SphereCollidable>(entity)) {
			ColliderSphere colliderSphere = ecs->getComponent<SphereCollidable>(entity).collider;
			transform(&colliderSphere, entityTransform.position, entityTransform.rotation, entityTransform.scale);

			entityAABB.position = colliderSphere.center;
			entityAABB.size = Math::vec3(colliderSphere.radius);
		}
		else if (ecs->hasComponent<AABBCollidable>(entity)) {
			ColliderAABB colliderAABB = ecs->getComponent<AABBCollidable>(entity).collider;
			transform(&colliderAABB, entityTransform.position, entityTransform.rotation, entityTransform.scale);

			entityAABB.position = getCenter(&colliderAABB);
			entityAABB.size = (colliderAABB.max - colliderAABB.min) / 2.0f;
		}
		else if (ecs->hasComponent<CapsuleCollidable>(entity)) {
			ColliderCapsule colliderCapsule = ecs->getComponent<CapsuleCollidable>(entity).collider;
			transform(&colliderCapsule, entityTransform.position, entityTransform.rotation, entityTransform.scale);

			entityAABB.position = getCenter(&colliderCapsule);

			ColliderAABB baseAABB;
			baseAABB.min = colliderCapsule.base - Math::vec3(colliderCapsule.radius);
			baseAABB.max = colliderCapsule.base + Math::vec3(colliderCapsule.radius);

			ColliderAABB tipAABB;
			tipAABB.min = colliderCapsule.tip - Math::vec3(colliderCapsule.radius);
			tipAABB.max = colliderCapsule.tip + Math::vec3(colliderCapsule.radius);

			ColliderAABB capsuleAABB;
			capsuleAABB.min = Math::vec3(std::min(baseAABB.min.x, tipAABB.min.x), std::min(baseAABB.min.y, tipAABB.min.y), std::min(baseAABB.min.z, tipAABB.min.z));
			capsuleAABB.max = Math::vec3(std::max(baseAABB.max.x, tipAABB.max.x), std::max(baseAABB.max.y, tipAABB.max.y), std::max(baseAABB.max.z, tipAABB.max.z));

			entityAABB.size = (capsuleAABB.max - capsuleAABB.min) / 2.0f;
		}
		else {
			continue;
		}

		entityAABBs[entity] = entityAABB;

		if ((entityAABB.position.x - entityAABB.size.x) < sceneAABB.min.x) {
			sceneAABB.min.x = entityAABB.position.x - entityAABB.size.x;
		}
		if ((entityAABB.position.y - entityAABB.size.y) < sceneAABB.min.y) {
			sceneAABB.min.y = entityAABB.position.y - entityAABB.size.y;
		}
		if ((entityAABB.position.z - entityAABB.size.z) < sceneAABB.min.z) {
			sceneAABB.min.z = entityAABB.position.z - entityAABB.size.z;
		}

		if ((entityAABB.position.x + entityAABB.size.x) > sceneAABB.max.x) {
			sceneAABB.max.x = entityAABB.position.x + entityAABB.size.x;
		}
		if ((entityAABB.position.y + entityAABB.size.y) > sceneAABB.max.y) {
			sceneAABB.max.y = entityAABB.position.y + entityAABB.size.y;
		}
		if ((entityAABB.position.z + entityAABB.size.z) > sceneAABB.max.z) {
			sceneAABB.max.z = entityAABB.position.z + entityAABB.size.z;
		}
	}

	Octree<Entity> octree = Octree<Entity>(getCenter(&sceneAABB), ((sceneAABB.max - sceneAABB.min) / 2.0f) * 100.0f, 6);
	for (const auto& it : entityAABBs) {
		octree.insert(it.first, it.second.position, it.second.size);
	}

	octree.execute([this](std::vector<Octree<Entity>::Entry>& entries) {
		for (std::vector<Octree<Entity>::Entry>::iterator i = entries.begin(); i != entries.end(); i++) {
			for (std::vector<Octree<Entity>::Entry>::iterator j = std::next(i); j != entries.end(); j++) {
				BroadphaseCollision broadphaseCollision;
				broadphaseCollision.entity1 = std::min(i->object, j->object);
				broadphaseCollision.entity2 = std::max(i->object, j->object);

				const Rigidbody& entity1Rigidbody = ecs->getComponent<Rigidbody>(broadphaseCollision.entity1);
				const Rigidbody& entity2Rigidbody = ecs->getComponent<Rigidbody>(broadphaseCollision.entity2);

				if (entity1Rigidbody.isStatic && entity2Rigidbody.isStatic) {
					return;
				}

				m_broadphaseCollisions.insert(broadphaseCollision);
			}
		}
		});
}

void NtshEngn::PhysicsModule::collisionsNarrowphase() {
	m_narrowphaseCollisions.clear();

	std::mutex mutex;

	jobSystem->dispatch(static_cast<uint32_t>(m_broadphaseCollisions.size()), (static_cast<uint32_t>(m_broadphaseCollisions.size()) + jobSystem->getNumThreads() - 1) / jobSystem->getNumThreads(), [this, &mutex](JobDispatchArguments args) {
		std::set<BroadphaseCollision>::iterator it = m_broadphaseCollisions.begin();
		std::advance(it, args.jobIndex);

		const BroadphaseCollision& broadphaseCollision = *it;

		const Entity entity1 = broadphaseCollision.entity1;
		const Entity entity2 = broadphaseCollision.entity2;

		ColliderShape* collider1Shape = nullptr;

		ColliderSphere collider1Sphere;
		ColliderAABB collider1AABB;
		ColliderCapsule collider1Capsule;
		if (ecs->hasComponent<SphereCollidable>(entity1)) {
			collider1Sphere = ecs->getComponent<SphereCollidable>(entity1).collider;
			collider1Shape = &collider1Sphere;
		}
		else if (ecs->hasComponent<AABBCollidable>(entity1)) {
			collider1AABB = ecs->getComponent<AABBCollidable>(entity1).collider;
			collider1Shape = &collider1AABB;
		}
		else if (ecs->hasComponent<CapsuleCollidable>(entity1)) {
			collider1Capsule = ecs->getComponent<CapsuleCollidable>(entity1).collider;
			collider1Shape = &collider1Capsule;
		}

		const Transform& entity1Transform = ecs->getComponent<Transform>(entity1);
		transform(collider1Shape, entity1Transform.position, entity1Transform.rotation, entity1Transform.scale);

		ColliderShape* collider2Shape = nullptr;

		ColliderSphere collider2Sphere;
		ColliderAABB collider2AABB;
		ColliderCapsule collider2Capsule;
		if (ecs->hasComponent<SphereCollidable>(entity2)) {
			collider2Sphere = ecs->getComponent<SphereCollidable>(entity2).collider;
			collider2Shape = &collider2Sphere;
		}
		else if (ecs->hasComponent<AABBCollidable>(entity2)) {
			collider2AABB = ecs->getComponent<AABBCollidable>(entity2).collider;
			collider2Shape = &collider2AABB;
		}
		else if (ecs->hasComponent<CapsuleCollidable>(entity2)) {
			collider2Capsule = ecs->getComponent<CapsuleCollidable>(entity2).collider;
			collider2Shape = &collider2Capsule;
		}

		const Transform& entity2Transform = ecs->getComponent<Transform>(entity2);
		transform(collider2Shape, entity2Transform.position, entity2Transform.rotation, entity2Transform.scale);

		IntersectionInformation intersectionInformation = intersect(collider1Shape, collider2Shape);
		if (intersectionInformation.hasIntersected) {
			NarrowphaseCollision narrowphaseCollision;
			narrowphaseCollision.entity1 = entity1;
			narrowphaseCollision.entity2 = entity2;
			narrowphaseCollision.intersectionNormal = Math::vec3(intersectionInformation.intersectionNormal.data());
			narrowphaseCollision.intersectionDepth = intersectionInformation.intersectionDepth;

			std::unique_lock<std::mutex> lock(mutex);
			m_narrowphaseCollisions.push_back(narrowphaseCollision);
			lock.unlock();
		}
		});

	jobSystem->wait();
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

	const float x = std::max(aabb->min.x, std::min(sphere->center.x, aabb->max.x));
	const float y = std::max(aabb->min.y, std::min(sphere->center.y, aabb->max.y));
	const float z = std::max(aabb->min.z, std::min(sphere->center.z, aabb->max.z));

	const Math::vec3 sphereOnAABB = Math::vec3(x, y, z);

	if (sphereOnAABB == sphere->center) {
		intersectionInformation.hasIntersected = true;

		const Math::vec3 aabbCenter = getCenter(aabb);
		if (aabbCenter == sphere->center) {
			intersectionInformation.intersectionNormal = Math::vec3(0.0f, 1.0f, 0.0f);
			intersectionInformation.intersectionDepth = ((aabb->max.y - aabb->min.y) / 2.0f) + sphere->radius;

			return intersectionInformation;
		}

		const std::array<Math::vec3, 6> normals = {
			Math::vec3(-1.0f, 0.0f, 0.0f),
			Math::vec3(1.0f, 0.0f, 0.0f),
			Math::vec3(0.0f, -1.0f, 0.0f),
			Math::vec3(0.0f, 1.0f, 0.0f),
			Math::vec3(0.0f, 0.0f, -1.0f),
			Math::vec3(0.0f, 0.0f, 1.0f)
		};

		const std::array<float, 6> distances = {
			aabb->max.x - sphere->center.x,
			sphere->center.x - aabb->min.x,
			aabb->max.y - sphere->center.y,
			sphere->center.y - aabb->min.y,
			aabb->max.z - sphere->center.z,
			sphere->center.z - aabb->min.z
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

		intersectionInformation.intersectionNormal = normals[collidedFace];
		intersectionInformation.intersectionDepth = distances[collidedFace] + sphere->radius;

		return intersectionInformation;
	}

	const float distance = (sphereOnAABB - sphere->center).length();

	if ((distance < 0.000001f) || (distance >= sphere->radius)) {
		intersectionInformation.hasIntersected = false;

		return intersectionInformation;
	}

	intersectionInformation.hasIntersected = true;
	intersectionInformation.intersectionNormal = Math::normalize(sphereOnAABB - sphere->center);
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
		aabb2->max.x - aabb1->min.x,
		aabb1->max.x - aabb2->min.x,
		aabb2->max.y - aabb1->min.y,
		aabb1->max.y - aabb2->min.y,
		aabb2->max.z - aabb1->min.z,
		aabb1->max.z - aabb2->min.z
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
	sphereFromCapsule.center = closestPointOnCapsule;
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