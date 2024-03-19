#include "ntshengn_physics_module.h"
#include "../Module/utils/ntshengn_module_defines.h"
#include "../Module/utils/ntshengn_dynamic_library.h"
#include "../Common/utils/ntshengn_defines.h"
#include "../Common/utils/ntshengn_enums.h"
#include "../Common/utils/ntshengn_utils_octree.h"
#include <algorithm>
#include <limits>
#include <array>
#include <cmath>

void NtshEngn::PhysicsModule::init() {
}

void NtshEngn::PhysicsModule::update(double dt) {
	m_timeAccumulator += dt;

	uint32_t iterations = 0;
	while ((m_timeAccumulator >= m_maxDeltaTime) && (iterations < m_maxIterations)) {
		const float dtSeconds = static_cast<float>(m_maxDeltaTime / 1000.0);

		// Euler integrator
		eulerIntegrator(dtSeconds);

		// Collisions detection
		collisionsDetection();

		// Collisions response
		collisionsResponse();

		m_timeAccumulator -= m_maxDeltaTime;

		iterations++;
	}
}

void NtshEngn::PhysicsModule::destroy() {
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderShape* shape1, const ColliderShape* shape2) {
	if ((shape1->getType() == ColliderShapeType::Box) && (shape2->getType() == ColliderShapeType::Box)) {
		return intersect(static_cast<const ColliderBox*>(shape1), static_cast<const ColliderBox*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Box) && (shape2->getType() == ColliderShapeType::Sphere)) {
		return intersect(static_cast<const ColliderBox*>(shape1), static_cast<const ColliderSphere*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Box) && (shape2->getType() == ColliderShapeType::Capsule)) {
		return intersect(static_cast<const ColliderBox*>(shape1), static_cast<const ColliderCapsule*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Sphere) && (shape2->getType() == ColliderShapeType::Box)) {
		return intersect(static_cast<const ColliderSphere*>(shape1), static_cast<const ColliderBox*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Sphere) && (shape2->getType() == ColliderShapeType::Sphere)) {
		return intersect(static_cast<const ColliderSphere*>(shape1), static_cast<const ColliderSphere*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Sphere) && (shape2->getType() == ColliderShapeType::Capsule)) {
		return intersect(static_cast<const ColliderSphere*>(shape1), static_cast<const ColliderCapsule*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Capsule) && (shape2->getType() == ColliderShapeType::Box)) {
		return intersect(static_cast<const ColliderCapsule*>(shape1), static_cast<const ColliderBox*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Capsule) && (shape2->getType() == ColliderShapeType::Sphere)) {
		return intersect(static_cast<const ColliderCapsule*>(shape1), static_cast<const ColliderSphere*>(shape2));
	}
	else if ((shape1->getType() == ColliderShapeType::Capsule) && (shape2->getType() == ColliderShapeType::Capsule)) {
		return intersect(static_cast<const ColliderCapsule*>(shape1), static_cast<const ColliderCapsule*>(shape2));
	}

	return IntersectionInformation();
}

std::vector<NtshEngn::RaycastInformation> NtshEngn::PhysicsModule::raycast(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax) {
	std::vector<RaycastInformation> raycastInformations;

	const Math::vec3 invRayDirection = Math::vec3(1.0f / rayDirection.x, 1.0f / rayDirection.y, 1.0f / rayDirection.z);

	for (Entity entity : entities) {
		const Transform& entityTransform = ecs->getComponent<Transform>(entity);

		if (ecs->hasComponent<Collidable>(entity)) {
			Collidable collidable = ecs->getComponent<Collidable>(entity);

			if (collidable.collider->getType() == ColliderShapeType::Box) {
				ColliderBox* colliderBox = static_cast<ColliderBox*>(collidable.collider.get());
				transform(colliderBox, entityTransform.position, entityTransform.rotation, entityTransform.scale);

				const Math::mat4 boxRotation = Math::rotate(colliderBox->rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
					Math::rotate(colliderBox->rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
					Math::rotate(colliderBox->rotation.z, Math::vec3(0.0f, 0.0f, 1.0f));

				const Math::vec3 rayToBox = colliderBox->center - rayOrigin;

				Math::vec3 rayDirectionProjected = Math::vec3(Math::dot(boxRotation.x, rayDirection),
					Math::dot(boxRotation.y, rayDirection),
					Math::dot(boxRotation.z, rayDirection));

				const Math::vec3 rayToBoxProjected = Math::vec3(Math::dot(boxRotation.x, rayToBox),
					Math::dot(boxRotation.y, rayToBox),
					Math::dot(boxRotation.z, rayToBox));

				bool noIntersection = false;
				std::array<float, 6> t = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
				for (uint8_t i = 0; i < 3; i++) {
					if (rayDirectionProjected[i] == 0.0f) {
						if (((-rayToBoxProjected[i] - colliderBox->halfExtent[i]) > 0.0f) || ((-rayToBoxProjected[i] + colliderBox->halfExtent[i]) < 0.0f)) {
							noIntersection = true;
							break;
						}
						rayDirectionProjected[i] = 0.000001f;
					}

					t[(i * 2) + 0] = (rayToBoxProjected[i] + colliderBox->halfExtent[i]) / rayDirectionProjected[i];
					t[(i * 2) + 1] = (rayToBoxProjected[i] - colliderBox->halfExtent[i]) / rayDirectionProjected[i];
				}

				if (noIntersection) {
					continue;
				}

				const Math::mat4 boxInverseTransform = Math::transpose(Math::inverse(Math::translate(colliderBox->center) *
					boxRotation *
					Math::scale(colliderBox->halfExtent)));

				const float distanceMin = std::max(std::max(std::min(t[0], t[1]), std::min(t[2], t[3])), std::min(t[4], t[5]));
				const float distanceMax = std::min(std::min(std::max(t[0], t[1]), std::max(t[2], t[3])), std::max(t[4], t[5]));
				if ((distanceMax >= 0.0f) && (distanceMin <= distanceMax) && ((distanceMin >= tMin) && (distanceMin <= tMax))) {
					Math::vec3 normal;
					if (distanceMin == t[0]) {
						normal = Math::normalize(Math::vec3(boxInverseTransform * Math::vec4(Math::vec3(-1.0f, 0.0f, 0.0f), 0.0f)));
					}
					else if (distanceMin == t[1]) {
						normal = Math::normalize(Math::vec3(boxInverseTransform * Math::vec4(Math::vec3(1.0f, 0.0f, 0.0f), 0.0f)));
					}
					else if (distanceMin == t[2]) {
						normal = Math::normalize(Math::vec3(boxInverseTransform * Math::vec4(Math::vec3(0.0f, -1.0f, 0.0f), 0.0f)));
					}
					else if (distanceMin == t[3]) {
						normal = Math::normalize(Math::vec3(boxInverseTransform * Math::vec4(Math::vec3(0.0f, 1.0f, 0.0f), 0.0f)));
					}
					else if (distanceMin == t[4]) {
						normal = Math::normalize(Math::vec3(boxInverseTransform * Math::vec4(Math::vec3(0.0f, 0.0f, -1.0f), 0.0f)));
					}
					else if (distanceMin == t[5]) {
						normal = Math::normalize(Math::vec3(boxInverseTransform * Math::vec4(Math::vec3(0.0f, 0.0f, 1.0f), 0.0f)));
					}

					RaycastInformation raycastInformation;
					raycastInformation.entity = entity;
					raycastInformation.distance = distanceMin;
					raycastInformation.normal = normal;
					raycastInformations.push_back(raycastInformation);
				}
			}
			else if (collidable.collider->getType() == ColliderShapeType::Sphere) {
				ColliderSphere* colliderSphere = static_cast<ColliderSphere*>(collidable.collider.get());
				transform(colliderSphere, entityTransform.position, entityTransform.rotation, entityTransform.scale);

				const Math::vec3 co = rayOrigin - colliderSphere->center;
				const float a = Math::dot(rayDirection, rayDirection);
				const float b = 2.0f * Math::dot(co, rayDirection);
				const float c = Math::dot(co, co) - (colliderSphere->radius * colliderSphere->radius);
				const float discriminant = (b * b) - (4.0f * a * c);

				if (discriminant < 0.0f) {
					continue;
				}

				const float distance = (-b - (std::sqrt(discriminant))) / (2.0f * a);
				if ((distance >= tMin) && (distance <= tMax)) {
					RaycastInformation raycastInformation;
					raycastInformation.entity = entity;
					raycastInformation.distance = distance;
					raycastInformation.normal = Math::normalize((rayOrigin + (rayDirection * distance)) - colliderSphere->center);
					raycastInformations.push_back(raycastInformation);
				}
			}
			else if (collidable.collider->getType() == ColliderShapeType::Capsule) {
				ColliderCapsule* colliderCapsule = static_cast<ColliderCapsule*>(collidable.collider.get());
				transform(colliderCapsule, entityTransform.position, entityTransform.rotation, entityTransform.scale);

				const Math::vec3 ab = colliderCapsule->tip - colliderCapsule->base;
				const Math::vec3 ao = rayOrigin - colliderCapsule->base;

				const float abab = Math::dot(ab, ab);
				const float aoao = Math::dot(ao, ao);
				const float abrd = Math::dot(ab, rayDirection);
				const float abao = Math::dot(ab, ao);
				const float rdao = Math::dot(rayDirection, ao);

				const float a = abab - (abrd * abrd);
				float b = (abab * rdao) - (abao * abrd);
				float c = (abab * aoao) - (abao * abao) - (colliderCapsule->radius * colliderCapsule->radius * abab);
				float h = (b * b) - (a * c);
				if (h >= 0.0f) {
					float distance = (-b - std::sqrt(h)) / a;
					const float y = abao + (distance * abrd);
					if ((y > 0.0) && (y < abab) && ((distance >= tMin) && (distance <= tMax))) {
						const Math::vec3 position = rayOrigin + (rayDirection * distance);
						const Math::vec3 ap = position - colliderCapsule->base;

						RaycastInformation raycastInformation;
						raycastInformation.entity = entity;
						raycastInformation.distance = distance;
						raycastInformation.normal = (ap - (ab * std::clamp(Math::dot(ap, ab) / Math::dot(ab, ab), 0.0f, 1.0f))) / colliderCapsule->radius;
						raycastInformations.push_back(raycastInformation);

						continue;
					}

					const Math::vec3 co = (y <= 0.0f) ? ao : (rayOrigin - colliderCapsule->tip);
					b = Math::dot(rayDirection, co);
					c = Math::dot(co, co) - (colliderCapsule->radius * colliderCapsule->radius);

					h = (b * b) - c;
					distance = -b - std::sqrt(h);
					if ((h > 0.0f) && ((distance >= tMin) && (distance <= tMax))) {
						const Math::vec3 position = rayOrigin + (rayDirection * distance);
						const Math::vec3 ap = position - colliderCapsule->base;

						RaycastInformation raycastInformation;
						raycastInformation.entity = entity;
						raycastInformation.distance = distance;
						raycastInformation.normal = (ap - (ab * std::clamp(Math::dot(ap, ab) / Math::dot(ab, ab), 0.0f, 1.0f))) / colliderCapsule->radius;
						raycastInformations.push_back(raycastInformation);
					}
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
	componentMask.set(ecs->getComponentID<Rigidbody>());
	componentMask.set(ecs->getComponentID<Collidable>());

	return componentMask;
}

void NtshEngn::PhysicsModule::eulerIntegrator(float dtSeconds) {
	for (Entity entity : entities) {
		if (ecs->hasComponent<Rigidbody>(entity)) {
			Rigidbody& entityRigidbody = ecs->getComponent<Rigidbody>(entity);
			if (!entityRigidbody.isStatic) {
				Transform& entityTransform = ecs->getComponent<Transform>(entity);

				entityRigidbody.linearAcceleration = entityRigidbody.force / entityRigidbody.mass;
				entityRigidbody.angularAcceleration = entityRigidbody.torque / entityRigidbody.inertia;

				if (entityRigidbody.isAffectedByConstants) {
					entityRigidbody.linearAcceleration += m_gravity;
				}

				entityRigidbody.linearVelocity += entityRigidbody.linearAcceleration * dtSeconds;
				entityRigidbody.angularVelocity += entityRigidbody.angularAcceleration * dtSeconds;

				entityTransform.position += entityRigidbody.linearVelocity * dtSeconds;

				if (!entityRigidbody.lockRotation) {
					entityTransform.rotation += entityRigidbody.angularVelocity * dtSeconds;
				}
			}
			else {
				entityRigidbody.linearVelocity = { 0.0f, 0.0f, 0.0f };
				entityRigidbody.angularVelocity = { 0.0f, 0.0f, 0.0f };
			}

			entityRigidbody.force = { 0.0f, 0.0f, 0.0f };
			entityRigidbody.torque = { 0.0f, 0.0f, 0.0f };
		}
	}
}

void NtshEngn::PhysicsModule::collisionsDetection() {
	collisionsBroadphase();
	collisionsNarrowphase();
}

void NtshEngn::PhysicsModule::collisionsResponse() {
	std::unordered_map<Entity, ObjectDuringCollisionResponseState> objectStates;

	for (const NarrowphaseCollision& collision : m_narrowphaseCollisions) {
		if (!ecs->hasComponent<Rigidbody>(collision.entity1) || !ecs->hasComponent<Rigidbody>(collision.entity2)) {
			continue;
		}

		const Rigidbody& entity1Rigidbody = ecs->getComponent<Rigidbody>(collision.entity1);
		const Rigidbody& entity2Rigidbody = ecs->getComponent<Rigidbody>(collision.entity2);

		// Position correction
		Math::vec3 correction = std::max(collision.intersectionDepth, 0.0f) * collision.intersectionNormal;
		Math::vec3 entity1CorrectedPositionDelta = 0.0f;
		Math::vec3 entity2CorrectedPositionDelta = 0.0f;

		if (!entity1Rigidbody.isStatic && !entity2Rigidbody.isStatic) {
			// If no entity is static, correction is shared between both of them
			correction /= 2.0f;
		}

		if (!entity1Rigidbody.isStatic) {
			entity1CorrectedPositionDelta -= correction;

			objectStates[collision.entity1].position += entity1CorrectedPositionDelta;
		}

		if (!entity2Rigidbody.isStatic) {
			entity2CorrectedPositionDelta += correction;

			objectStates[collision.entity2].position += entity2CorrectedPositionDelta;
		}

		// Inverse mass and inertia
		float invMass1 = 0.0f;
		float invMass2 = 0.0f;
		float invInertia1 = 0.0f;
		float invInertia2 = 0.0f;

		if (!entity1Rigidbody.isStatic) {
			invMass1 = 1.0f / entity1Rigidbody.mass;
			invInertia1 = 1.0f / entity1Rigidbody.inertia;
		}

		if (!entity2Rigidbody.isStatic) {
			invMass2 = 1.0f / entity2Rigidbody.mass;
			invInertia2 = 1.0f / entity2Rigidbody.inertia;
		}

		const float totalInverseMass = invMass1 + invMass2;

		// Restitution coefficient
		const float e = entity1Rigidbody.restitution * entity2Rigidbody.restitution;

		// For each collision point in the intersection information
		const float intersectionPointsAsFloat = static_cast<float>(collision.relativeIntersectionPoints.size());
		for (size_t i = 0; i < collision.relativeIntersectionPoints.size(); i++) {
			Math::vec3 entity1LinearVelocityDelta = Math::vec3(0.0f, 0.0f, 0.0f);
			Math::vec3 entity1AngularVelocityDelta = Math::vec3(0.0f, 0.0f, 0.0f);
			Math::vec3 entity2LinearVelocityDelta = Math::vec3(0.0f, 0.0f, 0.0f);
			Math::vec3 entity2AngularVelocityDelta = Math::vec3(0.0f, 0.0f, 0.0f);

			// Impulse
			Math::vec3 angularVelocity1 = Math::cross(entity1Rigidbody.angularVelocity, collision.relativeIntersectionPoints[i].first);
			Math::vec3 angularVelocity2 = Math::cross(entity2Rigidbody.angularVelocity, collision.relativeIntersectionPoints[i].second);
			Math::vec3 fullVelocity1 = entity1Rigidbody.linearVelocity + angularVelocity1;
			Math::vec3 fullVelocity2 = entity2Rigidbody.linearVelocity + angularVelocity2;

			Math::vec3 relativeVelocity = fullVelocity2 - fullVelocity1;
			float impulseForce = Math::dot(relativeVelocity, collision.intersectionNormal);

			if (impulseForce >= 0.0f) {
				continue;
			}

			const Math::vec3 angular1 = Math::cross(invInertia1 * Math::cross(collision.relativeIntersectionPoints[i].first, collision.intersectionNormal), collision.relativeIntersectionPoints[i].first);
			const Math::vec3 angular2 = Math::cross(invInertia2 * Math::cross(collision.relativeIntersectionPoints[i].second, collision.intersectionNormal), collision.relativeIntersectionPoints[i].second);
			const float angularEffect = Math::dot(angular1 + angular2, collision.intersectionNormal);

			// Apply impulse
			const float j = (-(1.0f + e) * impulseForce) / (totalInverseMass + angularEffect);
			const Math::vec3 impulse = (j * collision.intersectionNormal) / intersectionPointsAsFloat;

			if (!entity1Rigidbody.isStatic) {
				entity1LinearVelocityDelta -= invMass1 * impulse;
				if (!entity1Rigidbody.lockRotation) {
					entity1AngularVelocityDelta -= invInertia1 * Math::cross(collision.relativeIntersectionPoints[i].first, impulse);
				}

				objectStates[collision.entity1].linearVelocity += entity1LinearVelocityDelta;
				objectStates[collision.entity1].angularVelocity += entity1AngularVelocityDelta;
			}

			if (!entity2Rigidbody.isStatic) {
				entity2LinearVelocityDelta += invMass2 * impulse;
				if (!entity2Rigidbody.lockRotation) {
					entity2AngularVelocityDelta += invInertia2 * Math::cross(collision.relativeIntersectionPoints[i].second, impulse);
				}

				objectStates[collision.entity2].linearVelocity += entity2LinearVelocityDelta;
				objectStates[collision.entity2].angularVelocity += entity2AngularVelocityDelta;
			}

			// Friction
			angularVelocity1 = Math::cross(entity1Rigidbody.angularVelocity + entity1AngularVelocityDelta, collision.relativeIntersectionPoints[i].first);
			angularVelocity2 = Math::cross(entity2Rigidbody.angularVelocity + entity2AngularVelocityDelta, collision.relativeIntersectionPoints[i].second);
			fullVelocity1 = (entity1Rigidbody.linearVelocity + entity1LinearVelocityDelta) + angularVelocity1;
			fullVelocity2 = (entity2Rigidbody.linearVelocity + entity2LinearVelocityDelta) + angularVelocity2;

			relativeVelocity = fullVelocity2 - fullVelocity1;
			impulseForce = Math::dot(relativeVelocity, collision.intersectionNormal);

			Math::vec3 tangent = relativeVelocity - (impulseForce * collision.intersectionNormal);
			if (tangent.length() > 0.0001f) {
				tangent = Math::normalize(tangent);
			}

			const float fVelocity = Math::dot(relativeVelocity, tangent);

			float mu = Math::vec2(entity1Rigidbody.staticFriction, entity2Rigidbody.staticFriction).length();
			const float f = -fVelocity / totalInverseMass;

			Math::vec3 friction;
			if (std::abs(f) < (j * mu)) {
				friction = f * tangent;
			}
			else {
				mu = Math::vec2(entity1Rigidbody.dynamicFriction, entity2Rigidbody.dynamicFriction).length();
				friction = -j * tangent * mu;
			}
			friction /= intersectionPointsAsFloat;

			if (!entity1Rigidbody.isStatic) {
				objectStates[collision.entity1].linearVelocity -= invMass1 * friction;
				objectStates[collision.entity1].angularVelocity -= invInertia1 * Math::cross(collision.relativeIntersectionPoints[i].first, friction);
			}

			if (!entity2Rigidbody.isStatic) {
				objectStates[collision.entity2].linearVelocity += invMass2 * friction;
				objectStates[collision.entity2].angularVelocity += invInertia2 * Math::cross(collision.relativeIntersectionPoints[i].second, friction);
			}
		}
	}

	// Apply changes in velocities and position once all collisions are resolved
	for (const std::pair<Entity, ObjectDuringCollisionResponseState>& objectState : objectStates) {
		Transform& entityTransform = ecs->getComponent<Transform>(objectState.first);
		Rigidbody& entityRigidbody = ecs->getComponent<Rigidbody>(objectState.first);

		entityTransform.position += objectState.second.position;
		entityRigidbody.linearVelocity += objectState.second.linearVelocity;

		if (!entityRigidbody.lockRotation) {
			entityRigidbody.angularVelocity += objectState.second.angularVelocity;
		}
	}

	// Call scripts onCollisionEnter, onCollisionStill and onCollisionExit
	std::sort(m_narrowphaseCollisions.begin(), m_narrowphaseCollisions.end());
	std::vector<NarrowphaseCollision> collisionsEnter;
	std::vector<NarrowphaseCollision> collisionsStill;
	std::vector<NarrowphaseCollision> collisionsExit;
	std::set_difference(m_narrowphaseCollisions.begin(), m_narrowphaseCollisions.end(), m_previousNarrowphaseCollisions.begin(), m_previousNarrowphaseCollisions.end(), std::back_inserter(collisionsEnter));
	std::set_intersection(m_narrowphaseCollisions.begin(), m_narrowphaseCollisions.end(), m_previousNarrowphaseCollisions.begin(), m_previousNarrowphaseCollisions.end(), std::back_inserter(collisionsStill));
	std::set_difference(m_previousNarrowphaseCollisions.begin(), m_previousNarrowphaseCollisions.end(), m_narrowphaseCollisions.begin(), m_narrowphaseCollisions.end(), std::back_inserter(collisionsExit));

	// onCollisionEnter
	for (const NarrowphaseCollision& collisionEnter : collisionsEnter) {
		if (ecs->entityExists(collisionEnter.entity1) && ecs->hasComponent<Scriptable>(collisionEnter.entity1)) {
			CollisionInfo collisionInfo;
			collisionInfo.otherEntity = collisionEnter.entity2;
			collisionInfo.normal = collisionEnter.intersectionNormal;
			collisionInfo.depth = collisionEnter.intersectionDepth;
			
			for (const std::pair<Math::vec3, Math::vec3>& relativePoint : collisionEnter.relativeIntersectionPoints) {
				collisionInfo.relativePoints.push_back(relativePoint.first);
			}

			Scriptable& entityScriptable = ecs->getComponent<Scriptable>(collisionEnter.entity1);
			entityScriptable.script->onCollisionEnter(collisionInfo);
		}

		if (ecs->entityExists(collisionEnter.entity2) && ecs->hasComponent<Scriptable>(collisionEnter.entity2)) {
			CollisionInfo collisionInfo;
			collisionInfo.otherEntity = collisionEnter.entity1;
			collisionInfo.normal = -collisionEnter.intersectionNormal;
			collisionInfo.depth = collisionEnter.intersectionDepth;

			for (const std::pair<Math::vec3, Math::vec3>& relativePoint : collisionEnter.relativeIntersectionPoints) {
				collisionInfo.relativePoints.push_back(relativePoint.second);
			}

			Scriptable& entityScriptable = ecs->getComponent<Scriptable>(collisionEnter.entity2);
			entityScriptable.script->onCollisionEnter(collisionInfo);
		}
	}

	// onCollisionStill
	for (const NarrowphaseCollision& collisionStill : collisionsStill) {
		if (ecs->entityExists(collisionStill.entity1) && ecs->hasComponent<Scriptable>(collisionStill.entity1)) {
			CollisionInfo collisionInfo;
			collisionInfo.otherEntity = collisionStill.entity2;
			collisionInfo.normal = collisionStill.intersectionNormal;
			collisionInfo.depth = collisionStill.intersectionDepth;

			for (const std::pair<Math::vec3, Math::vec3>& relativePoint : collisionStill.relativeIntersectionPoints) {
				collisionInfo.relativePoints.push_back(relativePoint.first);
			}

			Scriptable& entityScriptable = ecs->getComponent<Scriptable>(collisionStill.entity1);
			entityScriptable.script->onCollisionStill(collisionInfo);
		}

		if (ecs->entityExists(collisionStill.entity2) && ecs->hasComponent<Scriptable>(collisionStill.entity2)) {
			CollisionInfo collisionInfo;
			collisionInfo.otherEntity = collisionStill.entity1;
			collisionInfo.normal = -collisionStill.intersectionNormal;
			collisionInfo.depth = collisionStill.intersectionDepth;

			for (const std::pair<Math::vec3, Math::vec3>& relativePoint : collisionStill.relativeIntersectionPoints) {
				collisionInfo.relativePoints.push_back(relativePoint.second);
			}

			Scriptable& entityScriptable = ecs->getComponent<Scriptable>(collisionStill.entity2);
			entityScriptable.script->onCollisionStill(collisionInfo);
		}
	}

	// onCollisionExit
	for (const NarrowphaseCollision& collisionExit : collisionsExit) {
		if (ecs->entityExists(collisionExit.entity1) && ecs->hasComponent<Scriptable>(collisionExit.entity1)) {
			CollisionInfo collisionInfo;
			collisionInfo.otherEntity = collisionExit.entity2;
			collisionInfo.normal = collisionExit.intersectionNormal;
			collisionInfo.depth = collisionExit.intersectionDepth;

			for (const std::pair<Math::vec3, Math::vec3>& relativePoint : collisionExit.relativeIntersectionPoints) {
				collisionInfo.relativePoints.push_back(relativePoint.first);
			}

			Scriptable& entityScriptable = ecs->getComponent<Scriptable>(collisionExit.entity1);
			entityScriptable.script->onCollisionExit(collisionInfo);
		}

		if (ecs->entityExists(collisionExit.entity2) && ecs->hasComponent<Scriptable>(collisionExit.entity2)) {
			CollisionInfo collisionInfo;
			collisionInfo.otherEntity = collisionExit.entity1;
			collisionInfo.normal = -collisionExit.intersectionNormal;
			collisionInfo.depth = collisionExit.intersectionDepth;

			for (const std::pair<Math::vec3, Math::vec3>& relativePoint : collisionExit.relativeIntersectionPoints) {
				collisionInfo.relativePoints.push_back(relativePoint.second);
			}

			Scriptable& entityScriptable = ecs->getComponent<Scriptable>(collisionExit.entity2);
			entityScriptable.script->onCollisionExit(collisionInfo);
		}
	}
}

void NtshEngn::PhysicsModule::collisionsBroadphase() {
	m_broadphaseCollisions.clear();

	Math::vec3 sceneAABBMin;
	Math::vec3 sceneAABBMax;

	std::unordered_map<Entity, AABB> entityAABBs;
	std::unordered_map<Entity, bool> entityStatic;
	for (const Entity& entity : entities) {
		AABB entityAABB;

		const Transform& entityTransform = ecs->getComponent<Transform>(entity);

		if (ecs->hasComponent<Collidable>(entity)) {
			Collidable collidable = ecs->getComponent<Collidable>(entity);

			if (collidable.collider->getType() == ColliderShapeType::Box) {
				ColliderBox* colliderBox = static_cast<ColliderBox*>(collidable.collider.get());
				transform(colliderBox, entityTransform.position, entityTransform.rotation, entityTransform.scale);

				const Math::vec3 min = Math::vec3(-1.0f, -1.0f, -1.0f);
				const Math::vec3 max = Math::vec3(1.0f, 1.0f, 1.0f);

				const Math::mat4 transformMatrix = Math::translate(colliderBox->center) *
					Math::rotate(colliderBox->rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
					Math::rotate(colliderBox->rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
					Math::rotate(colliderBox->rotation.z, Math::vec3(0.0f, 0.0f, 1.0f)) *
					Math::scale(colliderBox->halfExtent);

				std::array<Math::vec3, 8> boxPoints = {
					Math::vec3(min.x, min.y, min.z),
					Math::vec3(max.x, min.y, min.z),
					Math::vec3(max.x, min.y, max.z),
					Math::vec3(min.x, min.y, max.z),
					Math::vec3(min.x, max.y, min.z),
					Math::vec3(max.x, max.y, min.z),
					Math::vec3(max.x, max.y, max.z),
					Math::vec3(min.x, max.y, max.z)
				};

				Math::vec3 minAABB = Math::vec3(std::numeric_limits<float>::max());
				Math::vec3 maxAABB = Math::vec3(std::numeric_limits<float>::lowest());
				for (uint8_t i = 0; i < 8; i++) {
					boxPoints[i] = Math::vec3(transformMatrix * Math::vec4(boxPoints[i], 1.0f));

					if (boxPoints[i].x < minAABB.x) {
						minAABB.x = boxPoints[i].x;
					}
					if (boxPoints[i].x > maxAABB.x) {
						maxAABB.x = boxPoints[i].x;
					}

					if (boxPoints[i].y < minAABB.y) {
						minAABB.y = boxPoints[i].y;
					}
					if (boxPoints[i].y > maxAABB.y) {
						maxAABB.y = boxPoints[i].y;
					}

					if (boxPoints[i].z < minAABB.z) {
						minAABB.z = boxPoints[i].z;
					}
					if (boxPoints[i].z > maxAABB.z) {
						maxAABB.z = boxPoints[i].z;
					}
				}

				entityAABB.position = colliderBox->center;
				entityAABB.size = (maxAABB - minAABB) / 2.0f;
			}
			else if (collidable.collider->getType() == ColliderShapeType::Sphere) {
				ColliderSphere* colliderSphere = static_cast<ColliderSphere*>(collidable.collider.get());
				transform(colliderSphere, entityTransform.position, entityTransform.rotation, entityTransform.scale);

				entityAABB.position = colliderSphere->center;
				entityAABB.size = Math::vec3(colliderSphere->radius);
			}
			else if (collidable.collider->getType() == ColliderShapeType::Capsule) {
				ColliderCapsule* colliderCapsule = static_cast<ColliderCapsule*>(collidable.collider.get());
				transform(colliderCapsule, entityTransform.position, entityTransform.rotation, entityTransform.scale);

				entityAABB.position = getCenter(colliderCapsule);

				const Math::vec3 baseAABBMin = colliderCapsule->base - Math::vec3(colliderCapsule->radius);
				const Math::vec3 baseAABBMax = colliderCapsule->base + Math::vec3(colliderCapsule->radius);

				const Math::vec3 tipAABBMin = colliderCapsule->tip - Math::vec3(colliderCapsule->radius);
				const Math::vec3 tipAABBMax = colliderCapsule->tip + Math::vec3(colliderCapsule->radius);

				const Math::vec3 capsuleAABBMin = Math::vec3(std::min(baseAABBMin.x, tipAABBMin.x), std::min(baseAABBMin.y, tipAABBMin.y), std::min(baseAABBMin.z, tipAABBMin.z));
				const Math::vec3 capsuleAABBMax = Math::vec3(std::max(baseAABBMax.x, tipAABBMax.x), std::max(baseAABBMax.y, tipAABBMax.y), std::max(baseAABBMax.z, tipAABBMax.z));

				entityAABB.size = (capsuleAABBMax - capsuleAABBMin) / 2.0f;
			}
		}
		else {
			continue;
		}

		bool entityIsStatic = false;
		if (ecs->hasComponent<Rigidbody>(entity)) {
			const Rigidbody& entityRigidbody = ecs->getComponent<Rigidbody>(entity);
			entityIsStatic = entityRigidbody.isStatic;
		}

		entityAABBs[entity] = entityAABB;
		entityStatic[entity] = entityIsStatic;

		if ((entityAABB.position.x - entityAABB.size.x) < sceneAABBMin.x) {
			sceneAABBMin.x = entityAABB.position.x - entityAABB.size.x;
		}
		if ((entityAABB.position.y - entityAABB.size.y) < sceneAABBMin.y) {
			sceneAABBMin.y = entityAABB.position.y - entityAABB.size.y;
		}
		if ((entityAABB.position.z - entityAABB.size.z) < sceneAABBMin.z) {
			sceneAABBMin.z = entityAABB.position.z - entityAABB.size.z;
		}

		if ((entityAABB.position.x + entityAABB.size.x) > sceneAABBMax.x) {
			sceneAABBMax.x = entityAABB.position.x + entityAABB.size.x;
		}
		if ((entityAABB.position.y + entityAABB.size.y) > sceneAABBMax.y) {
			sceneAABBMax.y = entityAABB.position.y + entityAABB.size.y;
		}
		if ((entityAABB.position.z + entityAABB.size.z) > sceneAABBMax.z) {
			sceneAABBMax.z = entityAABB.position.z + entityAABB.size.z;
		}
	}

	Octree<Entity> octree = Octree<Entity>((sceneAABBMin + sceneAABBMax) / 2.0f, ((sceneAABBMax - sceneAABBMin) / 2.0f) * 100.0f, 6);
	for (const auto& it : entityAABBs) {
		octree.insert(it.first, it.second.position, it.second.size);
	}

	octree.execute([this, &entityStatic](std::vector<Octree<Entity>::Entry>& entries) {
		for (std::vector<Octree<Entity>::Entry>::iterator i = entries.begin(); i != entries.end(); i++) {
			for (std::vector<Octree<Entity>::Entry>::iterator j = std::next(i); j != entries.end(); j++) {
				BroadphaseCollision broadphaseCollision;
				broadphaseCollision.entity1 = std::min(i->object, j->object);
				broadphaseCollision.entity2 = std::max(i->object, j->object);

				if (!entityStatic[broadphaseCollision.entity1] || !entityStatic[broadphaseCollision.entity2]) {
					m_broadphaseCollisions.insert(broadphaseCollision);
				}
			}
		}
		});
}

void NtshEngn::PhysicsModule::collisionsNarrowphase() {
	m_previousNarrowphaseCollisions = m_narrowphaseCollisions;
	m_narrowphaseCollisions.clear();

	std::mutex mutex;

	jobSystem->dispatch(static_cast<uint32_t>(m_broadphaseCollisions.size()), (static_cast<uint32_t>(m_broadphaseCollisions.size()) + jobSystem->getNumThreads() - 1) / jobSystem->getNumThreads(), [this, &mutex](JobDispatchArguments args) {
		std::set<BroadphaseCollision>::iterator it = m_broadphaseCollisions.begin();
		std::advance(it, args.jobIndex);

		const BroadphaseCollision& broadphaseCollision = *it;

		const Entity entity1 = broadphaseCollision.entity1;
		const Entity entity2 = broadphaseCollision.entity2;

		Collidable collidable1 = ecs->getComponent<Collidable>(entity1);

		const Transform& entity1Transform = ecs->getComponent<Transform>(entity1);
		transform(collidable1.collider.get(), entity1Transform.position, entity1Transform.rotation, entity1Transform.scale);

		Collidable collidable2 = ecs->getComponent<Collidable>(entity2);

		const Transform& entity2Transform = ecs->getComponent<Transform>(entity2);
		transform(collidable2.collider.get(), entity2Transform.position, entity2Transform.rotation, entity2Transform.scale);

		IntersectionInformation intersectionInformation = intersect(collidable1.collider.get(), collidable2.collider.get());
		if (intersectionInformation.hasIntersected) {
			NarrowphaseCollision narrowphaseCollision;
			narrowphaseCollision.entity1 = entity1;
			narrowphaseCollision.entity2 = entity2;
			narrowphaseCollision.intersectionNormal = intersectionInformation.normal;
			narrowphaseCollision.intersectionDepth = intersectionInformation.depth;
			narrowphaseCollision.relativeIntersectionPoints = intersectionInformation.relativePoints;

			std::unique_lock<std::mutex> lock(mutex);
			m_narrowphaseCollisions.push_back(narrowphaseCollision);
			lock.unlock();
		}
		});

	jobSystem->wait();
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderBox* box1, const ColliderBox* box2) {
	IntersectionInformation intersectionInformation;

	const Math::mat4 box1Rotation = Math::rotate(box1->rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(box1->rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(box1->rotation.z, Math::vec3(0.0f, 0.0f, 1.0f));

	const Math::mat4 box1Transform = Math::translate(box1->center) *
		box1Rotation *
		Math::scale(box1->halfExtent);

	const Math::mat4 box2Rotation = Math::rotate(box2->rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(box2->rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(box2->rotation.z, Math::vec3(0.0f, 0.0f, 1.0f));

	const Math::mat4 box2Transform = Math::translate(box2->center) *
		box2Rotation *
		Math::scale(box2->halfExtent);

	std::array<Math::vec3, 15> axisToTest = {
		box1Rotation.x,
		box1Rotation.y,
		box1Rotation.z,
		box2Rotation.x,
		box2Rotation.y,
		box2Rotation.z
	};

	for (uint8_t i = 0; i < 3; i++) {
		axisToTest[6 + (i * 3) + 0] = Math::cross(axisToTest[i], axisToTest[0]);
		axisToTest[6 + (i * 3) + 1] = Math::cross(axisToTest[i], axisToTest[1]);
		axisToTest[6 + (i * 3) + 2] = Math::cross(axisToTest[i], axisToTest[2]);
	}

	const std::array<Math::vec3, 8> box1Corners = {
		Math::vec3(box1Transform * Math::vec4(-1.0f, -1.0f, -1.0f, 1.0f)),
		Math::vec3(box1Transform * Math::vec4(1.0f, -1.0f, -1.0f, 1.0f)),
		Math::vec3(box1Transform * Math::vec4(1.0f, -1.0f, 1.0f, 1.0f)),
		Math::vec3(box1Transform * Math::vec4(-1.0f, -1.0f, 1.0f, 1.0f)),
		Math::vec3(box1Transform * Math::vec4(-1.0f, 1.0f, -1.0f, 1.0f)),
		Math::vec3(box1Transform * Math::vec4(1.0f, 1.0f, -1.0f, 1.0f)),
		Math::vec3(box1Transform * Math::vec4(1.0f, 1.0f, 1.0f, 1.0f)),
		Math::vec3(box1Transform * Math::vec4(-1.0f, 1.0f, 1.0f, 1.0f))
	};

	const std::array<Math::vec3, 8> box2Corners = {
		Math::vec3(box2Transform * Math::vec4(-1.0f, -1.0f, -1.0f, 1.0f)),
		Math::vec3(box2Transform * Math::vec4(1.0f, -1.0f, -1.0f, 1.0f)),
		Math::vec3(box2Transform * Math::vec4(1.0f, -1.0f, 1.0f, 1.0f)),
		Math::vec3(box2Transform * Math::vec4(-1.0f, -1.0f, 1.0f, 1.0f)),
		Math::vec3(box2Transform * Math::vec4(-1.0f, 1.0f, -1.0f, 1.0f)),
		Math::vec3(box2Transform * Math::vec4(1.0f, 1.0f, -1.0f, 1.0f)),
		Math::vec3(box2Transform * Math::vec4(1.0f, 1.0f, 1.0f, 1.0f)),
		Math::vec3(box2Transform * Math::vec4(-1.0f, 1.0f, 1.0f, 1.0f))
	};

	const std::array<std::pair<Math::vec3, Math::vec3>, 12> box1Edges = {
		std::pair<Math::vec3, Math::vec3>(box1Corners[0], box1Corners[1]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[1], box1Corners[2]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[2], box1Corners[3]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[3], box1Corners[0]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[4], box1Corners[5]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[5], box1Corners[6]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[6], box1Corners[7]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[7], box1Corners[4]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[0], box1Corners[4]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[1], box1Corners[5]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[2], box1Corners[6]),
		std::pair<Math::vec3, Math::vec3>(box1Corners[3], box1Corners[7])
	};

	const std::array<std::pair<Math::vec3, Math::vec3>, 12> box2Edges = {
		std::pair<Math::vec3, Math::vec3>(box2Corners[0], box2Corners[1]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[1], box2Corners[2]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[2], box2Corners[3]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[3], box2Corners[0]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[4], box2Corners[5]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[5], box2Corners[6]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[6], box2Corners[7]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[7], box2Corners[4]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[0], box2Corners[4]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[1], box2Corners[5]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[2], box2Corners[6]),
		std::pair<Math::vec3, Math::vec3>(box2Corners[3], box2Corners[7])
	};

	intersectionInformation.depth = std::numeric_limits<float>::max();
	for (uint8_t i = 0; i < 15; i++) {
		if (Math::dot(axisToTest[i], axisToTest[i]) < 0.0001f) {
			continue;
		}
		axisToTest[i] = Math::normalize(axisToTest[i]);

		float box1IntervalMin = Math::dot(axisToTest[i], box1Corners[0]);
		float box1IntervalMax = box1IntervalMin;

		float box2IntervalMin = Math::dot(axisToTest[i], box2Corners[0]);
		float box2IntervalMax = box2IntervalMin;

		for (uint8_t j = 1; j < 8; j++) {
			const float box1Projection = Math::dot(axisToTest[i], box1Corners[j]);
			if (box1Projection < box1IntervalMin) {
				box1IntervalMin = box1Projection;
			}
			if (box1Projection > box1IntervalMax) {
				box1IntervalMax = box1Projection;
			}

			const float box2Projection = Math::dot(axisToTest[i], box2Corners[j]);
			if (box2Projection < box2IntervalMin) {
				box2IntervalMin = box2Projection;
			}
			if (box2Projection > box2IntervalMax) {
				box2IntervalMax = box2Projection;
			}
		}

		if ((box2IntervalMin > box1IntervalMax) || (box1IntervalMin > box2IntervalMax)) {
			intersectionInformation.hasIntersected = false;

			return intersectionInformation;
		}

		const float box1Interval = box1IntervalMax - box1IntervalMin;
		const float box2Interval = box2IntervalMax - box2IntervalMin;
		const float intervalMin = std::min(box1IntervalMin, box2IntervalMin);
		const float intervalMax = std::max(box1IntervalMax, box2IntervalMax);
		const float interval = intervalMax - intervalMin;
		float depth = (box1Interval + box2Interval) - interval;

		if (((box1IntervalMin < box2IntervalMin) && (box1IntervalMax > box2IntervalMax)) ||
			((box2IntervalMin < box1IntervalMin) && (box2IntervalMax > box1IntervalMax))) {
			depth += std::min(std::abs(box1IntervalMin - box2IntervalMin), std::abs(box1IntervalMax - box2IntervalMax));
		}

		if (depth < intersectionInformation.depth) {
			const float flipNormal = (Math::dot(box2->center - box1->center, axisToTest[i]) < 0.0f) ? -1.0f : 1.0f;
			intersectionInformation.hasIntersected = true;
			intersectionInformation.normal = axisToTest[i] * flipNormal;
			intersectionInformation.depth = depth;
		}
	}

	if (intersectionInformation.hasIntersected) {
		std::vector<Math::vec3> intersectionPoints = clipEdgesToBox(box1Edges, box2, box2Rotation);
		std::vector<Math::vec3> intersectionPoints2To1 = clipEdgesToBox(box2Edges, box1, box1Rotation);
		intersectionPoints.insert(intersectionPoints.end(), intersectionPoints2To1.begin(), intersectionPoints2To1.end());
		
		std::vector<Math::vec3> alreadySeenPoints;
		for (size_t i = 0; i < intersectionPoints.size(); i++) {
			std::pair<Math::vec3, Math::vec3> relativePoint = { intersectionPoints[i] - box1->center, intersectionPoints[i] - box2->center };

			bool foundPoint = false;
			for (size_t j = 0; j < alreadySeenPoints.size(); j++) {
				if (intersectionPoints[i] == alreadySeenPoints[j]) {
					foundPoint = true;
				}
			}
			if (!foundPoint) {
				alreadySeenPoints.push_back(intersectionPoints[i]);
				intersectionInformation.relativePoints.push_back(relativePoint);
			}
		}
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderBox* box, const ColliderSphere* sphere) {
	IntersectionInformation intersectionInformation;

	Math::vec3 closestPoint = box->center;
	const Math::vec3 direction = sphere->center - box->center;

	const Math::mat4 boxRotation = Math::rotate(box->rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(box->rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(box->rotation.z, Math::vec3(0.0f, 0.0f, 1.0f));
	for (uint8_t i = 0; i < 3; i++) {
		float closestHalfExtent = Math::dot(direction, boxRotation[i]);
		if (closestHalfExtent > box->halfExtent[i]) {
			closestHalfExtent = box->halfExtent[i];
		}
		if (closestHalfExtent < -box->halfExtent[i]) {
			closestHalfExtent = -box->halfExtent[i];
		}

		closestPoint += closestHalfExtent * boxRotation[i];
	}

	const float distance = (sphere->center - closestPoint).length();

	if ((distance < 0.000001f) || (distance >= sphere->radius)) {
		intersectionInformation.hasIntersected = false;

		return intersectionInformation;
	}

	const Math::vec3 intersectionNormal = Math::normalize(sphere->center - closestPoint);
	const Math::vec3 outsidePoint = -intersectionNormal * sphere->radius;

	intersectionInformation.hasIntersected = true;
	intersectionInformation.normal = intersectionNormal;
	intersectionInformation.depth = sphere->radius - distance;
	intersectionInformation.relativePoints = { { closestPoint - box->center, outsidePoint } };

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderBox* box, const ColliderCapsule* capsule) {
	IntersectionInformation intersectionInformation;

	const Math::mat4 boxRotation = Math::rotate(box->rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(box->rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(box->rotation.z, Math::vec3(0.0f, 0.0f, 1.0f));
	
	float distanceToSegmentOrigin;
	Math::vec3 pointOnBox;
	const float squaredDistance = squaredDistanceSegmentBox(capsule->base, capsule->tip, box, boxRotation, distanceToSegmentOrigin, pointOnBox);
	if (squaredDistance >= (capsule->radius * capsule->radius)) {
		intersectionInformation.hasIntersected = false;

		return intersectionInformation;
	}

	if (squaredDistance != 0.0f) {
		const Math::vec3 pointOnSegment = capsule->base + ((capsule->tip - capsule->base) * distanceToSegmentOrigin);
		pointOnBox = box->center + Math::vec3(boxRotation * Math::vec4(pointOnBox, 1.0f));

		Math::vec3 normal = pointOnSegment - pointOnBox;
		const float normalLength = normal.length();

		if (normalLength > 0.0f) {
			normal *= 1.0f / normalLength;

			boxCapsuleIntersectionInformationRay(box, boxRotation, capsule, normal, intersectionInformation);

			if (intersectionInformation.relativePoints.size() == 2) {
				return intersectionInformation;
			}

			boxCapsuleIntersectionInformationEdge(box, boxRotation, capsule, normal, intersectionInformation);

			if (intersectionInformation.relativePoints.empty()) {
				intersectionInformation.hasIntersected = true;
				intersectionInformation.normal = normal;
				intersectionInformation.depth = capsule->radius - std::sqrt(squaredDistance);
				intersectionInformation.relativePoints.push_back({ pointOnBox - box->center, pointOnBox - getCenter(capsule) });
			}
		}
	}

	return intersectionInformation;
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
	intersectionInformation.normal = Math::normalize(centerDiff);
	intersectionInformation.depth = (sphere1->radius + sphere2->radius) - centerDiffLength;
	intersectionInformation.relativePoints = { { intersectionInformation.normal * sphere1->radius, -intersectionInformation.normal * sphere2->radius } };

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderSphere* sphere, const ColliderCapsule* capsule) {
	ColliderSphere sphereFromCapsule;
	sphereFromCapsule.center = closestPointOnSegment(sphere->center, capsule->base, capsule->tip);
	sphereFromCapsule.radius = capsule->radius;

	return intersect(sphere, &sphereFromCapsule);
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule1, const ColliderCapsule* capsule2) {
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
		intersectionInformation.normal = -intersectionInformation.normal;
		for (size_t i = 0; i < intersectionInformation.relativePoints.size(); i++) {
			std::swap(intersectionInformation.relativePoints[i].first, intersectionInformation.relativePoints[i].second);
		}
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderSphere* sphere, const ColliderBox* box) {
	IntersectionInformation intersectionInformation = intersect(box, sphere);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.normal = -intersectionInformation.normal;
		for (size_t i = 0; i < intersectionInformation.relativePoints.size(); i++) {
			std::swap(intersectionInformation.relativePoints[i].first, intersectionInformation.relativePoints[i].second);
		}
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule, const ColliderBox* box) {
	IntersectionInformation intersectionInformation = intersect(box, capsule);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.normal = -intersectionInformation.normal;
		for (size_t i = 0; i < intersectionInformation.relativePoints.size(); i++) {
			std::swap(intersectionInformation.relativePoints[i].first, intersectionInformation.relativePoints[i].second);
		}
	}

	return intersectionInformation;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderShape* shape) {
	if (shape->getType() == ColliderShapeType::Box) {
		return getCenter(static_cast<const ColliderBox*>(shape));
	}
	else if (shape->getType() == ColliderShapeType::Sphere) {
		return getCenter(static_cast<const ColliderSphere*>(shape));
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		return getCenter(static_cast<const ColliderCapsule*>(shape));
	}

	return Math::vec3(0.0f, 0.0f, 0.0f);
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderBox* box) {
	return box->center;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderSphere* sphere) {
	return sphere->center;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderCapsule* capsule) {
	return (capsule->base + capsule->tip) / 2.0f;
}

void NtshEngn::PhysicsModule::transform(ColliderShape* shape, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
	if (shape->getType() == ColliderShapeType::Box) {
		transform(static_cast<ColliderBox*>(shape), translation, rotation, scale);
	}
	else if (shape->getType() == ColliderShapeType::Sphere) {
		transform(static_cast<ColliderSphere*>(shape), translation, scale);
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		transform(static_cast<ColliderCapsule*>(shape), translation, rotation, scale);
	}
}

void NtshEngn::PhysicsModule::transform(ColliderBox* box, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
	box->center += translation;
	box->halfExtent.x *= std::abs(scale.x);
	box->halfExtent.y *= std::abs(scale.y);
	box->halfExtent.z *= std::abs(scale.z);

	const Math::quat originalRotation = Math::to_quat(box->rotation);
	const Math::quat modelRotation = Math::to_quat(rotation);
	box->rotation = Math::to_vec3(originalRotation * modelRotation);
}

void NtshEngn::PhysicsModule::transform(ColliderSphere* sphere, const Math::vec3& translation, const Math::vec3& scale) {
	sphere->center += translation;
	sphere->radius *= std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z)));
}

void NtshEngn::PhysicsModule::transform(ColliderCapsule* capsule, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
	const Math::vec3 capsuleCenter = getCenter(capsule);

	capsule->base -= capsuleCenter;
	capsule->tip -= capsuleCenter;
	const Math::mat4 rotationMatrix = Math::translate(translation) * Math::rotate(rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(rotation.z, Math::vec3(0.0f, 0.0f, 1.0f));

	capsule->base = Math::vec3(rotationMatrix * Math::vec4(capsule->base, 1.0f)) + capsuleCenter;
	capsule->tip = Math::vec3(rotationMatrix * Math::vec4(capsule->tip, 1.0f)) + capsuleCenter;

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

// https://github.com/NVIDIA-Omniverse/PhysX/blob/439b4767167e1753d335b613b059140d4d1b4757/physx/source/geomutils/src/distance/GuDistanceSegmentBox.cpp
float NtshEngn::PhysicsModule::squaredDistanceLineBoxFace(uint8_t index0, uint8_t index1, uint8_t index2, Math::vec3& point, const Math::vec3& direction, const Math::vec3& boxHalfExtent, const Math::vec3& halfExtentToPoint, float& distanceToLineOrigin) {
	float squaredDistance = 0.0f;

	Math::vec3 pointPlusHalfExtent;
	pointPlusHalfExtent[index1] = point[index1] + boxHalfExtent[index1];
	pointPlusHalfExtent[index2] = point[index2] + boxHalfExtent[index2];

	if ((direction[index0] * pointPlusHalfExtent[index1]) >= (direction[index1] * halfExtentToPoint[index0])) {
		if ((direction[index0] * pointPlusHalfExtent[index2]) >= (direction[index2] * halfExtentToPoint[index0])) {
			point[index0] = halfExtentToPoint[index0];
			const float inv = 1.0f / direction[index0];
			point[index1] -= direction[index1] * halfExtentToPoint[index0] * inv;
			point[index2] -= direction[index2] * halfExtentToPoint[index0] * inv;
			distanceToLineOrigin = -halfExtentToPoint[index0] * inv;
		}
		else {
			float lSquared = (direction[index0] * direction[index0]) + (direction[index2] * direction[index2]);
			float tmp = (lSquared * pointPlusHalfExtent[index1]) - (direction[index1] * ((direction[index0] * halfExtentToPoint[index0]) + (direction[index2] * pointPlusHalfExtent[index2])));
			if (tmp <= (2.0f * lSquared * boxHalfExtent[index1])) {
				const float t = tmp / lSquared;
				lSquared += direction[index1] * direction[index1];
				tmp = pointPlusHalfExtent[index1] - t;
				const float delta = (direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * tmp) + (direction[index2] * pointPlusHalfExtent[index2]);
				distanceToLineOrigin = -delta / lSquared;
				squaredDistance += (halfExtentToPoint[index0] * halfExtentToPoint[index0]) + (tmp * tmp) + (pointPlusHalfExtent[index2] * pointPlusHalfExtent[index2]) + (delta * distanceToLineOrigin);
				point[index0] = boxHalfExtent[index0];
				point[index1] = t - boxHalfExtent[index1];
				point[index2] = -boxHalfExtent[index2];
			}
			else {
				lSquared += direction[index1] * direction[index1];
				const float delta = (direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * halfExtentToPoint[index1]) + (direction[index2] * pointPlusHalfExtent[index2]);
				distanceToLineOrigin = -delta / lSquared;
				squaredDistance += (halfExtentToPoint[index0] * halfExtentToPoint[index0]) + (halfExtentToPoint[index1] * halfExtentToPoint[index1]) + (pointPlusHalfExtent[index2] * pointPlusHalfExtent[index2]) + (delta * distanceToLineOrigin);
				point[index0] = boxHalfExtent[index0];
				point[index1] = boxHalfExtent[index1];
				point[index2] = -boxHalfExtent[index2];
			}
		}
	}
	else {
		if ((direction[index0] * pointPlusHalfExtent[index2]) >= (direction[index2] * halfExtentToPoint[index0])) {
			float lSquared = (direction[index0] * direction[index0]) + (direction[index1] * direction[index1]);
			float tmp = (lSquared * pointPlusHalfExtent[index2]) - (direction[index2] * ((direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * pointPlusHalfExtent[index1])));
			if (tmp <= (2.0f * lSquared * boxHalfExtent[index2])) {
				const float t = tmp / lSquared;
				lSquared += direction[index2] * direction[index2];
				tmp = pointPlusHalfExtent[index2] - t;
				const float delta = (direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * pointPlusHalfExtent[index1]) + (direction[index2] * tmp);
				distanceToLineOrigin = -delta / lSquared;
				squaredDistance += (halfExtentToPoint[index0] * halfExtentToPoint[index0]) + (pointPlusHalfExtent[index1] * pointPlusHalfExtent[index1]) + (tmp * tmp) + (delta * distanceToLineOrigin);
				point[index0] = boxHalfExtent[index0];
				point[index1] = -boxHalfExtent[index1];
				point[index2] = t - boxHalfExtent[index2];
			}
			else {
				lSquared += direction[index2] * direction[index2];
				const float delta = (direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * pointPlusHalfExtent[index1]) + (direction[index2] * halfExtentToPoint[index2]);
				distanceToLineOrigin = -delta / lSquared;
				squaredDistance += (halfExtentToPoint[index0] * halfExtentToPoint[index0]) + (pointPlusHalfExtent[index1] * pointPlusHalfExtent[index1]) + (halfExtentToPoint[index2] * halfExtentToPoint[index2]) + (delta * distanceToLineOrigin);
				point[index0] = boxHalfExtent[index0];
				point[index1] = -boxHalfExtent[index1];
				point[index2] = boxHalfExtent[index2];
			}
		}
		else {
			float lSquared = (direction[index0] * direction[index0]) + (direction[index2] * direction[index2]);
			float tmp = (lSquared * pointPlusHalfExtent[index1]) - (direction[index1] * ((direction[index0] * halfExtentToPoint[index0]) + (direction[index2] * pointPlusHalfExtent[index2])));
			if (tmp >= 0.0f) {
				if (tmp <= (2.0f * lSquared * boxHalfExtent[index1])) {
					const float t = tmp / lSquared;
					lSquared += direction[index1] * direction[index1];
					tmp = pointPlusHalfExtent[index1] - t;
					const float delta = (direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * tmp) + (direction[index2] * pointPlusHalfExtent[index2]);
					distanceToLineOrigin = -delta / lSquared;
					squaredDistance += (halfExtentToPoint[index0] * halfExtentToPoint[index0]) + (tmp * tmp) + (pointPlusHalfExtent[index2] * pointPlusHalfExtent[index2]) + (delta * distanceToLineOrigin);
					point[index0] = boxHalfExtent[index0];
					point[index1] = t - boxHalfExtent[index1];
					point[index2] = -boxHalfExtent[index2];
				}
				else {
					lSquared += direction[index1] * direction[index1];
					const float delta = (direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * halfExtentToPoint[index1]) + (direction[index2] * pointPlusHalfExtent[index2]);
					distanceToLineOrigin = -delta / lSquared;
					squaredDistance += (halfExtentToPoint[index0] * halfExtentToPoint[index0]) + (halfExtentToPoint[index1] * halfExtentToPoint[index1]) + (pointPlusHalfExtent[index2] * pointPlusHalfExtent[index2]) + (delta * distanceToLineOrigin);
					point[index0] = boxHalfExtent[index0];
					point[index1] = boxHalfExtent[index1];
					point[index2] = -boxHalfExtent[index2];
				}

				return squaredDistance;
			}

			lSquared = (direction[index0] * direction[index0]) + (direction[index1] * direction[index1]);
			tmp = (lSquared * pointPlusHalfExtent[index2]) - (direction[index2] * ((direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * pointPlusHalfExtent[index1])));
			if (tmp >= 0.0f) {
				if (tmp <= (2.0f * lSquared * boxHalfExtent[index2])) {
					const float t = tmp / lSquared;
					lSquared += direction[index2] * direction[index2];
					tmp = pointPlusHalfExtent[index2] - t;
					const float delta = (direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * pointPlusHalfExtent[index1]) + (direction[index2] * tmp);
					distanceToLineOrigin = -delta / lSquared;
					squaredDistance += (halfExtentToPoint[index0] * halfExtentToPoint[index0]) + (pointPlusHalfExtent[index1] * pointPlusHalfExtent[index1]) + (tmp * tmp) + (delta * distanceToLineOrigin);
					point[index0] = boxHalfExtent[index0];
					point[index1] = -boxHalfExtent[index1];
					point[index2] = t - boxHalfExtent[index2];
				}
				else {
					lSquared += direction[index2] * direction[index2];
					const float delta = (direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * pointPlusHalfExtent[index1]) + (direction[index2] * halfExtentToPoint[index2]);
					distanceToLineOrigin = -delta / lSquared;
					squaredDistance += (halfExtentToPoint[index0] * halfExtentToPoint[index0]) + (pointPlusHalfExtent[index1] * pointPlusHalfExtent[index1]) + (halfExtentToPoint[index2] * halfExtentToPoint[index2]) + (delta * distanceToLineOrigin);
					point[index0] = boxHalfExtent[index0];
					point[index1] = -boxHalfExtent[index1];
					point[index2] = boxHalfExtent[index2];
				}

				return squaredDistance;
			}

			lSquared += direction[index2] * direction[index2];
			const float delta = (direction[index0] * halfExtentToPoint[index0]) + (direction[index1] * pointPlusHalfExtent[index1]) + (direction[index2] * pointPlusHalfExtent[index2]);
			distanceToLineOrigin = -delta / lSquared;
			squaredDistance += (halfExtentToPoint[index0] * halfExtentToPoint[index0]) + (pointPlusHalfExtent[index1] * pointPlusHalfExtent[index1]) + (pointPlusHalfExtent[index2] * pointPlusHalfExtent[index2]) + (delta * distanceToLineOrigin);
			point[index0] = boxHalfExtent[index0];
			point[index1] = -boxHalfExtent[index1];
			point[index2] = -boxHalfExtent[index2];
		}
	}

	return squaredDistance;
}

float NtshEngn::PhysicsModule::squaredDistanceLineBoxNo0(Math::vec3& point, const Math::vec3& direction, const Math::vec3& boxHalfExtent, float& distanceToLineOrigin) {
	float squaredDistance = 0.0f;
	
	const Math::vec3 halfExtentToPoint = point - boxHalfExtent;

	const float productDirXHetpY = direction.x * halfExtentToPoint.y;
	const float productDirYHetpX = direction.y * halfExtentToPoint.x;
	
	if (productDirXHetpY >= productDirYHetpX) {
		const float productDirZHetpX = direction.z * halfExtentToPoint.x;
		const float productDirXHetpZ = direction.x * halfExtentToPoint.z;

		if (productDirZHetpX >= productDirXHetpZ) {
			squaredDistance = squaredDistanceLineBoxFace(0, 1, 2, point, direction, boxHalfExtent, halfExtentToPoint, distanceToLineOrigin);
		}
		else {
			squaredDistance = squaredDistanceLineBoxFace(2, 0, 1, point, direction, boxHalfExtent, halfExtentToPoint, distanceToLineOrigin);
		}
	}
	else {
		const float productDirZHetpY = direction.z * halfExtentToPoint.y;
		const float productDirYHetpZ = direction.y * halfExtentToPoint.z;

		if (productDirZHetpY >= productDirYHetpZ) {
			squaredDistance = squaredDistanceLineBoxFace(1, 2, 0, point, direction, boxHalfExtent, halfExtentToPoint, distanceToLineOrigin);
		}
		else {
			squaredDistance = squaredDistanceLineBoxFace(2, 0, 1, point, direction, boxHalfExtent, halfExtentToPoint, distanceToLineOrigin);
		}
	}

	return squaredDistance;
}

float NtshEngn::PhysicsModule::squaredDistanceLineBoxOne0(uint8_t index0, uint8_t index1, uint8_t index2, Math::vec3& point, const Math::vec3& direction, const Math::vec3& boxHalfExtent, float& distanceToLineOrigin) {
	float squaredDistance = 0.0f;

	const float halfExtentToPoint0 = point[index0] - boxHalfExtent[index0];
	const float halfExtentToPoint1 = point[index1] - boxHalfExtent[index1];

	const float product0 = direction[index1] * halfExtentToPoint0;
	const float product1 = direction[index0] * halfExtentToPoint1;

	if (product0 >= product1) {
		point[index0] = boxHalfExtent[index0];

		float halfExtentPlusPoint1 = point[index1] + boxHalfExtent[index1];
		const float delta = product0 - (direction[index0] * halfExtentPlusPoint1);
		if (delta >= 0.0f) {
			const float invLSquare = 1.0f / ((direction[index0] * direction[index0]) + (direction[index1] * direction[index1]));
			squaredDistance += delta * delta * invLSquare;
			point[index1] = -boxHalfExtent[index1];
			distanceToLineOrigin = -((direction[index0] * halfExtentToPoint0) + (direction[index1] * halfExtentPlusPoint1)) * invLSquare;
		}
		else {
			const float inv = 1.0f / direction[index0];
			point[index1] -= product0 * inv;
			distanceToLineOrigin = -halfExtentToPoint0 * inv;
		}
	}
	else {
		point[index1] = boxHalfExtent[index1];

		float halfExtentPlusPoint0 = point[index0] + boxHalfExtent[index0];
		const float delta = product1 - (direction[index1] * halfExtentPlusPoint0);
		if (delta >= 0.0f) {
			const float invLSquare = 1.0f / ((direction[index0] * direction[index0]) + (direction[index1] * direction[index1]));
			squaredDistance += delta * delta * invLSquare;
			point[index0] = -boxHalfExtent[index0];
			distanceToLineOrigin = -((direction[index0] * halfExtentPlusPoint0) + (direction[index1] * halfExtentToPoint1)) * invLSquare;
		}
		else {
			const float inv = 1.0f / direction[index1];
			point[index0] -= product1 * inv;
			distanceToLineOrigin = -halfExtentToPoint1 * inv;
		}
	}

	if (point[index2] < -boxHalfExtent[index2]) {
		const float delta = point[index2] + boxHalfExtent[index2];
		squaredDistance += delta * delta;
		point[index2] = -boxHalfExtent[index2];
	}
	else if (point[index2] > boxHalfExtent[index2]) {
		const float delta = point[index2] - boxHalfExtent[index2];
		squaredDistance += delta * delta;
		point[index2] = boxHalfExtent[index2];
	}

	return squaredDistance;
}

float NtshEngn::PhysicsModule::squaredDistanceLineBoxTwo0(uint8_t index0, uint8_t index1, uint8_t index2, Math::vec3& point, const Math::vec3& direction, const Math::vec3& boxHalfExtent, float& distanceToLineOrigin) {
	float squaredDistance = 0.0f;

	distanceToLineOrigin = (boxHalfExtent[index0] - point[index0]) / direction[index0];

	point[index0] = boxHalfExtent[index0];

	if (point[index1] < -boxHalfExtent[index1]) {
		const float delta = point[index1] + boxHalfExtent[index1];
		squaredDistance += delta * delta;
		point[index1] = -boxHalfExtent[index1];
	}
	else if (point[index1] > boxHalfExtent[index1]) {
		const float delta = point[index1] - boxHalfExtent[index1];
		squaredDistance += delta * delta;
		point[index1] = boxHalfExtent[index1];
	}

	if (point[index2] < -boxHalfExtent[index2]) {
		const float delta = point[index2] + boxHalfExtent[index2];
		squaredDistance += delta * delta;
		point[index2] = -boxHalfExtent[index2];
	}
	else if (point[index2] > boxHalfExtent[index2]) {
		const float delta = point[index2] - boxHalfExtent[index2];
		squaredDistance += delta * delta;
		point[index2] = boxHalfExtent[index2];
	}

	return squaredDistance;
}

float NtshEngn::PhysicsModule::squaredDistanceLineBoxThree0(Math::vec3& point, const Math::vec3& boxHalfExtent) {
	float squaredDistance = 0.0f;
	
	if (point.x < -boxHalfExtent.x) {
		const float delta = point.x + boxHalfExtent.x;
		squaredDistance += delta * delta;
		point.x = -boxHalfExtent.x;
	}
	else if (point.x > boxHalfExtent.x) {
		const float delta = point.x - boxHalfExtent.x;
		squaredDistance += delta * delta;
		point.x = boxHalfExtent.x;
	}

	if (point.y < -boxHalfExtent.y) {
		const float delta = point.y + boxHalfExtent.y;
		squaredDistance += delta * delta;
		point.y = -boxHalfExtent.y;
	}
	else if (point.y > boxHalfExtent.y) {
		const float delta = point.y - boxHalfExtent.y;
		squaredDistance += delta * delta;
		point.y = boxHalfExtent.y;
	}

	if (point.z < -boxHalfExtent.z) {
		const float delta = point.z + boxHalfExtent.z;
		squaredDistance += delta * delta;
		point.z = -boxHalfExtent.z;
	}
	else if (point.z > boxHalfExtent.z) {
		const float delta = point.z - boxHalfExtent.z;
		squaredDistance += delta * delta;
		point.z = boxHalfExtent.z;
	}

	return squaredDistance;
}

float NtshEngn::PhysicsModule::squaredDistanceLineBox(const Math::vec3& lineOrigin, const Math::vec3& lineDirection, const ColliderBox* box, const Math::mat4& boxRotation, float& distanceToLineOrigin, Math::vec3& linePointOnBox) {
	const Math::vec3 xAxis = Math::vec3(boxRotation.x);
	const Math::vec3 yAxis = Math::vec3(boxRotation.y);
	const Math::vec3 zAxis = Math::vec3(boxRotation.z);

	const Math::vec3 boxToLineOrigin = lineOrigin - box->center;
	
	Math::vec3 point = Math::vec3(Math::dot(xAxis, boxToLineOrigin), Math::dot(yAxis, boxToLineOrigin), Math::dot(zAxis, boxToLineOrigin));
	Math::vec3 direction = Math::vec3(Math::dot(xAxis, lineDirection), Math::dot(yAxis, lineDirection), Math::dot(zAxis, lineDirection));

	std::array<bool, 3> reflect;

	for (uint8_t i = 0; i < 3; i++) {
		if (direction[i] < 0.0f) {
			point[i] = -point[i];
			direction[i] = -direction[i];
			reflect[i] = true;
		}
		else {
			reflect[i] = false;
		}
	}

	float squaredDistance;

	if (direction.x > 0.0f) {
		if (direction.y > 0.0f) {
			if (direction.z > 0.0f) {
				squaredDistance = squaredDistanceLineBoxNo0(point, direction, box->halfExtent, distanceToLineOrigin);
			}
			else {
				squaredDistance = squaredDistanceLineBoxOne0(0, 1, 2, point, direction, box->halfExtent, distanceToLineOrigin);
			}
		}
		else {
			if (direction.z > 0.0f) {
				squaredDistance = squaredDistanceLineBoxOne0(0, 2, 1, point, direction, box->halfExtent, distanceToLineOrigin);
			}
			else {
				squaredDistance = squaredDistanceLineBoxTwo0(0, 1, 2, point, direction, box->halfExtent, distanceToLineOrigin);
			}
		}
	}
	else {
		if (direction.y > 0.0f) {
			if (direction.z > 0.0f) {
				squaredDistance = squaredDistanceLineBoxOne0(1, 2, 0, point, direction, box->halfExtent, distanceToLineOrigin);
			}
			else {
				squaredDistance = squaredDistanceLineBoxTwo0(1, 0, 2, point, direction, box->halfExtent, distanceToLineOrigin);
			}
		}
		else {
			if (direction.z > 0.0f) {
				squaredDistance = squaredDistanceLineBoxTwo0(2, 0, 1, point, direction, box->halfExtent, distanceToLineOrigin);
			}
			else {
				squaredDistance = squaredDistanceLineBoxThree0(point, box->halfExtent);
				distanceToLineOrigin = 0.0f;
			}
		}
	}

	for (uint8_t i = 0; i < 3; i++) {
		if (reflect[i]) {
			point[i] = -point[i];
		}

		linePointOnBox = point;
	}

	return squaredDistance;
}

float NtshEngn::PhysicsModule::squaredDistancePointBox(const Math::vec3& point, const ColliderBox* box, const Math::mat4& boxRotation, Math::vec3& pointOnBox) {
	const Math::vec3 xAxis = Math::vec3(boxRotation.x);
	const Math::vec3 yAxis = Math::vec3(boxRotation.y);
	const Math::vec3 zAxis = Math::vec3(boxRotation.z);

	const Math::vec3 boxToPoint = point - box->center;

	Math::vec3 closest = Math::vec3(Math::dot(xAxis, boxToPoint), Math::dot(yAxis, boxToPoint), Math::dot(zAxis, boxToPoint));

	float squaredDistance = 0.0f;

	for (uint8_t i = 0; i < 3; i++) {
		if (closest[i] < -box->halfExtent[i]) {
			const float delta = closest[i] + box->halfExtent[i];
			squaredDistance += delta * delta;
			closest[i] = -box->halfExtent[i];
		}
		else if (closest[i] > box->halfExtent[i]) {
			const float delta = closest[i] - box->halfExtent[i];
			squaredDistance += delta * delta;
			closest[i] = box->halfExtent[i];
		}
	}

	pointOnBox = closest;

	return squaredDistance;
}

float NtshEngn::PhysicsModule::squaredDistanceSegmentBox(const Math::vec3& segmentA, const Math::vec3& segmentB, const ColliderBox* box, const Math::mat4& boxRotation, float& distanceToSegmentOrigin, Math::vec3& segmentPointOnBox) {
	float distanceToLineOrigin;
	Math::vec3 linePointOnBox;
	float squaredDistance = squaredDistanceLineBox(segmentA, segmentB - segmentA, box, boxRotation, distanceToLineOrigin, linePointOnBox);

	if (distanceToLineOrigin >= 0.0f) {
		if (distanceToLineOrigin <= 1.0f) {
			distanceToSegmentOrigin = distanceToLineOrigin;
			segmentPointOnBox = linePointOnBox;

			return squaredDistance;
		}
		else {
			distanceToSegmentOrigin = 1.0f;

			return squaredDistancePointBox(segmentB, box, boxRotation, segmentPointOnBox);
		}
	}
	else {
		distanceToSegmentOrigin = 0.0f;

		return squaredDistancePointBox(segmentA, box, boxRotation, segmentPointOnBox);
	}
}

void NtshEngn::PhysicsModule::boxCapsuleIntersectionInformationRay(const ColliderBox* box, const Math::mat4& boxRotation, const ColliderCapsule* capsule, const Math::vec3& normal, IntersectionInformation& intersectionInformation) {
	intersectionInformation.depth = std::numeric_limits<float>::max();
	
	const Math::vec3 boxMin = -box->halfExtent;
	const Math::vec3 boxMax = box->halfExtent;

	const Math::vec3 rayDirection = -Math::vec3(Math::transpose(boxRotation) * Math::vec4(normal, 0.0f));

	for (uint8_t i = 0; i < 2; i++) {
		Math::vec3 pos;
		if (i == 0) {
			pos = capsule->base;
		}
		else {
			pos = capsule->tip;
		}

		const Math::vec3 rayOrigin = Math::vec3(Math::transpose(boxRotation) * Math::vec4(pos - box->center, 1.0f));

		bool rayAABBIntersection = true;
		float tMin = std::numeric_limits<float>::lowest();
		float tMax = std::numeric_limits<float>::max();

		for (uint8_t j = 0; j < 3; j++) {
			if ((rayDirection[j] > -0.0001f) && (rayDirection[j] < 0.0001f)) {
				if ((rayDirection[j] < boxMin[j]) || (rayDirection[j] > boxMax[j])) {
					rayAABBIntersection = false;
					break;
				}
			}
			else {
				const float invRayDirection = 1.0f / rayDirection[j];
				float t1 = (boxMin[j] - rayOrigin[j]) * invRayDirection;
				float t2 = (boxMax[j] - rayOrigin[j]) * invRayDirection;

				if (t1 > t2) {
					std::swap(t1, t2);
				}

				if (t1 > tMin) {
					tMin = t1;
				}
				if (t2 < tMax) {
					tMax = t2;
				}

				if ((tMin > tMax) || (tMax < 0.0001f)) {
					rayAABBIntersection = false;
					break;
				}
			}
		}

		if ((tMin > tMax) || (tMax < 0.0001f)) {
			rayAABBIntersection = false;
		}

		if (rayAABBIntersection && (tMin < capsule->radius)) {
			const Math::vec3 intersectionPoint = pos - (tMin * normal);

			intersectionInformation.hasIntersected = true;
			intersectionInformation.normal = normal;
			intersectionInformation.depth = std::min(intersectionInformation.depth, capsule->radius - tMin);
			intersectionInformation.relativePoints.push_back({ intersectionPoint - box->center, intersectionPoint - getCenter(capsule) });
		}
	}
}

void NtshEngn::PhysicsModule::boxCapsuleIntersectionInformationEdge(const ColliderBox* box, const Math::mat4& boxRotation, const ColliderCapsule* capsule, const Math::vec3& normal, IntersectionInformation& intersectionInformation) {
	const Math::mat4 boxTransform = Math::translate(box->center) *
		boxRotation *
		Math::scale(box->halfExtent);

	const std::array<Math::vec3, 8> boxCorners = {
		Math::vec3(boxTransform * Math::vec4(-1.0f, -1.0f, -1.0f, 1.0f)),
		Math::vec3(boxTransform * Math::vec4(1.0f, -1.0f, -1.0f, 1.0f)),
		Math::vec3(boxTransform * Math::vec4(1.0f, -1.0f, 1.0f, 1.0f)),
		Math::vec3(boxTransform * Math::vec4(-1.0f, -1.0f, 1.0f, 1.0f)),
		Math::vec3(boxTransform * Math::vec4(-1.0f, 1.0f, -1.0f, 1.0f)),
		Math::vec3(boxTransform * Math::vec4(1.0f, 1.0f, -1.0f, 1.0f)),
		Math::vec3(boxTransform * Math::vec4(1.0f, 1.0f, 1.0f, 1.0f)),
		Math::vec3(boxTransform * Math::vec4(-1.0f, 1.0f, 1.0f, 1.0f))
	};

	const std::array<std::pair<Math::vec3, Math::vec3>, 12> boxEdges = {
		std::pair<Math::vec3, Math::vec3>(boxCorners[0], boxCorners[1]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[1], boxCorners[2]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[2], boxCorners[3]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[3], boxCorners[0]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[4], boxCorners[5]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[5], boxCorners[6]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[6], boxCorners[7]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[7], boxCorners[4]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[0], boxCorners[4]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[1], boxCorners[5]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[2], boxCorners[6]),
		std::pair<Math::vec3, Math::vec3>(boxCorners[3], boxCorners[7])
	};

	Math::vec3 capsuleSegment = capsule->tip - capsule->base;
	capsuleSegment *= 0.01f / capsuleSegment.length();
	const Math::vec3 extendedCapsuleSegmentBase = capsule->base - capsuleSegment;
	const Math::vec3 extendedCapsuleSegmentTip = capsule->tip + capsuleSegment;

	const Math::vec3 extendedCapsuleSegment = extendedCapsuleSegmentTip - extendedCapsuleSegmentBase;

	const Math::vec3 planeAxis = -Math::cross(extendedCapsuleSegment, normal);
	const float planeDistance = -Math::dot(planeAxis, extendedCapsuleSegmentBase);

	uint8_t axis0 = 1;
	uint8_t axis1 = 2;

	const float absPlaneAxisX = std::abs(planeAxis.x);
	const float absPlaneAxisY = std::abs(planeAxis.y);
	const float absPlaneAxisZ = std::abs(planeAxis.z);

	if ((absPlaneAxisY > absPlaneAxisX) && (absPlaneAxisY > absPlaneAxisZ)) {
		axis0 = 2;
		axis1 = 0;
	}
	else if (absPlaneAxisZ > absPlaneAxisX) {
		axis0 = 0;
		axis1 = 1;
	}

	const float coefficient = 1.0f / ((extendedCapsuleSegment[axis1] * normal[axis0]) - (extendedCapsuleSegment[axis0] * normal[axis1]));

	for (uint8_t i = 0; i < 12; i++) {
		Math::vec3 intersectionPoint;
		float distance;

		const float distancePlaneBoxEdgeStart = Math::dot(boxEdges[i].first, planeAxis) + planeDistance;
		const float distancePlaneBoxEdgeEnd = Math::dot(boxEdges[i].second, planeAxis) + planeDistance;

		float tmp = distancePlaneBoxEdgeStart * distancePlaneBoxEdgeEnd;
		if (tmp <= 0.0f) {
			const Math::vec3 boxEdge = boxEdges[i].second - boxEdges[i].first;

			tmp = Math::dot(planeAxis, boxEdge);
			if ((tmp < -0.0001f) || (tmp > 0.0001f)) {
				intersectionPoint = boxEdges[i].first - (boxEdge * (distancePlaneBoxEdgeStart / tmp));
				distance = ((extendedCapsuleSegment[axis0] * (intersectionPoint[axis1] - extendedCapsuleSegmentBase[axis1])) - (extendedCapsuleSegment[axis1] * (intersectionPoint[axis0] - extendedCapsuleSegmentBase[axis0]))) * coefficient;
				if (distance >= 0.0f) {
					const Math::vec3 baseIntersectionPoint = intersectionPoint;
					intersectionPoint -= distance * -normal;

					const float intersectionPointInEdge = ((extendedCapsuleSegmentBase.x - intersectionPoint.x) * (extendedCapsuleSegmentTip.x - intersectionPoint.x)) + ((extendedCapsuleSegmentBase.y - intersectionPoint.y) * (extendedCapsuleSegmentTip.y - intersectionPoint.y)) + ((extendedCapsuleSegmentBase.z - intersectionPoint.z) * (extendedCapsuleSegmentTip.z - intersectionPoint.z));
					if ((intersectionPointInEdge < 0.0f) && (distance < capsule->radius)) {
						const Math::vec3 intersectionPointBoxRelative = baseIntersectionPoint - box->center;
						const Math::vec3 intersectionPointCapsuleRelative = baseIntersectionPoint - getCenter(capsule);

						if (!intersectionInformation.relativePoints.empty()) {
							if ((intersectionPointBoxRelative == intersectionInformation.relativePoints[0].first) && (intersectionPointCapsuleRelative == intersectionInformation.relativePoints[0].second)) {
								continue;
							}
						}

						intersectionInformation.hasIntersected = true;
						intersectionInformation.normal = normal;
						intersectionInformation.depth = std::min(intersectionInformation.depth, capsule->radius - distance);
						intersectionInformation.relativePoints.push_back({ intersectionPointBoxRelative, intersectionPointCapsuleRelative });
					}
				}
			}
		}
	}
}

std::vector<NtshEngn::Math::vec3> NtshEngn::PhysicsModule::clipEdgesToBox(const std::array<std::pair<Math::vec3, Math::vec3>, 12>& edges, const ColliderBox* box, const Math::mat4& boxRotation) {
	std::vector<Math::vec3> intersectionPoints;

	// For each plane of the box
	for (uint8_t i = 0; i < 6; i++) {
		Math::vec3 planeAxis;
		float planeDistance;
		if (i % 2 == 0) {
			planeAxis = boxRotation[i / 2];
			planeDistance = Math::dot(boxRotation[i / 2], (box->center + (boxRotation[i / 2] * box->halfExtent[i / 2])));
		}
		else {
			planeAxis = -boxRotation[i / 2];
			planeDistance = -Math::dot(boxRotation[i / 2], (box->center - (boxRotation[i / 2] * box->halfExtent[i / 2])));
		}

		// For each edge
		for (uint8_t j = 0; j < 12; j++) {
			const Math::vec3 edge = edges[j].second - edges[j].first;
			const float planeAxisDotEdge = Math::dot(planeAxis, edge);
			if ((planeAxisDotEdge > -0.0001f) && (planeAxisDotEdge < 0.0001f)) {
				continue;
			}

			const float planeAxisDotEdgeStart = Math::dot(planeAxis, edges[j].first);
			const float t = (planeDistance - planeAxisDotEdgeStart) / planeAxisDotEdge;

			if ((t >= 0.0f) && (t <= 1.0f)) {
				const Math::vec3 intersectionPoint = edges[j].first + (edge * t);
				const Math::vec3 intersectionPointDir = intersectionPoint - box->center;

				// Check if intersection point is inside box
				bool isInsideBox = true;
				for (uint8_t k = 0; k < 3; k++) {
					const float intersectionPointDirDotAxisToProject = Math::dot(intersectionPointDir, Math::vec3(boxRotation[k]));

					if ((intersectionPointDirDotAxisToProject < -(box->halfExtent[k] + 0.0001f)) || (intersectionPointDirDotAxisToProject > (box->halfExtent[k] + 0.0001f))) {
						isInsideBox = false;

						break;
					}
				}
				if (isInsideBox) {
					intersectionPoints.push_back(intersectionPoint);
				}
			}
		}
	}

	return intersectionPoints;
}

extern "C" NTSHENGN_MODULE_API NtshEngn::PhysicsModuleInterface* createModule() {
	return new NtshEngn::PhysicsModule;
}

extern "C" NTSHENGN_MODULE_API void destroyModule(NtshEngn::PhysicsModuleInterface* m) {
	delete m;
}