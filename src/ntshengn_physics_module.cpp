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

	return componentMask;
}

void NtshEngn::PhysicsModule::eulerIntegrator(float dtSeconds) {
	for (Entity entity : entities) {
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

void NtshEngn::PhysicsModule::collisionsDetection() {
	collisionsBroadphase();
	collisionsNarrowphase();
}

void NtshEngn::PhysicsModule::collisionsResponse() {
	std::unordered_map<Entity, ObjectDuringCollisionResponseState> objectStates;

	for (const NarrowphaseCollision& collision : m_narrowphaseCollisions) {
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
				entity2AngularVelocityDelta += invInertia2 * Math::cross(collision.relativeIntersectionPoints[i].second, impulse);

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
}

void NtshEngn::PhysicsModule::collisionsBroadphase() {
	m_broadphaseCollisions.clear();

	Math::vec3 sceneAABBMin;
	Math::vec3 sceneAABBMax;

	std::unordered_map<Entity, AABB> entityAABBs;
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

		entityAABBs[entity] = entityAABB;

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

	octree.execute([this](std::vector<Octree<Entity>::Entry>& entries) {
		for (std::vector<Octree<Entity>::Entry>::iterator i = entries.begin(); i != entries.end(); i++) {
			for (std::vector<Octree<Entity>::Entry>::iterator j = std::next(i); j != entries.end(); j++) {
				BroadphaseCollision broadphaseCollision;
				broadphaseCollision.entity1 = std::min(i->object, j->object);
				broadphaseCollision.entity2 = std::max(i->object, j->object);

				const Rigidbody& entity1Rigidbody = ecs->getComponent<Rigidbody>(broadphaseCollision.entity1);
				const Rigidbody& entity2Rigidbody = ecs->getComponent<Rigidbody>(broadphaseCollision.entity2);

				if (!entity1Rigidbody.isStatic || !entity2Rigidbody.isStatic) {
					m_broadphaseCollisions.insert(broadphaseCollision);
				}
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
	const Math::vec3 outsidePoint = (intersectionNormal * -1.0f) * sphere->radius;

	intersectionInformation.hasIntersected = true;
	intersectionInformation.normal = intersectionNormal;
	intersectionInformation.depth = sphere->radius - distance;
	intersectionInformation.relativePoints = { { closestPoint - box->center, outsidePoint } };

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderBox* box, const ColliderCapsule* capsule) {
	ColliderSphere sphereFromCapsule;
	sphereFromCapsule.center = closestPointOnSegment(box->center, capsule->base, capsule->tip);
	sphereFromCapsule.radius = capsule->radius;

	return intersect(box, &sphereFromCapsule);
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
	intersectionInformation.relativePoints = { { intersectionInformation.normal * sphere1->radius, (intersectionInformation.normal * -1.0f) * sphere2->radius } };

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
		intersectionInformation.normal = intersectionInformation.normal * -1.0f;
		for (size_t i = 0; i < intersectionInformation.relativePoints.size(); i++) {
			std::swap(intersectionInformation.relativePoints[i].first, intersectionInformation.relativePoints[i].second);
		}
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderSphere* sphere, const ColliderBox* box) {
	IntersectionInformation intersectionInformation = intersect(box, sphere);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.normal = intersectionInformation.normal * -1.0f;
		for (size_t i = 0; i < intersectionInformation.relativePoints.size(); i++) {
			std::swap(intersectionInformation.relativePoints[i].first, intersectionInformation.relativePoints[i].second);
		}
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule, const ColliderBox* box) {
	IntersectionInformation intersectionInformation = intersect(box, capsule);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.normal = intersectionInformation.normal * -1.0f;
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

std::vector<NtshEngn::Math::vec3> NtshEngn::PhysicsModule::clipEdgesToBox(const std::array<std::pair<Math::vec3, Math::vec3>, 12>& edges, const ColliderBox* box, const Math::mat4& boxRotation) {
	std::vector<Math::vec3> intersectionPoints;

	// For each plane of the box
	for (uint8_t i = 0; i < 6; i++) {
		Math::vec3 planeAxis;
		float planeDistance;
		if (i % 2 == 0) {
			planeAxis = boxRotation[i / 2];
			planeDistance = Math::dot(planeAxis, (box->center + (planeAxis * box->halfExtent[i / 2])));
		}
		else {
			planeAxis = boxRotation[i / 2] * -1.0f;
			planeDistance = -Math::dot(planeAxis, (box->center - (planeAxis * box->halfExtent[i / 2])));
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

					if ((intersectionPointDirDotAxisToProject < -box->halfExtent[k]) || (intersectionPointDirDotAxisToProject > box->halfExtent[k])) {
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