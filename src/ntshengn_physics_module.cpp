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
	else {
		return gjk(shape1, shape2);
	}
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

void NtshEngn::PhysicsModule::onEntityComponentAdded(Entity entity, Component componentID) {
	if (componentID == ecs->getComponentID<Rigidbody>()) {
		m_rigidbodyStates[entity] = RigidbodyState();
	}
}

void NtshEngn::PhysicsModule::onEntityComponentRemoved(Entity entity, Component componentID) {
	if (componentID == ecs->getComponentID<Rigidbody>()) {
		m_rigidbodyStates.erase(entity);
	}
}

void NtshEngn::PhysicsModule::eulerIntegrator(float dtSeconds) {
	for (Entity entity : entities) {
		Rigidbody& entityRigidbody = ecs->getComponent<Rigidbody>(entity);
		RigidbodyState& entityRigidbodyState = m_rigidbodyStates[entity];
		if (!entityRigidbody.isStatic) {
			Transform& entityTransform = ecs->getComponent<Transform>(entity);

			entityRigidbodyState.linearAcceleration = entityRigidbody.force / entityRigidbody.mass;
			entityRigidbodyState.angularAcceleration = entityRigidbody.torque / entityRigidbody.inertia;
			if (entityRigidbody.isAffectedByConstants) {
				entityRigidbodyState.linearAcceleration += m_gravity;
			}

			entityRigidbodyState.linearVelocity += entityRigidbodyState.linearAcceleration * dtSeconds;
			entityRigidbodyState.angularVelocity += entityRigidbodyState.angularAcceleration * dtSeconds;

			entityTransform.position += entityRigidbodyState.linearVelocity * dtSeconds;
			entityTransform.rotation += entityRigidbodyState.angularVelocity * dtSeconds;
		}
		else {
			entityRigidbodyState.linearVelocity = { 0.0f, 0.0f, 0.0f };
			entityRigidbodyState.angularVelocity = { 0.0f, 0.0f, 0.0f };
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
	for (const NarrowphaseCollision& collision : m_narrowphaseCollisions) {
		const Rigidbody& entity1Rigidbody = ecs->getComponent<Rigidbody>(collision.entity1);
		RigidbodyState& entity1RigidbodyState = m_rigidbodyStates[collision.entity1];

		Transform& entity1Transform = ecs->getComponent<Transform>(collision.entity1);
		Collidable entity1Collidable = ecs->getComponent<Collidable>(collision.entity1);

		transform(entity1Collidable.collider.get(), entity1Transform.position, entity1Transform.rotation, entity1Transform.scale);

		const Rigidbody& entity2Rigidbody = ecs->getComponent<Rigidbody>(collision.entity2);
		RigidbodyState& entity2RigidbodyState = m_rigidbodyStates[collision.entity2];

		Transform& entity2Transform = ecs->getComponent<Transform>(collision.entity2);
		Collidable entity2Collidable = ecs->getComponent<Collidable>(collision.entity2);

		transform(entity2Collidable.collider.get(), entity2Transform.position, entity2Transform.rotation, entity2Transform.scale);

		const float invMass1 = 1.0f / entity1Rigidbody.mass;
		const float invMass2 = 1.0f / entity2Rigidbody.mass;
		const float totalInverseMass = invMass1 + invMass2;

		const float invInertia1 = 1.0f / entity1Rigidbody.inertia;
		const float invInertia2 = 1.0f / entity2Rigidbody.inertia;

		const float e = entity1Rigidbody.restitution * entity2Rigidbody.restitution;

		if (entity1Rigidbody.isStatic && entity2Rigidbody.isStatic) {
			continue;
		}

		Math::vec3 entity1LinearVelocity = Math::vec3(0.0f, 0.0f, 0.0f);
		Math::vec3 entity1AngularVelocity = Math::vec3(0.0f, 0.0f, 0.0f);
		Math::vec3 entity2LinearVelocity = Math::vec3(0.0f, 0.0f, 0.0f);
		Math::vec3 entity2AngularVelocity = Math::vec3(0.0f, 0.0f, 0.0f);
		for (size_t i = 0; i < collision.intersectionPoints.size(); i++) {
			// Impulse
			const Math::vec3 relativeIntersectionPoint1 = collision.intersectionPoints[i] - getCenter(entity1Collidable.collider.get());
			const Math::vec3 relativeIntersectionPoint2 = collision.intersectionPoints[i] - getCenter(entity2Collidable.collider.get());
			Math::vec3 angularVelocity1 = Math::cross(entity1RigidbodyState.angularVelocity, relativeIntersectionPoint1);
			Math::vec3 angularVelocity2 = Math::cross(entity2RigidbodyState.angularVelocity, relativeIntersectionPoint2);
			Math::vec3 fullVelocity1 = entity1RigidbodyState.linearVelocity + angularVelocity1;
			Math::vec3 fullVelocity2 = entity2RigidbodyState.linearVelocity + angularVelocity2;

			Math::vec3 relativeVelocity = fullVelocity2 - fullVelocity1;
			float impulseForce = Math::dot(relativeVelocity, collision.intersectionNormal);

			if (impulseForce >= 0.0f) {
				continue;
			}

			const Math::vec3 angular1 = Math::cross(invInertia1 * Math::cross(relativeIntersectionPoint1, collision.intersectionNormal), relativeIntersectionPoint1);
			const Math::vec3 angular2 = Math::cross(invInertia2 * Math::cross(relativeIntersectionPoint2, collision.intersectionNormal), relativeIntersectionPoint2);
			const float angularEffect = Math::dot(angular1 + angular2, collision.intersectionNormal);

			// Apply impulse
			const float j = (-(1.0f + e) * impulseForce) / (totalInverseMass + angularEffect);
			const Math::vec3 impulse = j * collision.intersectionNormal;

			if (!entity1Rigidbody.isStatic) {
				entity1LinearVelocity -= (invMass1 * impulse) / static_cast<float>(collision.intersectionPoints.size());
				entity1AngularVelocity -= (invInertia1 * Math::cross(relativeIntersectionPoint1, impulse)) / static_cast<float>(collision.intersectionPoints.size());
			}

			if (!entity2Rigidbody.isStatic) {
				entity2LinearVelocity += (invMass2 * impulse) / static_cast<float>(collision.intersectionPoints.size());
				entity2AngularVelocity += (invInertia2 * Math::cross(relativeIntersectionPoint2, impulse)) / static_cast<float>(collision.intersectionPoints.size());
			}

			// Friction
			angularVelocity1 = Math::cross(entity1RigidbodyState.angularVelocity, relativeIntersectionPoint1);
			angularVelocity2 = Math::cross(entity2RigidbodyState.angularVelocity, relativeIntersectionPoint2);
			fullVelocity1 = entity1RigidbodyState.linearVelocity + angularVelocity1;
			fullVelocity2 = entity2RigidbodyState.linearVelocity + angularVelocity2;

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

			if (!entity1Rigidbody.isStatic) {
				entity1LinearVelocity -= (invMass1 * friction) / static_cast<float>(collision.intersectionPoints.size());
				entity1AngularVelocity -= (invInertia1 * Math::cross(relativeIntersectionPoint1, friction)) / static_cast<float>(collision.intersectionPoints.size());
			}

			if (!entity2Rigidbody.isStatic) {
				entity2LinearVelocity += (invMass2 * friction) / static_cast<float>(collision.intersectionPoints.size());
				entity2AngularVelocity += (invInertia2 * Math::cross(relativeIntersectionPoint2, friction)) / static_cast<float>(collision.intersectionPoints.size());
			}
		}
		entity1RigidbodyState.linearVelocity += entity1LinearVelocity;
		entity1RigidbodyState.angularVelocity += entity1AngularVelocity;
		entity2RigidbodyState.linearVelocity += entity2LinearVelocity;
		entity2RigidbodyState.angularVelocity += entity2AngularVelocity;

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
			narrowphaseCollision.intersectionNormal = intersectionInformation.intersectionNormal;
			narrowphaseCollision.intersectionDepth = intersectionInformation.intersectionDepth;
			narrowphaseCollision.intersectionPoints = intersectionInformation.intersectionPoints;

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
	intersectionInformation.intersectionPoints = { sphere1->center + (intersectionInformation.intersectionNormal * sphere1->radius) };

	return intersectionInformation;
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

	intersectionInformation.intersectionDepth = std::numeric_limits<float>::max();
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

		if (depth < intersectionInformation.intersectionDepth) {
			const float flipNormal = (Math::dot(box2->center - box1->center, axisToTest[i]) < 0.0f) ? -1.0f : 1.0f;
			intersectionInformation.hasIntersected = true;
			intersectionInformation.intersectionNormal = axisToTest[i] * flipNormal;
			intersectionInformation.intersectionDepth = depth;
		}
	}

	if (intersectionInformation.hasIntersected) {
		std::vector<Math::vec3> intersectionPoints = clipEdgesToBox(box1Edges, box2, box2Rotation);
		intersectionInformation.intersectionPoints.insert(intersectionInformation.intersectionPoints.end(), intersectionPoints.begin(), intersectionPoints.end());

		intersectionPoints = clipEdgesToBox(box2Edges, box1, box1Rotation);
		intersectionInformation.intersectionPoints.insert(intersectionInformation.intersectionPoints.end(), intersectionPoints.begin(), intersectionPoints.end());
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
	const Math::vec3 outsidePoint = sphere->center - (intersectionNormal * sphere->radius);

	intersectionInformation.hasIntersected = true;
	intersectionInformation.intersectionNormal = intersectionNormal;
	intersectionInformation.intersectionDepth = sphere->radius - distance;
	intersectionInformation.intersectionPoints = { closestPoint };

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderBox* box, const ColliderCapsule* capsule) {
	const Math::vec3 closestPointOnCapsule = closestPointOnSegment(box->center, capsule->base, capsule->tip);

	ColliderSphere sphereFromCapsule;
	sphereFromCapsule.center = closestPointOnCapsule;
	sphereFromCapsule.radius = capsule->radius;

	return intersect(box, &sphereFromCapsule);
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
		intersectionInformation.intersectionNormal = intersectionInformation.intersectionNormal * -1.0f;
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderSphere* sphere, const ColliderBox* box) {
	IntersectionInformation intersectionInformation = intersect(box, sphere);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.intersectionNormal = intersectionInformation.intersectionNormal * -1.0f;
	}

	return intersectionInformation;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderCapsule* capsule, const ColliderBox* box) {
	IntersectionInformation intersectionInformation = intersect(box, capsule);
	if (intersectionInformation.hasIntersected) {
		intersectionInformation.intersectionNormal = intersectionInformation.intersectionNormal * -1.0f;
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

			const std::tuple<Math::vec3, float, std::pair<Math::vec3, Math::vec3>> intersectionNormalAndDepth = epa(shape1, shape2, simplex);
			intersectionInformation.intersectionNormal = std::get<0>(intersectionNormalAndDepth);
			intersectionInformation.intersectionDepth = std::get<1>(intersectionNormalAndDepth) + 0.001f;
			intersectionInformation.intersectionPoints = { std::get<2>(intersectionNormalAndDepth).first, std::get<2>(intersectionNormalAndDepth).second };

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

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::support(const ColliderShape* shape1, const ColliderShape* shape2, const Math::vec3& direction) {
	const Math::vec3 p1 = getFarthestPointInDirection(shape1, direction);
	const Math::vec3 p2 = getFarthestPointInDirection(shape2, direction * -1.0f);

	return p1 - p2;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderShape* shape, const Math::vec3& direction) {
	if (shape->getType() == ColliderShapeType::Box) {
		return getFarthestPointInDirection(static_cast<const ColliderBox*>(shape), direction);
	}
	else if (shape->getType() == ColliderShapeType::Sphere) {
		return getFarthestPointInDirection(static_cast<const ColliderSphere*>(shape), direction);
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		return getFarthestPointInDirection(static_cast<const ColliderCapsule*>(shape), direction);
	}

	return Math::vec3(0.0f, 0.0f, 0.0f);
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderBox* box, const Math::vec3& direction) {
	const Math::vec3 min = Math::vec3(-1.0f, -1.0f, -1.0f);
	const Math::vec3 max = Math::vec3(1.0f, 1.0f, 1.0f);

	const Math::mat4 transformMatrix = Math::translate(box->center) *
		Math::rotate(box->rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(box->rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(box->rotation.z, Math::vec3(0.0f, 0.0f, 1.0f)) *
		Math::scale(box->halfExtent);

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

	float maxDistance = std::numeric_limits<float>::lowest();
	Math::vec3 furthestPoint;
	for (uint8_t i = 0; i < 8; i++) {
		boxPoints[i] = Math::vec3(transformMatrix * Math::vec4(boxPoints[i], 1.0f));

		const float distance = Math::dot(boxPoints[i], direction);
		if (distance > maxDistance) {
			maxDistance = distance;
			furthestPoint = boxPoints[i];
		}
	}

	return furthestPoint;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderSphere* sphere, const Math::vec3& direction) {
	const float directionLength = direction.length();

	return sphere->center + direction * (sphere->radius / directionLength);
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getFarthestPointInDirection(const ColliderCapsule* capsule, const Math::vec3& direction) {
	const float directionLength = direction.length();

	return ((Math::dot(direction, Math::vec3(capsule->tip - capsule->base)) >= 0.0) ? capsule->tip : capsule->base) + direction * (capsule->radius / directionLength);
}

std::tuple<NtshEngn::Math::vec3, float, std::pair<NtshEngn::Math::vec3, NtshEngn::Math::vec3>> NtshEngn::PhysicsModule::epa(const ColliderShape* shape1, const ColliderShape* shape2, GJKSimplex& simplex) {
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
	std::pair<Math::vec3, Math::vec3> intersectionPoints;
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
				intersectionPoints.first = polytope[faces[minTriangle]];
				intersectionPoints.first = polytope[faces[minTriangle + 1]];
			}

			faces.insert(faces.end(), newFaces.begin(), newFaces.end());
			normals.insert(normals.end(), newNormals.begin(), newNormals.end());
		}
	}

	return { minNormal, minDistance, intersectionPoints };
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
	sphere->center = translation;
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

extern "C" NTSHENGN_MODULE_API NtshEngn::PhysicsModuleInterface * createModule() {
	return new NtshEngn::PhysicsModule;
}

extern "C" NTSHENGN_MODULE_API void destroyModule(NtshEngn::PhysicsModuleInterface * m) {
	delete m;
}