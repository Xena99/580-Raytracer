#include "Raytracer.h"
#include <iostream>
#include "ExternalPlugins/json.hpp"
#include "ExternalPlugins/CImg/CImg.h"
#include <fstream>
#include <random>

//For timing render duration
#include <chrono> 
/// <summary>
/// For float comparison
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
bool Raytracer::NearlyEquals(float a, float b) {
	return (std::abs(a - b) < EPSILON);
}

//TODO: For each shape compute model matrix
/// <summary>
/// Takes in a ray and computes intersection with scene, if has hit outputs closest hit with hit point in world space, distance and normal in hitInfo struct
/// </summary>
/// <param name="ray"></param>
/// <param name="hitInfo"></param>
/// <returns></returns>
/// The depth parameter in recursive ray tracing is used to control the maximum number of times a ray can be reflected
Raytracer::Pixel Raytracer::Raycast(Ray& ray, int bounces) {
	RaycastHitInfo info;
	if (!IntersectScene(ray, info)) {
		return BG_COLOR; // No hit, return background color
	}

	const Material& material = *info.material;
	Pixel localColor = SHADOW_COLOR;

	//Loop through all light sources and determine if pixel is occluded from it
	//If is, no contribution added, else compute local Phong lighting and add the contribution
	for (Light& light : mScene->lights) {
		//Calculate ambient light
		if (light.lightType == Light::Ambient) {
			Vector3 _ambientColor = material.surfaceColor * material.Ka * light.color * light.intensity;

			//is this where we multiply ao by the color?
			_ambientColor = _ambientColor * CalculateAmbientOcclusion(info.hitPoint, info.normal);
			
			// Convert to Pixel format, conversion clamps for you
			Pixel _ambientCol(_ambientColor);
			localColor = localColor + _ambientCol;
			continue;
		}

		//Compute the direction to light (hit point to light)
		//Directional light dir is already stored
		Vector3 lightDir;
		if (light.lightType == Light::Directional) {
			//Kevin: negated light direction assuming incoming light is towards pixel
			//However for shadow ray we are shooting from pixel to light, thus may need to negate the direction
			lightDir = -(light.direction);
		}
		//For point light we need to compute it since it stores position
		else if (light.lightType == Light::Point) {
			lightDir = (light.position - info.hitPoint);
		}
		lightDir.normalize();
		//Make new ray from intersection point to light source
		Ray lightRay(info.hitPoint + lightDir * SHADOW_CLIPPING_OFFSET, lightDir);
		RaycastHitInfo lightInfo;

		//For directional light, this value is not used as directional light doesn't have position property
		float distToLight = (light.position - info.hitPoint).length();

		//If pixel is occluded to this current light source, no light contribution is added
		//If intersection is past the distance to light, then means it's behind light, thus still contribution
		if (!IntersectScene(lightRay, lightInfo) || (lightInfo.distance > distToLight && light.lightType == Light::Point)) {
			// Calculate local color using Phong lighting or other shading model
			localColor = localColor + CalculateLocalColor(info, light, material);
		}
		else {
			localColor = localColor + SHADOW_COLOR;
		}
	}

	//Clamp the color so far after light contributions are added
	localColor.clamp();
	//If we have bounces left, calculate reflection and refraction
	if (bounces > 0) {
		float _kt;
		float _kr;

		Pixel reflectionColor;
		Pixel refractionColor;
		// Add reflection
		if (material.Ks > 0) {
			// Calculate reflection ray
			Vector3 reflectionRayDir = Vector3::reflect(ray.direction, info.normal);
			reflectionRayDir.normalize();
			Ray reflectionRay(info.hitPoint + reflectionRayDir * SHADOW_CLIPPING_OFFSET, reflectionRayDir);

			// For each hit, if the material of the hit object is reflective, 
			// you need to trace a new ray from the hit point in the reflection direction. 
			// This new ray will then be used to determine what the original ray "sees" in the reflection.
			reflectionColor = Raycast(reflectionRay, bounces - 1);

		}

		// Add refraction
		if (material.Kt > 0) {
			Vector3 refractionRayDir = CalculateRefraction(ray.direction, info.normal, material.refractiveIndex);
			Ray refractionRay(info.hitPoint + refractionRayDir * SHADOW_CLIPPING_OFFSET, refractionRayDir);
			refractionColor = Raycast(refractionRay, bounces - 1);
		}

		ComputeFresnel(material.refractiveIndex, info.normal, ray.direction, _kr, _kt);
		// Blend the reflection and refraction colors based on Fresnel effect and material's reflective and transmissive properties
		Pixel finalReflectionColor = reflectionColor * _kr * material.Ks;
		Pixel finalRefractionColor = refractionColor * _kt * material.Kt;

		// Combine local color, reflection, and refraction
		float albedoColorFactor = 1 - material.Ks - material.Kt;
		albedoColorFactor = std::max(albedoColorFactor, 0.0f); // Ensures the factor is not negative.

		// Combine the colors with their respective factors.
		localColor = (localColor * albedoColorFactor) + (finalReflectionColor * material.Ks) + (finalRefractionColor * material.Kt);
		//localColor = localColor * CalculateAmbientOcclusion(info.hitPoint, info.normal);
	}

	return localColor.clamp();
}

void Raytracer::ComputeFresnel(float indexOfRefraction, const Vector3& normal, const Vector3& incident, float& Kr, float& Kt) {
	// Calculate the cosine of the angle of incidence
	float cosi = Clipf(Vector3::dot(incident, normal), -1.0f, 1.0f);

	// Check if we're inside the surface
	bool inside = cosi > 0;
	float eta_i = 1; // Index of refraction of the incident medium (air in this case)
	float eta_t = indexOfRefraction; // Index of refraction of the transmitting medium (object)

	// If we're inside the object, we need to invert indices of refraction
	if (inside) {
		std::swap(eta_i, eta_t);
		cosi = -cosi; // Flip the sign of cosi since we're inside the surface
	}

	// Snell's law
	float sint = eta_i / eta_t * sqrtf(std::max(0.f, 1 - cosi * cosi));

	// Total internal reflection check
	if (sint >= 1) {
		Kr = 1; // No light passes through the surface, it's all reflected
		Kt = 0;
	}
	else {
		float cost = sqrtf(std::max(0.f, 1 - sint * sint));
		cosi = fabsf(cosi);

		// Compute Fresnel reflectance using Schlick's approximation
		float Rs = ((eta_t * cosi) - (eta_i * cost)) / ((eta_t * cosi) + (eta_i * cost));
		float Rp = ((eta_i * cosi) - (eta_t * cost)) / ((eta_i * cosi) + (eta_t * cost));
		Kr = (Rs * Rs + Rp * Rp) / 2;

		// Transmission is 1 - the reflectance
		Kt = 1 - Kr;
	}
}

Raytracer::Vector3 Raytracer::CalculateRefraction(const Vector3& I, const Vector3& N, const float& indexM2) {
	float cosi = I.dot(N);

	if (cosi < -1) {
		cosi = -1;
	}
	else if (cosi > 1) {
		cosi = 1;
	}

	float indexM1Ref = 1; //hard coding using air as default
	float indexM2Ref = indexM2;

	Vector3 n = N;

	if (cosi < 0) {
		cosi = -1 * cosi;
	}
	else {
		float temp = indexM1Ref;
		indexM1Ref = indexM2Ref;
		indexM2Ref = temp;
		n = -N;
	}

	float eta = indexM1Ref / indexM2Ref;

	float k = 1 - eta * eta * (1 - cosi * cosi);

	if (k < 0) {
		return Vector3(0, 0, 0);
	}
	else {
		return I * eta + n * (eta * cosi - sqrtf(k));
	}
}


float Raytracer::Clipf(float input, int min, int max) {
	if (input < min) return min;
	if (input > max) return max;
	return input;
}


Raytracer::Pixel Raytracer::CalculateLocalColor(const RaycastHitInfo& hitInfo, const Light& light, const Material& material) {
	Vector3 lightVector;
	if (light.lightType == Light::Point) {
		lightVector = light.position - hitInfo.hitPoint; // From hit point to light source
		lightVector.normalize(); // It's important to normalize the direction
	}
	else {
		lightVector = light.direction * -1; // For directional lights
		lightVector.normalize();
	}
	//Assign normal to use for lighting based on mesh type
	Vector3 _normal;
	switch (hitInfo.type) {
	case Mesh::RT_POLYGON: {
		Vector3 triVertNormals[3] = { hitInfo.triangle->v0.vertexNormal, hitInfo.triangle->v1.vertexNormal , hitInfo.triangle->v2.vertexNormal };
		//Interpolated normal
		_normal = InterpolateVector3(triVertNormals, hitInfo.alpha, hitInfo.beta, hitInfo.gamma, true);
		break;
	}
	case Mesh::RT_SPHERE: {
		_normal = hitInfo.normal;
		break;
	}
	}
	_normal.normalize();

	//Lighting = diffuse + specular
	Vector3 lighting;

	float diffuseStrength = fmax(lightVector.dot(_normal), 0);
	Vector3 _diffuse = light.color * diffuseStrength * light.intensity;

	//Specular
	Vector3 reflection = Vector3::reflect(lightVector, _normal);
	reflection.normalize();

	Vector3 viewVector = mScene->camera.from - hitInfo.hitPoint;
	viewVector.normalize();

	float specularStrength = fmax(Vector3::dot(viewVector, reflection), 0);
	specularStrength = std::powf(specularStrength, material.specularExponet);
	Vector3 _specular = light.color * specularStrength * light.intensity;

	lighting = _diffuse * material.Kd + _specular * material.Ks;

	Vector3 color = material.surfaceColor * lighting;

	color.x = Clipf(color.x, 0, 1);
	color.y = Clipf(color.y, 0, 1);
	color.z = Clipf(color.z, 0, 1);

	Pixel finalColor = Pixel(color);

	return finalColor;
}

Raytracer::Vector3 Raytracer::RandomUnitVector() {
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	std::default_random_engine generator;
	Raytracer::Vector3 p;
	do {
		p = Raytracer::Vector3(distribution(generator), distribution(generator), distribution(generator)) * 2.0f - Raytracer::Vector3(1, 1, 1);
	} while (p.length_squared() >= 1.0f);
	return p;
}

Raytracer::Vector3 Raytracer::RandomInHemisphere(const Raytracer::Vector3& normal) {
	Raytracer::Vector3 in_unit_sphere = RandomUnitVector(); // Function to create a random unit vector
	if (in_unit_sphere.dot(normal) > 0.0) { // In the same hemisphere as the normal
		return in_unit_sphere;
	}
	else {
		return -in_unit_sphere;
	}
}

//float Raytracer::CalculateAmbientOcclusion(const Raytracer::Vector3& hitPoint, const Raytracer::Vector3& normal) {
//	int numSamples = 16; // Number of samples to check for occlusion
//	float occlusionRadius = 0.5f; // Max distance to check for occlusion
//	int occludedCount = 0;
//
//	for (int i = 0; i < numSamples; ++i) {
//		Raytracer::Vector3 randomDir = RandomInHemisphere(normal);
//		Ray occlusionRay(hitPoint, randomDir);
//		RaycastHitInfo occlusionInfo;
//
//		if (IntersectScene(occlusionRay, occlusionInfo)) {
//			if (occlusionInfo.distance < occlusionRadius) {
//				occludedCount++;
//			}
//		}
//	}
//
//	// Return the proportion of the hemisphere that is NOT occluded
//	return 1.0f - (float)occludedCount / (float)numSamples;
//}

float Raytracer::CalculateAmbientOcclusion(const Raytracer::Vector3& hitPoint, const Raytracer::Vector3& normal) {
	// Pseudocode for calculating ambient occlusion
	int numSamples = 128; // Number of samples to check for occlusion
	float occlusion = 0.0;

	for (int i = 0; i < numSamples; i++) {
		Raytracer::Vector3 randomDir = RandomInHemisphere(normal);
		Ray occlusionRay(hitPoint+randomDir*SHADOW_CLIPPING_OFFSET, randomDir);
		RaycastHitInfo occlusionInfo;
		
		if (IntersectScene(occlusionRay,occlusionInfo)) {
			occlusion += 1.0f;
		}
	}
	std::cout << ((float)occlusion / (float)numSamples);
	return 1.0f-((float)occlusion / (float)numSamples);
}


Raytracer::Vector3 Raytracer::InterpolateVector3(const Vector3 vectors[], float alpha, float beta, float gamma, bool isNormal) {
	Vector3 result = vectors[0] * alpha + vectors[1] * beta + vectors[2] * gamma;
	if (isNormal)
		result.normalize();
	return result;
}

/// <summary>
/// Möller–Trumbore intersection algorithm, given a ray and triangle, test intersection
/// If successful intersection, outputs world space hit point and hit normal
/// </summary>
/// <param name="ray"></param>
/// <param name="triangle"></param>
/// <param name="hitInfo"></param>
/// <returns></returns>
bool Raytracer::IntersectTriangle(const Ray& ray, const Triangle& triangle, RaycastHitInfo& hitInfo, const Matrix& modelMatrix) {
	// Compute the inverse of the model matrix for transforming the ray
	Matrix inverseModelMatrix;
	Matrix::Inverse(modelMatrix, inverseModelMatrix);

	Vector3 _v0 = modelMatrix.TransformPoint(triangle.v0.vertexPos);
	Vector3 _v1 = modelMatrix.TransformPoint(triangle.v1.vertexPos);
	Vector3 _v2 = modelMatrix.TransformPoint(triangle.v2.vertexPos);

	//Image a plane where the triangle lies on, the ray will intersect with the plane if the plane is in front of the ray
	//Once intersection happens with the plane, we can then figure out if the intersection point is inside the triangle
	//Even if a plane is in front of the ray, the ray will miss it if it's parallel to the ray

	//Get plane normal, which will be the same for the triangle normal since triangle lies flat on the imaginary plane
	Vector3 _edge1 = _v1 - _v0;
	Vector3 _edge2 = _v2 - _v0;
	Vector3 planeNormal = Vector3::cross(_edge1, _edge2);
	planeNormal.normalize();

	float planeNormalDotRayDir = Vector3::dot(planeNormal, ray.direction);

	//Dot product of two vectors is 0 means they are perpendicular to each other, 0, and 1 are pointing along same dir or opposite
	//Perpendicular with the normal means no intersection and you have to go along the normal to point to the plane
	if (NearlyEquals(planeNormalDotRayDir, 0)) {
		return false;
	}

	//Ax + By + Cz + D = 0 for plane, where A B C is the normal to plane N = (A, B, C), x y z is any point on the plane
	//We can use any vertex x y z to substitute 
	float D = -Vector3::dot(planeNormal, _v0);

	//Since P is a point on the plane, we can substitute it into Ax + By + Cz + D = 0, and we already know D from above
	//We can calculate t, which is the distance from ray origin to intersection point
	float t = -(Vector3::dot(planeNormal, ray.origin) + D) / planeNormalDotRayDir;
	if (t <= EPSILON) {
		return false;  // Intersection is behind the ray
	}

	//P = RayOrigin + t * RayDirection -> point on plane
	Vector3 pointOnPlane = ray.origin + ray.direction * t;

	float totalArea = CalcTriangleAreaSigned(_v0, _v1, _v2, planeNormal);

	// Calculate the areas for the barycentric coordinates
	float alpha = CalcTriangleAreaSigned(pointOnPlane, _v1, _v2, planeNormal) / totalArea;
	float beta = CalcTriangleAreaSigned(_v0, pointOnPlane, _v2, planeNormal) / totalArea;
	float gamma = CalcTriangleAreaSigned(_v0, _v1, pointOnPlane, planeNormal) / totalArea;

	if (alpha < 0 || beta < 0 || gamma < 0) return false;

	//Write info only when pt is inside
	hitInfo.hitPoint = pointOnPlane;

	hitInfo.type = Mesh::RT_POLYGON;
	hitInfo.normal = planeNormal;
	hitInfo.normal.normalize();
	hitInfo.distance = t;
	hitInfo.alpha = alpha;
	hitInfo.beta = beta;
	hitInfo.gamma = gamma;
	return true;
}

/// <summary>
/// Intersecting with a sphere, tests intersection
/// </summary>
/// <param name="ray"></param>
/// <param name="triangle"></param>
/// <param name="hitInfo"></param>
/// <param name="modelMatrix"></param>
/// <returns></returns>
bool Raytracer::IntersectSphere(const Ray& ray, const Sphere& sphere, RaycastHitInfo& hitInfo, const Matrix& modelMatrix) {
	// In object space, the sphere is at its local origin, so we can simplify the intersection calculation
	Vector3 oc = ray.origin - modelMatrix.GetTranslation();  // Sphere center is at local origin after transformation, so no need to subtract its position
	float b = 2.0f * Vector3::dot(ray.direction, oc);
	float c = Vector3::dot(oc, oc) - (sphere.radius * sphere.radius);

	// Calculate the discriminant for the quadratic equation
	float discriminant = (b * b) - (4 * 1.0f * c);
	if (discriminant < EPSILON) return false;  // No intersection

	float sqrtDiscriminant = sqrt(discriminant);
	float t0 = (-b + sqrtDiscriminant) / (float)2;
	float t1 = (-b - sqrtDiscriminant) / (float)2;

	if (!GreaterThanZero(t0) && !GreaterThanZero(t1)) {
		return false;  // Both intersections are behind the ray origin
	}

	// Find the nearest positive intersection
	if (!GreaterThanZero(t0)) {
		if (!GreaterThanZero(t1)) {
			return false;
		}
		else {
			hitInfo.distance = t1;
		}
	}
	else {
		if (!GreaterThanZero(t1)) {
			hitInfo.distance = t0;
		}
		else {
			hitInfo.distance = std::fmin(t0, t1);
		}
	}

	// Transform the hit point and normal back to world space
	hitInfo.hitPoint = ray.origin + (ray.direction * hitInfo.distance);

	//Sphere.position is world space, thus transform hit point to world space then get direction from sphere position to it
	hitInfo.normal = hitInfo.hitPoint - modelMatrix.GetTranslation();
	hitInfo.normal.normalize();

	hitInfo.type = Mesh::RT_SPHERE;
	return true;
}


/// <summary>
/// Loops through all shape in scene to test intersection, sets material properties because that's a property within shape
/// </summary>
/// <param name="ray"></param>
/// <param name="hitInfo"></param>
/// <returns></returns>
bool Raytracer::IntersectScene(const Ray& ray, RaycastHitInfo& hitInfo) {
	RaycastHitInfo closestHit;
	bool hasFoundHit = false;
	for (Shape& m : mScene->shapes) {
		Mesh* mesh = mScene->meshMap[m.geometryId];

		//Compute model matrix from the current shape's transformation
		Matrix modelMatrix = ComputeModelMatrix(m.transforms);

		//Handle different types of mesh, polygon uses triangles where sphere is implicit
		if (mesh->type == Mesh::RT_POLYGON) {
			for (Triangle& tri : mesh->triangles) {
				RaycastHitInfo tempInfo;
				if (IntersectTriangle(ray, tri, tempInfo, modelMatrix)) {
					if (!hasFoundHit) {
						hasFoundHit = true;
						closestHit = tempInfo;
						closestHit.triangle = &tri;
						closestHit.material = &m.material;
					}
					else {
						if (tempInfo.distance < closestHit.distance) {
							closestHit = tempInfo;
							closestHit.triangle = &tri;
							closestHit.material = &m.material;
						}
					}
				}
			}
		}
		else if (mesh->type == Mesh::RT_SPHERE) {
			RaycastHitInfo tempInfo;
			if (IntersectSphere(ray, mesh->sphere, tempInfo, modelMatrix)) {
				if (!hasFoundHit) {
					hasFoundHit = true;
					closestHit = tempInfo;
					closestHit.sphere = &mesh->sphere;
					closestHit.material = &m.material;
				}
				else {
					if (tempInfo.distance < closestHit.distance) {
						closestHit = tempInfo;
						closestHit.sphere = &mesh->sphere;
						closestHit.material = &m.material;
					}
				}
			}
		}
	}

	if (!hasFoundHit) return false;
	hitInfo = closestHit;
	return true;
}

Raytracer::Matrix Raytracer::ComputeModelMatrix(const Raytracer::Transformation& transform) {
	// Create scale matrix
	Raytracer::Matrix scaleMatrix;
	LoadIdentityMatrix(scaleMatrix);
	scaleMatrix[0][0] = transform.scale.x;
	scaleMatrix[1][1] = transform.scale.y;
	scaleMatrix[2][2] = transform.scale.z;
	scaleMatrix[3][3] = 1.0f;

	// Create rotation matrices
	float radX = ToRadian(transform.rotation.x);
	float radY = ToRadian(transform.rotation.y);
	float radZ = ToRadian(transform.rotation.z);

	Raytracer::Matrix rotationXMatrix;
	LoadIdentityMatrix(rotationXMatrix);
	rotationXMatrix[1][1] = cos(radX);
	rotationXMatrix[1][2] = -sin(radX);
	rotationXMatrix[2][1] = sin(radX);
	rotationXMatrix[2][2] = cos(radX);
	rotationXMatrix[0][0] = 1.0f;
	rotationXMatrix[3][3] = 1.0f;

	Raytracer::Matrix rotationYMatrix;
	LoadIdentityMatrix(rotationYMatrix);
	rotationYMatrix[0][0] = cos(radY);
	rotationYMatrix[0][2] = sin(radY);
	rotationYMatrix[2][0] = -sin(radY);
	rotationYMatrix[2][2] = cos(radY);
	rotationYMatrix[1][1] = 1.0f;
	rotationYMatrix[3][3] = 1.0f;

	Raytracer::Matrix rotationZMatrix;
	LoadIdentityMatrix(rotationZMatrix);
	rotationZMatrix[0][0] = cos(radZ);
	rotationZMatrix[0][1] = -sin(radZ);
	rotationZMatrix[1][0] = sin(radZ);
	rotationZMatrix[1][1] = cos(radZ);
	rotationZMatrix[2][2] = 1.0f;
	rotationZMatrix[3][3] = 1.0f;

	// Combine rotation matrices
	Raytracer::Matrix rotationMatrix = rotationZMatrix * rotationYMatrix * rotationXMatrix;

	// Create translation matrix
	Raytracer::Matrix translationMatrix;
	LoadIdentityMatrix(translationMatrix);
	translationMatrix[0][3] = transform.translation.x;
	translationMatrix[1][3] = transform.translation.y;
	translationMatrix[2][3] = transform.translation.z;
	translationMatrix[0][0] = 1.0f;
	translationMatrix[1][1] = 1.0f;
	translationMatrix[2][2] = 1.0f;
	translationMatrix[3][3] = 1.0f;

	// Combine all transformations: Translation * Rotation * Scale
	Raytracer::Matrix modelMatrix = scaleMatrix * rotationMatrix * translationMatrix;
	return modelMatrix;
}


int Raytracer::LoadMesh(const std::string meshName) {
	auto it = mScene->meshMap.find(meshName);
	if (it != mScene->meshMap.end()) {
		std::cout << "Mesh map already contains " << meshName << ". Skipped loading" << std::endl;
		return RT_SUCCESS;
	}

	std::ifstream file(ASSETS_PATH + meshName + ".json");
	if (!file.is_open()) {
		std::cout << "File with name " << ASSETS_PATH << meshName << ".json" << " could not be found";
		return RT_FAILURE;
	}

	nlohmann::json jsonData;
	file >> jsonData;

	Mesh* mesh = new Mesh();
	std::string shapeType = jsonData["data"][0]["type"];
	for (const auto& item : jsonData["data"]) {
		if (shapeType == "polygon") {
			mesh->type = Mesh::RT_POLYGON;
			Triangle triangle;
			// Parse vertices
			for (int i = 0; i < 3; ++i) {
				std::string vertexKey = "v" + std::to_string(i);
				Vector3 pos = { item[vertexKey]["v"][0], item[vertexKey]["v"][1], item[vertexKey]["v"][2] };
				Vector3 norm = { item[vertexKey]["n"][0], item[vertexKey]["n"][1], item[vertexKey]["n"][2] };
				Vector2 tex = { item[vertexKey]["t"][0], item[vertexKey]["t"][1] };

				Vertex vertex;
				vertex.vertexPos = pos;
				vertex.vertexNormal = norm;
				vertex.texture = tex;

				switch (i) {
				case 0: triangle.v0 = vertex; break;
				case 1: triangle.v1 = vertex; break;
				case 2: triangle.v2 = vertex; break;
				}
			}
			mesh->triangles.push_back(triangle);
		}
		else if (shapeType == "sphere") {
			mesh->type = Mesh::RT_SPHERE;
			Sphere sphere;

			float radius = item["radius"];
			sphere.radius = radius;
			mesh->sphere = sphere;
		}
	}

	mScene->meshMap[meshName] = mesh;
	return RT_SUCCESS;
}

int Raytracer::LoadSceneJSON(const std::string scenePath) {
	using json = nlohmann::json;
	int status = 0;
	std::ifstream file(ASSETS_PATH + scenePath); // Assuming the JSON is stored in a file named "scene.json"

	if (!file.is_open()) {
		std::cerr << "Failed to open JSON file" << " Path: " << ASSETS_PATH << scenePath << "\n";
		return RT_FAILURE;
	}

	// Parse the JSON
	json jsonData;
	try {
		file >> jsonData;
	}
	catch (const std::exception& e) {
		std::cout << "Error parsing JSON" << "\n";
		return RT_FAILURE;
	}

	try {
		mScene = new Scene();
		// Parse shapes
		if (jsonData["scene"].find("shapes") != jsonData["scene"].end()) {
			for (const auto& shapeValue : jsonData["scene"]["shapes"]) {
				Shape shape;
				shape.id = shapeValue["id"];
				shape.geometryId = shapeValue["geometry"];
				if (shapeValue.contains("notes")) {
					shape.notes = shapeValue["notes"].get<std::string>();
				}
				//Write material
				const auto& material = shapeValue["material"];
				shape.material.surfaceColor.x = material["Cs"][0];
				shape.material.surfaceColor.y = material["Cs"][1];
				shape.material.surfaceColor.z = material["Cs"][2];
				shape.material.Ka = material["Ka"];
				shape.material.Kd = material["Kd"];
				shape.material.Ks = material["Ks"];
				shape.material.Kt = material["Kt"];
				shape.material.specularExponet = material["n"];

				//Write transformations
				for (const auto& transformElement : shapeValue["transforms"]) {

					// Handle rotation "Rx", "Ry", "Rz"
					if (transformElement.contains("Rx")) {
						shape.transforms.rotation.x = transformElement["Rx"].get<float>();
					}
					if (transformElement.contains("Ry")) {
						shape.transforms.rotation.y = transformElement["Ry"].get<float>();
					}
					if (transformElement.contains("Rz")) {
						shape.transforms.rotation.z = transformElement["Rz"].get<float>();
					}

					// Check for and process scaling "S"
					if (transformElement.contains("S") && transformElement["S"].is_array()) {
						auto S = transformElement["S"];
						shape.transforms.scale.x = S[0].get<float>();
						shape.transforms.scale.y = S[1].get<float>();
						shape.transforms.scale.z = S[2].get<float>();
					}

					// Check for and process translation "T"
					if (transformElement.contains("T") && transformElement["T"].is_array()) {
						auto T = transformElement["T"];
						shape.transforms.translation.x = T[0].get<float>();
						shape.transforms.translation.y = T[1].get<float>();
						shape.transforms.translation.z = T[2].get<float>();
					}
				}
				mScene->shapes.push_back(shape);
				//Auto load into mesh map
				status |= LoadMesh(shape.geometryId);
			}
		}

		// Parse camera
		if (jsonData["scene"].find("camera") != jsonData["scene"].end()) {
			const auto& cameraValue = jsonData["scene"]["camera"];
			mScene->camera.from.x = cameraValue["from"][0];
			mScene->camera.from.y = cameraValue["from"][1];
			mScene->camera.from.z = cameraValue["from"][2];
			mScene->camera.to.x = cameraValue["to"][0];
			mScene->camera.to.y = cameraValue["to"][1];
			mScene->camera.to.z = cameraValue["to"][2];
			mScene->camera.camNear = cameraValue["bounds"][0];
			mScene->camera.camFar = cameraValue["bounds"][1];
			mScene->camera.camRight = cameraValue["bounds"][2];
			mScene->camera.camLeft = cameraValue["bounds"][3];
			mScene->camera.camTop = cameraValue["bounds"][4];
			mScene->camera.camBottom = cameraValue["bounds"][5];

			mScene->camera.xRes = cameraValue["resolution"][0];
			mScene->camera.yRes = cameraValue["resolution"][1];
		}

		//Parse lights
		if (jsonData["scene"].find("lights") != jsonData["scene"].end()) {
			for (const auto& lightValue : jsonData["scene"]["lights"]) {
				Light light;

				// Common properties
				light.color = Vector3(lightValue["color"][0], lightValue["color"][1], lightValue["color"][2]);
				light.intensity = lightValue["intensity"];

				std::string typeStr = lightValue["type"];
				if (typeStr == "directional") {
					Vector3 from = Vector3(lightValue["from"][0], lightValue["from"][1], lightValue["from"][2]);
					Vector3 to = Vector3(lightValue["to"][0], lightValue["to"][1], lightValue["to"][2]);

					light.direction = to - from;
					light.direction.normalize();
					light.lightType = Light::Directional;
					mScene->directional = light;
				}
				else if (typeStr == "ambient") {
					light.lightType = Light::Ambient;
				}
				else if (typeStr == "point") {
					light.lightType = Light::Point;
					light.position = Vector3(lightValue["position"][0], lightValue["position"][1], lightValue["position"][2]);
				}
				mScene->lights.push_back(light);
			}
		}
		std::cout << "Scene parsing completed!\n";
		return status;
	}
	catch (const std::exception& e) {
		std::cout << "Error parsing JSON " << e.what() << "\n";
		return RT_FAILURE;
	}
}

Raytracer::Raytracer(int width, int height) {
	mDisplay = new Display();
	mDisplay->xRes = width;
	mDisplay->yRes = height;
	mDisplay->frameBuffer = new Pixel[width * height];
	mDisplay->fov = 60.0f;
}


/// <summary>
/// Flush the current frame buffer to a ppm image
/// </summary>
/// <param name="outputName"></param>
/// <returns></returns>
int Raytracer::FlushFrameBufferToPPM(std::string outputName) {
	if (mDisplay == nullptr || mDisplay->frameBuffer == nullptr) {
		std::cerr << "Display or frame buffer is null." << std::endl;
		return RT_FAILURE;
	}

	std::ofstream outfile(outputName, std::ios::binary);
	if (!outfile.is_open()) {
		std::cerr << "Failed to create output file: " << outputName << std::endl;
		return RT_FAILURE;
	}

	// Write the PPM header
	outfile << "P6\n" << mDisplay->xRes << " " << mDisplay->yRes << "\n255\n";
	// Write the pixel data
	for (int y = 0; y < mDisplay->yRes; y++) {
		for (int x = 0; x < mDisplay->xRes; x++) {
			const Pixel& pixel = mDisplay->frameBuffer[y * mDisplay->xRes + x];

			// Apply gamma correction (assuming sRGB color space with gamma 2.2)
			unsigned char gammaCorrectR = static_cast<unsigned char>(std::pow(pixel.r / 255.0f, 1.0f / 2.2f) * 255.0f);
			unsigned char gammaCorrectG = static_cast<unsigned char>(std::pow(pixel.g / 255.0f, 1.0f / 2.2f) * 255.0f);
			unsigned char gammaCorrectB = static_cast<unsigned char>(std::pow(pixel.b / 255.0f, 1.0f / 2.2f) * 255.0f);

			// Write the gamma-corrected RGB values to the file
			outfile.put(gammaCorrectR);
			outfile.put(gammaCorrectG);
			outfile.put(gammaCorrectB);
		}
	}

	outfile.close();
	return RT_SUCCESS;

}

void Raytracer::GenerateRay(int x, int y, Ray& ray) {
	// Convert pixel coordinates to NDC
	double NDCX = (2.0 * x) / mDisplay->xRes - 1;
	double NDCY = 1 - (2.0 * y) / mDisplay->yRes;
	float aspectRatio = (float)mDisplay->xRes / (float)mDisplay->yRes;

	// Adjust for aspect ratio and field of view
	NDCX *= aspectRatio * tan(ToRadian(mDisplay->fov / 2));
	NDCY *= tan(ToRadian(mDisplay->fov / 2));

	// Set the ray origin to the camera's position
	ray.origin = mScene->camera.from;

	// Set initial ray direction based on NDC coordinates
	Vector3 direction(NDCX, NDCY, -1.0);  // Assume the camera looks towards -z in camera space

	// Apply the inverse of the camera's view matrix to transform the direction to world space
	Matrix inverseViewMatrix;
	if (Matrix::Inverse(mScene->camera.viewMatrix, inverseViewMatrix) == RT_SUCCESS) {
		ray.direction = inverseViewMatrix.TransformDirection(direction);
		ray.direction.normalize();  // Normalize the direction to ensure it's a unit vector
	}
	else {
		// Handle the error case where the inverse could not be computed
		std::cerr << "Failed to compute the inverse of the view matrix.\n";
	}
}


int Raytracer::CalculateViewMatrix(Camera& camera, Vector3 u, Vector3 v, Vector3 n, Vector3 r) {
	Matrix view;
	view[0][0] = u.x; view[0][1] = u.y; view[0][2] = u.z; view[0][3] = -r.dot(u);
	view[1][0] = v.x; view[1][1] = v.y; view[1][2] = v.z; view[1][3] = -r.dot(v);
	view[2][0] = n.x; view[2][1] = n.y; view[2][2] = n.z; view[2][3] = -r.dot(n);
	view[3][0] = 0; view[3][1] = 0; view[3][2] = 0; view[3][3] = 1;

	camera.viewMatrix = view;
	return RT_SUCCESS;
}

void Raytracer::LoadIdentityMatrix(Matrix& mat) {
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			mat[i][j] = (i == j) ? 1.0f : 0.0f;
		}
	}
}

int Raytracer::CalculateProjectionMatrix(Camera& camera, float camNear, float camFar, float top, float bottom, float left, float right) {
	Matrix proj;
	LoadIdentityMatrix(proj);
	proj[0][0] = (2 * camNear) / (right - left);
	proj[1][1] = (2 * camNear) / (top - bottom);
	proj[0][2] = (right + left) / (right - left);
	proj[1][2] = (top + bottom) / (top - bottom);
	proj[2][2] = -(camFar + camNear) / (camFar - camNear);
	proj[3][2] = -1;
	proj[2][3] = -2 * camFar * camNear / (camFar - camNear);

	camera.projectMatrix = proj;
	return RT_SUCCESS;
}

int Raytracer::InitializeRenderer() {
	int status = 0;
	//Initialize camera
	//Calculate camerae matrix, we need to calculate u, v, n, r
	Vector3 n = (mScene->camera.from - mScene->camera.to);
	n.normalize();
	mScene->camera.viewDirection = n;
	//Assume world up
	Vector3 worldUp = { 0, 1, 0 };

	Vector3 u = worldUp.cross(n);
	u.normalize();

	Vector3 v = n.cross(u);
	v.normalize();

	Vector3 r = mScene->camera.from;
	status |= CalculateViewMatrix(mScene->camera, u, v, n, r);
	status |= CalculateProjectionMatrix(mScene->camera, mScene->camera.camNear, mScene->camera.camFar, mScene->camera.camTop, mScene->camera.camBottom, mScene->camera.camLeft, mScene->camera.camRight);
	return status;
}
int Raytracer::Render(const std::string outputName) {
	InitializeRenderer();
	//Loop through all pixels and call Raycast and store it in frame buffer
	int progress = 1;
	int total = mDisplay->xRes * mDisplay->yRes;
	for (int y = 0; y < mDisplay->yRes; y++) {
		for (int x = 0; x < mDisplay->xRes; x++) {
			Ray ray;
			GenerateRay(x, y, ray);
			mDisplay->frameBuffer[y * mDisplay->xRes + x] = Raycast(ray);
			std::cout << "\rRendered: " << progress << "/" << total << "     ";
			std::cout.flush();
			progress++;
			//int progress = y * mDisplay->xRes + x + 1;
			//PrintProgress(progress, total);
		}
	}

	return FlushFrameBufferToPPM(outputName);
}

float Raytracer::CalcTriangleAreaSigned(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& planeNormal) {
	Vector3 AB = B - A;
	Vector3 AC = C - A;
	Vector3 crossResult = AB.cross(AC);
	return 0.5 * crossResult.dot(planeNormal);
}

int main() {
	//For recording duration stats

	//Do ray tracing
	Raytracer rt(500, 500);
	rt.LoadSceneJSON("simpleSphereSceneAO.json");
	rt.Render("output.ppm");

	return 0;
}

void Raytracer::PrintProgress(int current, int total) {
	int progress = current * 100 / total;
	std::cout << "\r["; // Carriage return to overwrite the line
	int barWidth = 20; // Width of the progress bar in characters
	int pos = barWidth * progress / 100;
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos) std::cout << "=";
		else if (i == pos) std::cout << ">";
		else std::cout << " ";
	}
	std::cout << "] " << progress << "% (" << current << "/" << total << ")";
	std::cout.flush(); // Make sure the output is displayed immediately
}