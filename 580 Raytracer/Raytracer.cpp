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
Raytracer::Pixel Raytracer::Raycast(Ray& ray, int depth) {
	if (depth <= 0) {
		return BG_COLOR; // Reached maximum recursion depth
	}

	RaycastHitInfo info;
	if (!IntersectScene(ray, info)) {
		return BG_COLOR; // No hit, return background color
	}

	Pixel localColor = Pixel(0, 0, 0);

	//Loop through all light sources and determine if pixel is occluded from it
	//If is, no contribution added, else compute local Phong lighting and add the contribution
	for (Light& light : mScene->lights) {
		//Skip ambient light
		if (light.lightType == Light::Ambient) continue;

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
			lightDir.normalize();
		}
		//Make new ray from intersection point to light source
		Ray lightRay(info.hitPoint, light.direction);
		RaycastHitInfo lightInfo;

		//For directional light, this value is not used as directional light doesn't have position property
		float distToLight = (light.position - info.hitPoint).length();

		//If pixel is occluded to this current light source, no light contribution is added
		//If intersection is past the distance to light, then means it's behind light, thus still contribution
		if (!IntersectScene(lightRay, lightInfo) || (lightInfo.distance > distToLight && light.lightType == Light::Point)) {
			// Calculate local color using Phong lighting or other shading model
			localColor = localColor + CalculateLocalColor(info, light);
		}
	}

	//Clamp the color so far after light contributions are added
	localColor.clamp();

	//Check if the material is reflective
	const Material& material = mScene->shapes[info.triangle->shapeId].material;
	if (material.Ks > 0) {
		// Calculate reflection ray
		Vector3 reflectionRayDir = Vector3::reflect(ray.direction, info.normal);
		reflectionRayDir.normalize();
		Ray reflectionRay(info.hitPoint, reflectionRayDir);

		// For each hit, if the material of the hit object is reflective, 
		// you need to trace a new ray from the hit point in the reflection direction. 
		// This new ray will then be used to determine what the original ray "sees" in the reflection.
		Pixel reflectionColor = Raycast(reflectionRay, depth - 1);

		// The color returned by the reflected ray needs to be combined with the original
		// surface color to get the final color. 
		// This can be done using the material's reflection coefficient.
		localColor = MixColors(localColor, reflectionColor, material.Ks);
	}

	return localColor.clamp();
}

//color1 is the original surface color, color2 is the color returned by the reflected ray, 
// and weight is the material's reflection coefficient (Ks). 
Raytracer::Pixel Raytracer::MixColors(const Raytracer::Pixel& color1, const Raytracer::Pixel& color2, float weight) {
	Raytracer::Pixel result;
	result.r = static_cast<short>(std::min(255.0f, color1.r * (1 - weight) + color2.r * weight));
	result.g = static_cast<short>(std::min(255.0f, color1.g * (1 - weight) + color2.g * weight));
	result.b = static_cast<short>(std::min(255.0f, color1.b * (1 - weight) + color2.b * weight));
	return result;
}

float Raytracer::Clipf(float input, int min, int max) {
	if (input < min) return min;
	if (input > max) return max;
	return input;
}

Raytracer::Pixel Raytracer::CalculateLocalColor(const RaycastHitInfo& hitInfo, const Light& light) {
	//Retrieve triangle material
	const Material& material = mScene->shapes[hitInfo.triangle->shapeId].material;
	Vector3 triVertNormals[3] = { hitInfo.triangle->v0.vertexNormal, hitInfo.triangle->v1.vertexNormal , hitInfo.triangle->v2.vertexNormal };
	//Interpolated normal
	Vector3 _normal = InterpolateVector3(triVertNormals, hitInfo.alpha, hitInfo.beta, hitInfo.gamma, true);

	//Lighting = ambient + diffuse + specular
	Vector3 lighting;
	Vector3 _ambient = mScene->ambient.color * mScene->ambient.intensity;

	//Diffuse
	Vector3 lightVector = light.direction * -1;
	lightVector.normalize();

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

	lighting = _ambient * material.Ka + _diffuse * material.Kd + _specular * material.Ks;

	Vector3 color = material.surfaceColor * lighting;

	color.x = Clipf(color.x, 0, 1);
	color.y = Clipf(color.y, 0, 1);
	color.z = Clipf(color.z, 0, 1);

	Pixel finalColor;
	// Convert to Pixel format and clamp values
	finalColor.r = static_cast<short>(color.x * 255);
	finalColor.g = static_cast<short>(color.y * 255);
	finalColor.b = static_cast<short>(color.z * 255);

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

float Raytracer::CalculateAmbientOcclusion(const Raytracer::Vector3& hitPoint, const Raytracer::Vector3& normal) {
	int numSamples = 16; // Number of samples to check for occlusion
	float occlusionRadius = 0.5f; // Max distance to check for occlusion
	int occludedCount = 0;

	for (int i = 0; i < numSamples; ++i) {
		Raytracer::Vector3 randomDir = RandomInHemisphere(normal);
		Ray occlusionRay(hitPoint, randomDir);
		RaycastHitInfo occlusionInfo;

		if (IntersectScene(occlusionRay, occlusionInfo)) {
			if (occlusionInfo.distance < occlusionRadius) {
				occludedCount++;
			}
		}
	}

	// Return the proportion of the hemisphere that is NOT occluded
	return 1.0f - (float)occludedCount / (float)numSamples;
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
	//Image a plane where the triangle lies on, the ray will intersect with the plane if the plane is in front of the ray
	//Once intersection happens with the plane, we can then figure out if the intersection point is inside the triangle
	//Even if a plane is in front of the ray, the ray will miss it if it's parallel to the ray

	//Get plane normal, which will be the same for the triangle normal since triangle lies flat on the imaginary plane
	Vector3 _edge1 = triangle.v1.vertexPos - triangle.v0.vertexPos;
	Vector3 _edge2 = triangle.v2.vertexPos - triangle.v0.vertexPos;
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
	float D = -(planeNormal.x * triangle.v0.vertexPos.x + planeNormal.y * triangle.v0.vertexPos.y + planeNormal.z * triangle.v0.vertexPos.z);

	//Since P is a point on the plane, we can substitute it into Ax + By + Cz + D = 0, and we already know D from above
	//We can calculate t, which is the distance from ray origin to intersection point
	float t = -(Vector3::dot(planeNormal, ray.origin) + D) / Vector3::dot(planeNormal, ray.direction);

	if (t < 0) {
		//The plane is behind the ray, thus no intersection
		return false;
	}
	//P = RayOrigin + t * RayDirection -> point on plane
	Vector3 pointOnPlane = ray.origin + (ray.direction * t);

	float totalArea = CalcTriangleAreaSigned(triangle.v0.vertexPos, triangle.v1.vertexPos, triangle.v2.vertexPos, planeNormal);

	// Calculate the areas for the barycentric coordinates
	float alpha = CalcTriangleAreaSigned(pointOnPlane, triangle.v1.vertexPos, triangle.v2.vertexPos, planeNormal) / totalArea;
	float beta = CalcTriangleAreaSigned(triangle.v0.vertexPos, pointOnPlane, triangle.v2.vertexPos, planeNormal) / totalArea;
	float gamma = CalcTriangleAreaSigned(triangle.v0.vertexPos, triangle.v1.vertexPos, pointOnPlane, planeNormal) / totalArea;

	if (alpha <= EPSILON || beta <= EPSILON || gamma <= EPSILON) return false;

	//Write info only when pt is inside
	//Transform hit point to world space
	hitInfo.hitPoint = modelMatrix.TransformPoint(pointOnPlane);

	//Normal needs to use inverse transpose of the model matrix
	Matrix inverseTransposeModel;
	if (RT_FAILURE == Matrix::InverseAndTranspose(modelMatrix, inverseTransposeModel)) {
		std::cerr << "Failure during matrix inverse transpose calculation of model matrix";
		return false;
	}

	hitInfo.normal = inverseTransposeModel.TransformPoint(planeNormal);
	hitInfo.normal.normalize();
	hitInfo.distance = t;
	hitInfo.alpha = alpha;
	hitInfo.beta = beta;
	hitInfo.gamma = gamma;
	return true;
}

bool Raytracer::IntersectScene(const Ray& ray, RaycastHitInfo& hitInfo) {
	RaycastHitInfo closestHit;
	bool hasFoundHit = false;
	for (Shape& m : mScene->shapes) {
		Mesh* mesh = mScene->meshMap[m.geometryId];

		//Todo: Compute model matrix from the current shape's transformation
		Matrix modelMatrix = ComputeModelMatrix(m.transforms);

		for (Triangle& tri : mesh->triangles) {
			RaycastHitInfo tempInfo;
			if (IntersectTriangle(ray, tri, tempInfo, modelMatrix)) {
				if (!hasFoundHit) {
					hasFoundHit = true;
					closestHit = tempInfo;
					closestHit.triangle = &tri;
				}
				else {
					if (tempInfo.distance < closestHit.distance) {
						closestHit = tempInfo;
						closestHit.triangle = &tri;
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

	// Combine all transformations
	Raytracer::Matrix modelMatrix = translationMatrix * rotationMatrix * scaleMatrix;
	return modelMatrix;
}

//////Helper Functions//////
int Raytracer::LoadMesh(const std::string meshName, const int shapeId) {
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
	for (const auto& item : jsonData["data"]) {
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
		triangle.shapeId = shapeId;
		mesh->triangles.push_back(triangle);
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
				shape.notes = shapeValue["notes"];

				//Write material
				const auto& material = shapeValue["material"];
				shape.material.surfaceColor.x = material["Cs"][0];
				shape.material.surfaceColor.y = material["Cs"][1];
				shape.material.surfaceColor.z = material["Cs"][2];
				shape.material.Ka = material["Ka"];
				shape.material.Kd = material["Kd"];
				shape.material.Ks = material["Ks"];
				shape.material.specularExponet = material["n"];

				//Write transformations
				const auto& transforms = shapeValue["transforms"];
				shape.transforms.scale.x = transforms[1]["S"][0];
				shape.transforms.scale.y = transforms[1]["S"][1];
				shape.transforms.scale.z = transforms[1]["S"][2];

				//Write rotations
				if (transforms[0].find("Ry") != transforms[0].end()) {
					shape.transforms.rotation.y = transforms[0]["Ry"];
				}
				if (transforms[0].find("Rx") != transforms[0].end()) {
					shape.transforms.rotation.x = transforms[0]["Rx"];
				}
				if (transforms[0].find("Rz") != transforms[0].end()) {
					shape.transforms.rotation.z = transforms[0]["Rz"];
				}

				shape.transforms.translation.x = transforms[2]["T"][0];
				shape.transforms.translation.y = transforms[2]["T"][1];
				shape.transforms.translation.z = transforms[2]["T"][2];

				mScene->shapes.push_back(shape);
				//Auto load into mesh map
				status |= LoadMesh(shape.geometryId, mScene->shapes.size() - 1);
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
			// Access the pixel at (x, y)
			const Pixel& pixel = mDisplay->frameBuffer[y * mDisplay->xRes + x];
			// Write the RGB values to the file
			outfile.put(pixel.r);
			outfile.put(pixel.g);
			outfile.put(pixel.b);
		}
	}

	outfile.close();
	return RT_SUCCESS;

}

/// <summary>
/// Generates the ray based on x, y pixel locations
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="ray"></param>
void Raytracer::GenerateRay(int x, int y, Ray& ray) {
	//Find viewport position
	//X, Y to NDC which is [-1, 1]
	double NDCX = (2.0 * x) / mDisplay->xRes - 1;
	double NDCY = 1 - (2.0 * y) / mDisplay->yRes;
	float aspectRatio = (float)mDisplay->xRes / (float)mDisplay->yRes;
	NDCX *= aspectRatio * tan(ToRadian(mDisplay->fov / 2));
	NDCY *= tan(ToRadian(mDisplay->fov / 2));

	//Ray.origin is camera position, the from vector
	ray.origin = mScene->camera.from;

	ray.direction.x = NDCX;
	ray.direction.y = NDCY;
	ray.direction.z = -1.0f;

	//Kevin: For some reason this won't work
	//Matrix inverseViewMatrix;
	//Matrix::InverseAndTranspose(mScene->camera.viewMatrix, inverseViewMatrix);
	//ray.direction = inverseViewMatrix.TransformPoint(ray.direction);
	ray.direction.normalize();
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
			mDisplay->frameBuffer[y * mDisplay->xRes + x] = Raycast(ray, 9999999);
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
	auto startTime = std::chrono::high_resolution_clock::now();

	//Do ray tracing
	Raytracer rt(250, 250);
	rt.LoadSceneJSON("scene.json");
	rt.Render("output.ppm");
	auto stopTime = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime);
	std::cout << "\Raytracer completed in " << duration.count() << " milliseconds.\n";

	return 0;
}

void Raytracer::PrintProgress(int current, int total) {
	int progress = current * 100 / total;
	std::cout << "\r["; // Carriage return to overwrite the line
	int barWidth = 50; // Width of the progress bar in characters
	int pos = barWidth * progress / 100;
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos) std::cout << "=";
		else if (i == pos) std::cout << ">";
		else std::cout << " ";
	}
	std::cout << "] " << progress << "% (" << current << "/" << total << ")";
	std::cout.flush(); // Make sure the output is displayed immediately
}