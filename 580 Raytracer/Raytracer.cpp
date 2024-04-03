#include "Raytracer.h"
#include <iostream>
#include "ExternalPlugins/json.hpp"
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

	// Calculate local color using Phong lighting or other shading model
	Pixel localColor = CalculateLocalColor(info);

	// Check if the material is reflective
	if (info.triangle->material.Ks > 0) {
		// Calculate reflection ray
		Ray reflectionRay(info.hitPoint, Vector3::reflect(ray.direction, info.normal).normalize());

		// For each hit, if the material of the hit object is reflective, 
		// you need to trace a new ray from the hit point in the reflection direction. 
		// This new ray will then be used to determine what the original ray "sees" in the reflection.
		Pixel reflectionColor = Raycast(reflectionRay, depth - 1);

		// The color returned by the reflected ray needs to be combined with the original
		// surface color to get the final color. 
		// This can be done using the material's reflection coefficient.
		localColor = MixColors(localColor, reflectionColor, info.triangle->material.Ks);
	}

	return localColor;
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


Raytracer::Pixel Raytracer::CalculateLocalColor(const RaycastHitInfo& hitInfo) {
	Pixel color = { 0, 0, 0 };

	// Ambient component
	float aoFactor = CalculateAmbientOcclusion(hitInfo.hitPoint, hitInfo.normal);

	// Scale the ambient component by the AO factor
	Vector3 ambient = mScene->ambient.color * hitInfo.triangle->material.Ka * aoFactor;

	// Diffuse and specular components
	Vector3 diffuse = { 0, 0, 0 };
	Vector3 specular = { 0, 0, 0 };
	for (const Light& light : mScene->lights) {
		Vector3 lightDir = light.direction - hitInfo.hitPoint;
		lightDir.normalize();

		// Diffuse reflection
		float NdotL = std::max(0.0f, hitInfo.normal.dot(lightDir));
		diffuse = diffuse + light.color * NdotL * hitInfo.triangle->material.Kd;

		// Specular reflection
		Vector3 viewDir = mScene->camera.from - hitInfo.hitPoint;
		viewDir.normalize();
		Vector3 reflectDir = Vector3::reflect(-lightDir, hitInfo.normal);
		float RdotV = std::max(0.0f, reflectDir.dot(viewDir));
		specular = specular + light.color * hitInfo.triangle->material.Ks * std::pow(RdotV, hitInfo.triangle->material.specularExponet);
	}

	// Combine components
	Vector3 finalColor = ambient + diffuse + specular;

	// Convert to Pixel format and clamp values
	color.r = std::min(static_cast<short>(finalColor.x * 255), static_cast<short>(255));
	color.g = std::min(static_cast<short>(finalColor.y * 255), static_cast<short>(255));
	color.b = std::min(static_cast<short>(finalColor.z * 255), static_cast<short>(255));

	return color;
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

	//For all three edges, if dot(vector from first vertex of the edge to p, edge) > 0, then it means that the point is left of the edge
	//When point is left of all three edges, the point is inside the triangle

	//However, the orientation of the triangle could mess up the calculation is specified wrong
	//For practical purpose, we can add a step to compute cross product of edge with the point vector
	//And use the directional information to know if it's left or right
	Vector3 edge0 = triangle.v1.vertexPos - triangle.v0.vertexPos;
	Vector3 edge1 = triangle.v2.vertexPos - triangle.v1.vertexPos;
	Vector3 edge2 = triangle.v0.vertexPos - triangle.v2.vertexPos;

	Vector3 c0 = pointOnPlane - triangle.v0.vertexPos;
	Vector3 c1 = pointOnPlane - triangle.v1.vertexPos;
	Vector3 c2 = pointOnPlane - triangle.v2.vertexPos;

	bool isPtInsideTriangle = Vector3::dot(planeNormal, Vector3::cross(edge0, c0)) >= 0 &&
		Vector3::dot(planeNormal, Vector3::cross(edge1, c1)) >= 0 &&
		Vector3::dot(planeNormal, Vector3::cross(edge2, c2)) >= 0;

	if (!isPtInsideTriangle) return false;

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
	scaleMatrix[0][0] = transform.scale.x;
	scaleMatrix[1][1] = transform.scale.y;
	scaleMatrix[2][2] = transform.scale.z;
	scaleMatrix[3][3] = 1.0f;

	// Create rotation matrices
	float radX = transform.rotation.x * M_PI / 180.0f;
	float radY = transform.rotation.y * M_PI / 180.0f;
	float radZ = transform.rotation.z * M_PI / 180.0f;

	Raytracer::Matrix rotationXMatrix;
	rotationXMatrix[1][1] = cos(radX);
	rotationXMatrix[1][2] = -sin(radX);
	rotationXMatrix[2][1] = sin(radX);
	rotationXMatrix[2][2] = cos(radX);
	rotationXMatrix[0][0] = 1.0f;
	rotationXMatrix[3][3] = 1.0f;

	Raytracer::Matrix rotationYMatrix;
	rotationYMatrix[0][0] = cos(radY);
	rotationYMatrix[0][2] = sin(radY);
	rotationYMatrix[2][0] = -sin(radY);
	rotationYMatrix[2][2] = cos(radY);
	rotationYMatrix[1][1] = 1.0f;
	rotationYMatrix[3][3] = 1.0f;

	Raytracer::Matrix rotationZMatrix;
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

				//Auto load into mesh map
				status |= LoadMesh(shape.geometryId);

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
			mScene->camera.near = cameraValue["bounds"][0];
			mScene->camera.far = cameraValue["bounds"][1];
			mScene->camera.right = cameraValue["bounds"][2];
			mScene->camera.left = cameraValue["bounds"][3];
			mScene->camera.top = cameraValue["bounds"][4];
			mScene->camera.bottom = cameraValue["bounds"][5];

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

					mScene->directional = light;
					mScene->directional.direction = to - from;
					mScene->directional.direction.normalize();
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
}


/// <summary>
/// Flush the current frame buffer to a ppm image
/// </summary>
/// <param name="outputName"></param>
/// <returns></returns>
int Raytracer::FlushFrameBufferToPPM(std::string outputName) {
	if (mDisplay == nullptr || mDisplay->frameBuffer == nullptr) return RT_FAILURE;

	FILE* outfile = nullptr;
	errno_t errOutfile = fopen_s(&outfile, outputName.c_str(), "wb");
	if (errOutfile != 0 || outfile == nullptr) {
		std::cout << "Failed to create output file: " << outputName << "\n";
		return RT_FAILURE;
	}
	// Write the PPM header
	fprintf(outfile, "P3\n%d %d\n%d\n", mDisplay->xRes, mDisplay->yRes, 5333);

	for (int y = 0; y < mDisplay->yRes; y++) {
		for (int x = 0; x < mDisplay->xRes; x++) {
			// Accessing the pixel at (x, y)
			Pixel pixel = mDisplay->frameBuffer[y * mDisplay->xRes + x];

			// Write the RGB values to the file
			fprintf(outfile, "%d %d %d ", pixel.r, pixel.g, pixel.b);
		}
		fprintf(outfile, "\n"); // Newline after each row of pixels
	}
	return RT_SUCCESS;
}

int main() {
	//For recording duration stats
	auto startTime = std::chrono::high_resolution_clock::now();

	//Do ray tracing
	Raytracer rt(512, 512);
	rt.LoadSceneJSON("scene.json");

	auto stopTime = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime);
	std::cout << "\Raytracer completed in " << duration.count() << " milliseconds.\n";

	return 0;
}