# 580 Raytracer

## Overview
The **580 Raytracer** is a CPU-based ray tracing renderer implemented in C++. It supports rendering scenes with different light sources, materials, and geometric objects such as spheres and polygons. The program reads scene descriptions from JSON files and generates images using the ray tracing algorithm. The final rendered images are saved as PPM files.

## Features
- **Ray tracing algorithm** with recursive reflections and refractions.
- **Phong lighting model** for realistic shading.
- **Ambient occlusion** for improved realism.
- **Support for multiple light types** (ambient, directional, and point lights).
- **Scene loading from JSON** with object transformations (scaling, rotation, translation).
- **Support for triangle meshes and spheres** as primitives.
- **Ray-object intersection calculations** (Möller-Trumbore algorithm for triangles and analytic intersection for spheres).
- **Reflections, refractions, and Fresnel effects**.
- **Image output in PPM format**.

## Dependencies
- **C++ Standard Library**
- **CImg (for image handling)**
- **JSON for Modern C++ (nlohmann/json.hpp)**

## Getting Started

### Cloning the Repository
```sh
git clone https://github.com/Xena99/580-Raytracer.git
cd 580-Raytracer
```
By default, the program loads `simpleSphereScene.json` and renders `output.ppm`.

## Code Structure
The ray tracer is implemented in **Raytracer.h** and **Raytracer.cpp**.

### Main Components

### 1. **Raytracer Class**
This is the core class that handles rendering, ray tracing, and scene management.

### 2. **Scene Representation**
The scene consists of:
- **Shapes** (spheres, triangles)
- **Materials** (diffuse, specular, transparent properties)
- **Lights** (ambient, directional, point)
- **Camera** (position, view direction, projection)

### 3. **Rendering Pipeline**
1. **LoadSceneJSON**: Reads scene data from a JSON file.
2. **InitializeRenderer**: Computes camera matrices.
3. **Render**: Iterates through each pixel, generating rays and computing color.
4. **Raycast**: Determines color contributions from reflections, refractions, and lighting.
5. **FlushFrameBufferToPPM**: Writes the final image to a file.

### 4. **Ray-Object Intersection**
- **IntersectScene**: Loops through all objects and finds the closest intersection.
- **IntersectTriangle**: Uses the Möller-Trumbore algorithm.
- **IntersectSphere**: Uses analytical sphere intersection.

### 5. **Lighting and Shading**
- **CalculateLocalColor**: Computes shading using the Phong model.
- **CalculateAmbientOcclusion**: Estimates occlusion by casting sample rays.
- **ComputeFresnel**: Computes reflection and refraction coefficients based on material properties.

### 6. **Transformations**
- **ComputeModelMatrix**: Applies scaling, rotation, and translation to objects.
- **CalculateProjectionMatrix**: Sets up the projection matrix for rendering.

## Scene Format (JSON)
The ray tracer reads scenes from JSON files. Example:
```json
{
  "scene": {
    "shapes": [
      {
        "id": "sphere1",
        "geometry": "sphere",
        "material": {
          "Cs": [1.0, 0.0, 0.0],
          "Ka": 0.5,
          "Kd": 0.7,
          "Ks": 0.3,
          "Kt": 0.1,
          "n": 32
        },
        "transforms": [
          { "T": [0, 0, -3] },
          { "S": [1, 1, 1] }
        ]
      }
    ],
    "camera": {
      "from": [0, 0, 5],
      "to": [0, 0, 0],
      "bounds": [0.1, 100, 1, -1, 1, -1],
      "resolution": [500, 500]
    },
    "lights": [
      { "type": "ambient", "color": [1, 1, 1], "intensity": 0.2 },
      { "type": "directional", "from": [1, 1, 1], "to": [0, 0, 0], "color": [1, 1, 1], "intensity": 0.8 }
    ]
  }
}
```

## Output
The rendered image is saved as a **PPM file**. To view it, use an image viewer supporting PPM:
```sh
open output.ppm  # macOS
xdg-open output.ppm  # Linux
```


