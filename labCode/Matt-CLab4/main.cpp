#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Cylinder.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

int TASK_N=5;  // 1, 2, 3, 4, 5

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    if (argc>=2)
        TASK_N=(int)atoi(argv[1]);
    // change the resolution for quick debugging if rendering is slow
    // Scene scene(64, 64);
    Scene scene(256, 256); // use this resolution for final rendering
    // Scene scene(512, 512);
    // Scene scene(1024, 1024);

    if(TASK_N>=4)
        scene.spp = 32; // number of samples per pixel
    else
        scene.spp = 1;

    // Materials
    Material* pink = new Material(DIFFUSE, Vector3f(0.72f, 0.48f, 0.56f));
    Material* blue = new Material(DIFFUSE, Vector3f(0.2f, 0.6f, 0.86f));
    Material* green = new Material(DIFFUSE, Vector3f(0.5f, 0.7f, 0.13f));
    Material* red = new Material(DIFFUSE, Vector3f(1.0f, 0.0f, 0.0f));
    Material* purple = new Material(DIFFUSE, Vector3f(0.21f, 0.04f, 0.32f));
    Material* yellow = new Material(DIFFUSE, Vector3f(0.70f, 0.35f, 0.0f));
    Material* grey = new Material(DIFFUSE, Vector3f(0.48f, 0.45f, 0.4f));
    Material* glass = new Material(GLASS, Vector3f(0));
    Material* light = new Material(EMIT, Vector3f(1));
    light->m_emission=100;

    // Cornell box
    MeshTriangle back_and_roof("../models/cornellbox/floor.obj", Vector3f(0), grey);
    MeshTriangle left("../models/cornellbox/left.obj", Vector3f(0), pink);
    MeshTriangle right("../models/cornellbox/right.obj",Vector3f(0),  blue);
    MeshTriangle light_("../models/cornellbox/light.obj",Vector3f(0,-5,0), light);

    scene.Add(&back_and_roof);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);

    // Floor
    Vector3f verts[4] = {{0,0,0}, {552.8,0,0}, {549.6, 0,559.2}, {0,0,559.2}};
    Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    uint32_t vertIndex[6] = {0, 2, 1, 2,0,3};
    Material* mfloor=new Material(DIFFUSE, Vector3f(0));
    mfloor->textured=true;
    scene.Add(new MeshTriangle(verts, vertIndex, 2,st,mfloor));

    // Sphere
    scene.Add(new Sphere(Vector3f(450,50,90), 50, glass));

    // Cube
    MeshTriangle cube("../models/cornellbox/shortbox.obj", Vector3f(0), green);
    scene.Add(&cube);

    // Cylinder
    scene.Add(new Cylinder(Vector3f(370, 0, 350), Vector3f(0, 1, 0), 70, 200, purple));

    // Duck
    MeshTriangle duck("../models/bob-the-duck/bob.obj", Vector3f(0), red);
    scene.Add(&duck);

    // Bunny
    // MeshTriangle bunny("../models/bunny/bunny.obj", Vector3f(0), glass);
    // scene.Add(&bunny);

    // Point light
    scene.Add(std::make_unique<PointLight>(Vector3f(-2000, 4000, -3000), 0.5));

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}