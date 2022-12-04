#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"

// #include "AutoMorphingModel.h"

using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); 
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) }; 

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
   auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/cheburashka.off") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };

    sphere1 = Model::Create("sphere", sphereMesh, material);
   cyl = Model::Create("cyl", cylMesh, material);
    cube = Model::Create("cube", cubeMesh, material);
   sphere1->Scale(2);
   sphere1->showWireframe = true;
    sphere1->Translate({ -3,0,0 });
    cyl->Translate({ 3,0,0 });
    cyl->Scale(10.0f);
   cyl->showWireframe = true;
    cube->showWireframe = true;
    camera->Translate(20, Axis::Z);
    root->AddChild(sphere1);
    root->AddChild(cyl);
    root->AddChild(cube);


}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
}
