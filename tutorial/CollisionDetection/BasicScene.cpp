#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"


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
    auto program = std::make_shared<Program>("shaders/accumShader");
    auto material{ std::make_shared<Material>("material", program) };
    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/bunny.off") };
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/bunny.off") };
    bunny1 = Model::Create("bunny1", sphereMesh, material);
    bunny2 = Model::Create("bunny2", cylMesh, material);
    bunny1->showFaces = false;
    bunny2->showFaces = false;
    bunny1->Translate(-0.5, Axis::X);
    bunny2->Translate(0.5, Axis::X);
    camera->Translate(1.5, Axis::Z);
    camera->Translate(0.1, Axis::Y);
    root->AddChild(bunny1);
    root->AddChild(bunny2);
    bunny1tree.init(bunny1->GetMeshList()[0]->data[0].vertices, bunny1->GetMeshList()[0]->data[0].faces);
    bunny2tree.init(bunny2->GetMeshList()[0]->data[0].vertices, bunny2->GetMeshList()[0]->data[0].faces);
    cube1 = NewBox(bunny1tree.m_box,1);
    cube2 = NewBox(bunny2tree.m_box,2);
    bunny1->AddChild(cube1);
    bunny2->AddChild(cube2);
    cube1->showFaces = false;
    cube2->showFaces = false;
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    bunny1->Translate(-1 * moveLeftRight, Axis::X);
    bunny2->Rotate(0, Axis::X);
   //bunny1->Rotate(1* moveLeftRight, Axis::X);
    if (pickedModel != nullptr && pickedModel->name == "cube1") {
        pickedModel = bunny1;
    }
    if (pickedModel != nullptr && pickedModel->name == "cube2") {
        pickedModel = bunny2;
    }
    if (stat && Collision(&bunny1tree, &bunny2tree)) {
        moveLeftRight = 0;
        stat =  false;
        collisions = true;
        std::cout << "-------------------------^|^   Collision   ^|^-----------------------------" << std::endl;
    }else if(!stat){ 
        moveLeftRight = 0; 
    }
    
}


bool BasicScene::Collision(igl::AABB <Eigen::MatrixXd, 3>* tree1, igl::AABB <Eigen::MatrixXd, 3>* tree2)
{
   
    if (!Collision_Conditions(tree1->m_box, tree2->m_box)) {
        return false;
    }
    else {
        if (tree1->is_leaf() && tree2->is_leaf()) {
            cube3 = NewBox(tree1->m_box,3);
            cube4 = NewBox(tree2->m_box,4);
            bunny1->AddChild(cube3);
            bunny2->AddChild(cube4);
            return true;
        }
        igl::AABB<Eigen::MatrixXd, 3>* lefttree1 = tree1->is_leaf() ? tree1 : tree1->m_left;
        igl::AABB<Eigen::MatrixXd, 3>* righttree1 = tree1->is_leaf() ? tree1 : tree1->m_right;
        igl::AABB<Eigen::MatrixXd, 3>* lefttree2 = tree2->is_leaf() ? tree2 : tree2->m_left;
        igl::AABB<Eigen::MatrixXd, 3>* righttree2 = tree2->is_leaf() ? tree2 : tree2->m_right;

        if( Collision(lefttree1, lefttree2) || Collision(lefttree1, righttree2) ||
            Collision(righttree1, lefttree2) || Collision(righttree1, righttree2)){
            return true;
        }
        else
            return false;
    }
}

bool BasicScene::Collision_Conditions(Eigen::AlignedBox<double, 3> Frame1, Eigen::AlignedBox<double, 3> Frame2) {
    double R0, R1, R;
    Eigen::Matrix3d A = bunny1->GetRotation().cast<double>();
    Eigen::Matrix3d B = bunny2->GetRotation().cast<double>();
    Eigen::Matrix3d C = A.transpose() * B;
    Eigen::Vector3d CenterOfFrame1 = Frame1.center();
    Eigen::Vector3d CenterOfFrame2 = Frame2.center();
    Eigen::Vector4d CenterofA = Eigen::Vector4d(CenterOfFrame1[0], CenterOfFrame1[1], CenterOfFrame1[2], 1);
    Eigen::Vector4d CenterofB = Eigen::Vector4d(CenterOfFrame2[0], CenterOfFrame2[1], CenterOfFrame2[2], 1);
    //float ScaleA = bunny1->GetScaling(bunny1->GetTransform())(0,0);
  //  float ScaleB = bunny2->GetScaling(bunny2->GetTransform())(0,0);
    double a0 = Frame1.sizes()[0] / 2;
    double a1 =  Frame1.sizes()[1] / 2;
    double a2 =  Frame1.sizes()[2] / 2;
    double b0 =  Frame2.sizes()[0] / 2;
    double b1 =  Frame2.sizes()[1] / 2;
    double b2 =  Frame2.sizes()[2] / 2;
    Eigen::RowVector3d A0 = A.col(0).transpose();
    Eigen::RowVector3d A1 = A.col(1).transpose();
    Eigen::RowVector3d A2 = A.col(2).transpose();
    Eigen::RowVector3d B0 = B.col(0).transpose();
    Eigen::RowVector3d B1 = B.col(1).transpose();
    Eigen::RowVector3d B2 = B.col(2).transpose();
    Eigen::Vector4d newCenterOfB =  bunny2->GetTransform().cast<double>() * CenterofB;
    Eigen::Vector4d newCenterOfA =  bunny1->GetTransform().cast<double>() * CenterofA;
    Eigen::Vector4d Current_D = newCenterOfB - newCenterOfA;
    Eigen::Vector3d D = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; i++) {
        D[i] = Current_D[i];
    }
    
    R0 = a0;
    R1 = b0 * abs(C.row(0)[0]) + b1 * abs(C.row(0)[1]) + b2 * abs(C.row(0)[2]);
    R = abs(A0.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a1;
    R1 = b0 * abs(C.row(1)[0]) + b1 * abs(C.row(1)[1]) + b2 * abs(C.row(1)[2]);
    R = abs(A1.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a2;
    R1 = b0 * abs(C.row(2)[0]) + b1 * abs(C.row(2)[1]) + b2 * abs(C.row(2)[2]);
    R = abs(A2.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(0)[0]) + a1 * abs(C.row(1)[0]) + a2 * abs(C.row(2)[0]);
    R1 = b0;
    R = abs(B0.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(0)[1]) + a1 * abs(C.row(1)[1]) + a2 * abs(C.row(2)[1]);
    R1 = b1;
    R = abs(B1.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(0)[2]) + a1 * abs(C.row(1)[2]) + a2 * abs(C.row(2)[2]);
    R1 = b2;
    R = abs(B2.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a1 * abs(C.row(2)[0]) + a2 * abs(C.row(1)[0]);
    R1 = b1 * abs(C.row(0)[2]) + b2 * abs(C.row(0)[1]);
    R = abs((C.row(1)[0] * A2).dot(D) - (C.row(2)[0] * A1).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a1 * abs(C.row(2)[1]) + a2 * abs(C.row(1)[1]);
    R1 = b0 * abs(C.row(0)[2]) + b2 * abs(C.row(0)[0]);
    R = abs((C.row(1)[1] * A2).dot(D) - (C.row(2)[1] * A1).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a1 * abs(C.row(2)[2]) + a2 * abs(C.row(1)[2]);
    R1 = b0 * abs(C.row(0)[1]) + b1 * abs(C.row(0)[0]);
    R = abs((C.row(1)[2] * A2).dot(D) - (C.row(2)[2] * A1).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(2)[0]) + a2 * abs(C.row(0)[0]);
    R1 = b1 * abs(C.row(1)[2]) + b2 * abs(C.row(1)[1]);
    R = abs((C.row(2)[0] * A0).dot(D) - (C.row(0)[0] * A2).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(2)[1]) + a2 * abs(C.row(0)[1]);
    R1 = b0 * abs(C.row(1)[2]) + b2 * abs(C.row(0)[1]);
    R = abs((C.row(2)[1] * A0.dot(D) - (C.row(0)[1] * A2).dot(D)));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(2)[2]) + a2 * abs(C.row(0)[2]);
    R1 = b0 * abs(C.row(1)[1]) + b1 * abs(C.row(1)[0]);
    R = abs((C.row(2)[2] * A0).dot(D) - (C.row(0)[2] * A2).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(1)[0]) + a1 * abs(C.row(0)[0]);
    R1 = b1 * abs(C.row(2)[2]) + b2 * abs(C.row(2)[1]);
    R = abs((C.row(0)[0] * A1).dot(D) - (C.row(1)[0] * A0).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(1)[1]) + a1 * abs(C.row(0)[1]);
    R1 = b0 * abs(C.row(2)[2]) + b2 * abs(C.row(2)[0]);
    R = abs((C.row(0)[1] * A1).dot(D) - (C.row(1)[1] * A0).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(1)[2]) + a1 * abs(C.row(0)[2]);
    R1 = b0 * abs(C.row(2)[1]) + b1 * abs(C.row(2)[0]);
    R = abs((C.row(0)[2] * A1).dot(D) - (C.row(1)[2] * A0).dot(D));
    if (R > R0 + R1) { return false; }
   
    return true;
}

std::shared_ptr<cg3d::Model> BasicScene::NewBox(Eigen::AlignedBox<double, 3> Frame , int  index ) {
    Eigen::MatrixXd V(8, 3), VertexToNormal, C;
    Eigen::MatrixXi F(12, 3);
    Eigen::RowVector3d V0 = Frame.corner(Frame.BottomLeftFloor);
    Eigen::RowVector3d V1 = Frame.corner(Frame.TopLeftFloor);
    Eigen::RowVector3d V2 = Frame.corner(Frame.TopRightFloor);
    Eigen::RowVector3d V3 = Frame.corner(Frame.BottomRightFloor);
    Eigen::RowVector3d V4 = Frame.corner(Frame.BottomLeftCeil);
    Eigen::RowVector3d V5 = Frame.corner(Frame.TopLeftCeil);
    Eigen::RowVector3d V6 = Frame.corner(Frame.TopRightCeil);
    Eigen::RowVector3d V7 = Frame.corner(Frame.BottomRightCeil);
    V << V0[0], V0[1], V0[2], V1[0], V1[1], V1[2], V2[0], V2[1], V2[2], V3[0], V3[1],
        V3[2], V4[0], V4[1], V4[2], V5[0], V5[1], V5[2], V6[0], V6[1], V6[2], V7[0], V7[1], V7[2];
    F << 1, 2, 5, 2, 5, 6, 4, 5, 7, 5, 6, 7, 0, 3, 4, 3, 4, 7, 0, 1, 3, 1, 2, 3, 0, 1, 5, 0, 4, 5, 2, 3, 7, 2, 6, 7;
    igl::per_vertex_normals(V, F, VertexToNormal);
    C = Eigen::MatrixXd::Zero(V.rows(), 2);
    std::shared_ptr<Mesh> m = std::make_shared<Mesh>("Cube", V, F, VertexToNormal, C);
    std::vector<std::shared_ptr<Mesh>> vec;
    vec.push_back(m);
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) };
    std::shared_ptr<cg3d::Model> model;
    if(index == 1){  model = Model::Create("cube1", m, material); }
    else if(index ==2) {model = Model::Create("cube2", m, material); }
    else{ model = Model::Create("cube", m, material); }
    model->showWireframe = true;
    return model;
}

