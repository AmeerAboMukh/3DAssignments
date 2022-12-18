#pragma once

#include "Scene.h"
#include "AABB.h"
#include "igl/per_vertex_normals.h"
#include <utility>

class BasicScene : public cg3d::Scene
{
public:
  
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    bool Collision_Conditions(Eigen::AlignedBox<double, 3> Frame1, Eigen::AlignedBox<double, 3> Frame2);
    bool Collision(igl::AABB <Eigen::MatrixXd, 3>* tree1, igl::AABB <Eigen::MatrixXd, 3>* tree2);
    std::shared_ptr<cg3d::Model> NewBox(Eigen::AlignedBox<double, 3> Frame,int index);
    std::shared_ptr<cg3d::Model> cyl;
private:
    std::shared_ptr<Movable> root;
    igl::AABB <Eigen::MatrixXd, 3> bunny1tree, bunny2tree;
   
};

