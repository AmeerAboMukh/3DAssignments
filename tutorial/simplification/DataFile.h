#pragma once
#include <Eigen/Core>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
//#include <igl/parallel_for.h>
#include <igl/per_face_normals.h>
#include <igl/circulation.h>
#include <iostream>
#include <Eigen/LU>
#include <igl/vertex_triangle_adjacency.h>

class DataFile 
{
	public:
		DataFile() = default;
		DataFile(const Eigen::MatrixXi& F, const Eigen::MatrixXd& V);
		bool Our_simplification(int NumberFacesToDecrease);//for Part 2
		bool simplification(int NumberFacesToDecrease);// for Part 1
		void CalcMatrixNormsOfVert();
		double CalcCost(int e);
		bool Our_collapse_edge();
		int Num_Collapsed;
		int NumberOFacesToDecrease = 0;
		int SizeOFQ ;
		Eigen::VectorXi EMAP,EQ;
		Eigen::MatrixXi E, EF, EI,F;
		Eigen::MatrixXd C,V;
		igl::min_heap< std::tuple<double, int, int> > Q;
		std::vector<Eigen::MatrixXd> Vec_of_Q_Matrix; 
		std::vector<double> Q_delete;
};