#include "DataFile.h"

DataFile::DataFile(const Eigen::MatrixXi& F, const Eigen::MatrixXd& V) : F(F), V(V)//constructor for Part 2 
{
	igl::edge_flaps(F,this->E,this->EMAP,this->EF,this->EI);
    this->Num_Collapsed = 0;
	this->EQ =  Eigen::VectorXi::Zero(E.rows());
	this->C.resize(E.rows(), V.cols());
	this->Q = {};
    this->Vec_of_Q_Matrix.resize(V.rows());
    this->Q_delete.resize(E.rows());
    CalcMatrixNormsOfVert();//initalize of Vec_of_Q_Matrix for Part2
    for (int i = 0; i < E.rows(); i++) {//initalize of Q and Q_delete for Part2
        CalcCost(i);
      
   }
    this->SizeOFQ = Q.size();

    
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ************(constructor for Part 1)***************
// 
//DataFile::DataFile(const Eigen::MatrixXi& F, const Eigen::MatrixXd& V) : F(F), V(V)
//{
//    igl::edge_flaps(F, this->E, this->EMAP, this->EF, this->EI);
//    this->Num_Collapsed = 0;
//    this->EQ = Eigen::VectorXi::Zero(E.rows());
//    this->C.resize(E.rows(), V.cols());
//    this->Q = {};
//    this->Vec_of_Q_Matrix.resize(V.rows());
//    this->Q_delete.resize(E.rows());
//    Eigen::VectorXd costs(E.rows());
//    igl::parallel_for(E.rows(), [&](const int e)
//        {
//            double cost = e;
//            Eigen::RowVectorXd p(1, 3);
//            igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);//initalize of Q for Part1
//            C.row(e) = p;
//            costs(e) = cost;
//        }, 10000);
//    for (int e = 0;e < E.rows();e++)
//    {
//        Q.emplace(costs(e), e, 0);
//    }
//    this->SizeOFQ = Q.size();
//
//
//}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool DataFile::Our_simplification(int NumberOFaces) {//Part 2 simplifacition
    bool something_collapsed = false;
    NumberOFacesToDecrease = NumberOFaces;
    while (NumberOFacesToDecrease > 0) { 
        bool Pass = Our_collapse_edge();
        if (Pass) {
            something_collapsed = true;
        }
        else{
            break;
        }

    }
    std::cout << "Number Collapsed Edge :- " << Num_Collapsed << std::endl;
    NumberOFacesToDecrease = 0;
    SizeOFQ = SizeOFQ - Num_Collapsed;
    Num_Collapsed = 0;
    return something_collapsed;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ************(simplifacition for Part 1)***************

bool DataFile::simplification(int NumberOFaces) {
    bool something_collapsed = false;
    NumberOFacesToDecrease = NumberOFaces;
    while (NumberOFacesToDecrease > 0) {
       bool Pass = igl::collapse_edge(igl::shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C);
        if (Pass) {
            something_collapsed = true;
            NumberOFacesToDecrease--;
            Num_Collapsed++;
        }
        else {
            break;
        }

    }
    std::cout << "Number Collapsed Edge :- " << Num_Collapsed << std::endl;
    NumberOFacesToDecrease = 0;
    SizeOFQ = SizeOFQ - Num_Collapsed;
    Num_Collapsed = 0;
    return something_collapsed;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void DataFile::CalcMatrixNormsOfVert() {
    std::vector <std::vector<int>> VertexToFaces;
    std::vector <std::vector<int>> VertexToFacesI;
    Eigen::MatrixXd NormalsOfFaces;
    igl::vertex_triangle_adjacency(V.rows(), F, VertexToFaces,VertexToFacesI);
    igl::per_face_normals(V, F, NormalsOfFaces);
    for (int i = 0; i < V.rows(); i++) {
        Eigen::Matrix4d SumKp = Eigen::Matrix4d::Zero();
        for (int j = 0; j < VertexToFaces[i].size(); j++) {
            Eigen::Matrix4d Kp = Eigen::Matrix4d::Zero();
            Eigen::Vector3d normal_of_Face = NormalsOfFaces.row(VertexToFaces[i][j]).normalized();
            double d = 0;
            for (int c = 0; c < 3; c++) {
                d = d + normal_of_Face[c] * V.row(i)[c];
            }
            d = -1 * d;
            Eigen::Vector4d normal_of_Face_WithD = Eigen::Vector4d(normal_of_Face[0], normal_of_Face[1], normal_of_Face[2], d);
        
            for(int x = 0; x < 4;x++){
                Kp.col(x) = normal_of_Face_WithD[x] * normal_of_Face_WithD;
            }
           
            SumKp = SumKp + Kp;
        }
        Vec_of_Q_Matrix[i] = SumKp;
        
    }
}

double DataFile::CalcCost(int e) {
    Eigen::Vector4d New_Position = Eigen::Vector4d::Zero();
    double cost;
    Eigen::Matrix4d Q_Tag = Vec_of_Q_Matrix[E(e, 0)] + Vec_of_Q_Matrix[E(e, 1)];
    Q_Tag.row(3) = Eigen::Vector4d(0, 0, 0, 1);
    if (Q_Tag.determinant() != 0) {

        Eigen::Matrix4Xd Q_Tag_inverse = Q_Tag.inverse();
        New_Position = Q_Tag_inverse.col(3);
        Q_Tag = (Vec_of_Q_Matrix[E(e, 0)] + Vec_of_Q_Matrix[E(e, 1)]);
        cost = New_Position.transpose() * Q_Tag * New_Position;
    }
    else {
       
        Eigen::Vector4d V1;
        Eigen::Vector4d V2;
        V1[0] = V.row(E(e, 0))[0];
        V1[1] = V.row(E(e, 0))[1];
        V1[2] = V.row(E(e, 0))[2];
        V1[3] = 1;
        V2[0] = V.row(E(e, 1))[0];
        V2[1] = V.row(E(e, 1))[1];
        V2[2] = V.row(E(e, 1))[2];
        V2[3] = 1;
        double Option1_cost = V1.transpose() * Q_Tag * V1;
        double Option2_cost = V2.transpose() * Q_Tag * V2;     
        double Option3_cost = ((V1 + V2) / 2).transpose() * Q_Tag * ((V1 + V2) / 2);
        if (Option1_cost < Option2_cost && Option1_cost < Option3_cost) {
            New_Position = V1;
            cost = Option1_cost;
        }
        else if (Option2_cost < Option1_cost && Option2_cost < Option3_cost) {
            New_Position = V2;
            cost = Option2_cost;
        }
        else {
            New_Position = (V1 + V2) / 2;
            cost = Option3_cost;
        }
    }
    Eigen::Vector3d New_Position_in_C = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; i++) {
        New_Position_in_C[i] = New_Position[i];
    }

    C.row(e) = New_Position_in_C;
    std::tuple<double, int, int> Cost_To_Queue = std::make_tuple(cost, e, 0);
    Q.emplace(Cost_To_Queue);
    Q_delete[e] = cost;
    return cost;
}

bool DataFile::Our_collapse_edge() {
    bool collapsed = true;
    if (Q.empty() || NumberOFacesToDecrease == 0) {
        return false;
    }
    else {
        std::tuple<double, int, int> lowest_Cost = Q.top();
        Q.pop();
        int edge = lowest_Cost._Get_rest()._Myfirst._Val;
        double cost = lowest_Cost._Myfirst._Val;

        if (cost != Q_delete[edge] || Q_delete[edge] == std::numeric_limits<double>::infinity()) {
            return true;
        }
        else {
            int e1, e2, f1, f2;
            int V1 = E(edge, 0);
            int V2 = E(edge, 1);
            Eigen::VectorXd Position = C.row(edge);
            std::vector<int> NSt = igl::circulation(edge, true, EMAP, EF, EI);
            std::vector<int> NSf = igl::circulation(edge, false, EMAP, EF, EI);
            NSt.insert(NSt.end(), NSf.begin(), NSf.end());
            bool collapsed = igl::collapse_edge(edge, C.row(edge), V, F, E, EMAP, EF, EI, e1, e2, f1, f2);
            if (collapsed) {
                Num_Collapsed++;
                NumberOFacesToDecrease--;
                Vec_of_Q_Matrix[V1] = Vec_of_Q_Matrix[V1] + Vec_of_Q_Matrix[V2];
                Vec_of_Q_Matrix[V2] = Vec_of_Q_Matrix[V1];
                for (auto& n : NSt)
                {
                    if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
                        F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
                        F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
                    {
                        for (int v = 0; v < 3; v++)
                        {
                            const int ei = EMAP(v * F.rows() + n);
                            CalcCost(ei);
                            Q_delete[e1] = std::numeric_limits<double>::infinity();
                            Q_delete[e2] = std::numeric_limits<double>::infinity();
                            Q_delete[edge] = std::numeric_limits<double>::infinity();
                           



                        }
                    }

                }
                std::cout << "edge<" << edge << ">,cost= <" << cost << ">,new v position(<" << Position[0] << ">,<" << Position[1] << ">,<" << Position[2] << ">)" << std::endl;
                return collapsed;
            }
            return false;
        }
        return true;
    }
    return collapsed;
}
