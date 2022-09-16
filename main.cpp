#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include <Eigen/Core>

#include <sstream>
#include <iostream>
#include <cassert>

// mesh data
Eigen::MatrixXd V;
Eigen::MatrixXi F;

void SegmentPlaneIntersection(const Eigen::RowVector3d& segment_point, const Eigen::RowVector3d& segment_vector, const Eigen::RowVector3d& plane_point, const Eigen::RowVector3d& plane_normal, std::vector<Eigen::RowVector3d>& output_points) {
    
    Eigen::RowVector3d segment_dir = segment_vector.normalized();
    assert(0 != segment_dir.norm() && 0 != plane_normal.norm());
    if (0 == segment_dir.dot(plane_normal)) return; // Dir is parallel to normal

    double t = 0.0;
    t = (plane_point - segment_point).dot(plane_normal) / segment_dir.dot(plane_normal);

    Eigen::RowVector3d intersection_point = segment_point + t * segment_dir;
    if ((intersection_point - segment_point).norm() < segment_vector.norm() && 0 == (plane_point - intersection_point).dot(plane_normal)) {
        //std::cout << "intersection_point : " << intersection_point << std::endl;
        output_points.emplace_back(intersection_point);
    }

}

void TrianglePlaneIntersection(const Eigen::MatrixXd& V, const Eigen::RowVector3i& vertices_indexs, const Eigen::RowVector3d& plane_point, const Eigen::RowVector3d& plane_normal, std::vector<Eigen::RowVector3d>& output_points) {

    Eigen::RowVector3d A(V.row(vertices_indexs(0)));
    Eigen::RowVector3d B(V.row(vertices_indexs(1)));
    Eigen::RowVector3d C(V.row(vertices_indexs(2)));
    SegmentPlaneIntersection(A, B - A, plane_point, plane_normal, output_points);
    SegmentPlaneIntersection(B, C - B, plane_point, plane_normal, output_points);
    SegmentPlaneIntersection(C, A - C, plane_point, plane_normal, output_points);
}


void MeshPlaneIntersection(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::RowVector3d& plane_point, const Eigen::RowVector3d& plane_normal, std::vector<Eigen::RowVector3d>& output_points) {
    for (int triangle_index = 0; triangle_index < F.rows(); ++triangle_index) {
        TrianglePlaneIntersection(V, F.row(triangle_index), plane_point, plane_normal, output_points);
    }
}

int main(int argc, char *argv[])
{
    // Load a mesh in OFF format
    igl::readOFF("./mouseBrain.off", V, F);
    // Find the bounding box
    Eigen::Vector3d m = V.colwise().minCoeff();
    Eigen::Vector3d M = V.colwise().maxCoeff();
    std::cout << "min : " << "(" << m(0) << ", " << m(1) << ", " << m(2) << ")" << std::endl;
    std::cout << "max : " << "(" << M(0) << ", " << M(1) << ", " << M(2) << ")" << std::endl;

    // Corners of the bounding box
    Eigen::MatrixXd V_box(8, 3);
    V_box <<
        m(0), m(1), m(2),
        M(0), m(1), m(2),
        M(0), M(1), m(2),
        m(0), M(1), m(2),
        m(0), m(1), M(2),
        M(0), m(1), M(2),
        M(0), M(1), M(2),
        m(0), M(1), M(2);

    // Edges of the bounding box
    Eigen::MatrixXi E_box(12, 2);
    E_box << 0,1, 1,2, 2,3, 3,0, 4,5, 5,6, 6,7, 7,4, 0,4, 1,5, 2,6, 7,3;

    // customize a plane
    Eigen::RowVector3d plane_point((m + M) / 2);
    Eigen::RowVector3d plane_normal(0, 1, 0);
    std::cout << "plane_point : " << plane_point << std::endl;

    std::vector<Eigen::RowVector3d> output_points;
    MeshPlaneIntersection(V, F, plane_point, plane_normal, output_points);
    std::cout << output_points.size() << std::endl;

    Eigen::MatrixXd intersections(output_points.size(), 3);
    for (size_t i = 0; i < output_points.size(); ++i) {
        intersections.row(i) = output_points[i];
    }
    std::cout << "intersections MatrixXd size : (" << intersections.rows() << "," << intersections.cols() << ")" << std::endl;


    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);

    // Plot the corners of the bounding box as points
    //viewer.data().add_points(V_box, Eigen::RowVector3d(0, 1, 1));
    viewer.data().add_points(intersections, Eigen::RowVector3d(0, 1, 1));

    //// Plot the edges of the bounding box
    //for (unsigned i = 0; i < E_box.rows(); ++i)
    //    viewer.data().add_edges(V_box.row(E_box(i, 0)), V_box.row(E_box(i, 1)), Eigen::RowVector3d(0, 1, 1));

    // Launch the viewer
    viewer.launch();
}