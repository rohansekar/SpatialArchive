#ifndef MESH_EDITOR_HPP
#define MESH_EDITOR_HPP

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Polygon_mesh_processing/repair.h>

// Define the Kernel and the Mesh type
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Point_set_3<Point, Vector> PointSet;

// Define polygonal mesh processing
namespace PMP = CGAL::Polygon_mesh_processing;

class MeshEditor {
private:
    Mesh originalMesh, editedMesh;  // The CGAL mesh object

public:
    friend class PointCloud;
    // Constructor and Destructor
    MeshEditor() = default;
    explicit MeshEditor(const Mesh& mesh): originalMesh(mesh), editedMesh(mesh) {};
    ~MeshEditor() = default;

    // Mesh manipulation methods
    void repairMesh();                            // Repair and clean the mesh
    void analyzeMesh();                           // Analyze properties of the mesh
    void smoothMesh();                            // Smooth the mesh surface

    // Methods to extend functionality
    void removeSmallComponents(std::size_t size_threshold);  // Remove small isolated components
    void fillHoles();                                           // Fill holes in the mesh
    void consolidateVertices();                                 // Merge close vertices
    void reconstruction();
    std::vector<Mesh> segmentMesh(Mesh& originalMesh);
    std::vector<Mesh> segmentation(Mesh& originalMesh);

    // Getter funcions
    const Mesh& getOriginalMesh() const { return originalMesh; }
    const Mesh& getEditedMesh() const { return editedMesh; }

    // Setter functions
    void setOriginalMesh(const Mesh& mesh);
    void setEditedMesh(const Mesh&mesh);

    // Reset function
    void reset();

public:
    // Define Point Cloud class
    class PointCloud {
    public:
        PointSet points;
        double average_spacing;

        PointCloud() = default;
        void computeAverageSpacing(size_t k);
        void estimateNormals(size_t k);
        void simplifyPointCloud(double epsilon);
        void smoothPointCloud(double threshold, size_t k);
        void removeOutliers(size_t k);
        void structurePointCloud();
        Mesh reconstructSurfaceMesh(Mesh& mesh);
        void surfaceMeshToPointCloud(const Mesh& mesh);
    };



    // Add more methods here for additional functionalities
};

#endif
