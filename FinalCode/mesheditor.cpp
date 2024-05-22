#include "mesheditor.hpp"
#include <vector>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/tangential_relaxation.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/structure_point_set.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/facets_in_complex_2_to_triangle_mesh.h>
#include <CGAL/mesh_segmentation.h>

typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;

// Hole filling
typedef boost::graph_traits<Mesh>::halfedge_descriptor Halfedge_descriptor;
typedef boost::graph_traits<Mesh>::vertex_descriptor Vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor Face_descriptor;

// Structuring
typedef CGAL::First_of_pair_property_map<std::pair<Point, Vector>>  Point_map;
typedef CGAL::Second_of_pair_property_map<std::pair<Point, Vector>> Normal_map;
typedef typename PointSet::Point_map Set_point_map;
typedef typename PointSet::Vector_map Set_normal_map;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits
  <Kernel, std::vector<std::pair<Point, Vector>>, Point_map, Normal_map>              Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits>    Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits>               Plane;

// Reconstruction
typedef CGAL::Poisson_reconstruction_function<Kernel> Reconstruction;
typedef CGAL::Surface_mesh_default_triangulation_3 Triangulation;
typedef CGAL::Implicit_surface_3<Kernel, Reconstruction> Surface;
typedef Kernel::Sphere_3 Sphere;
typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<Triangulation> C2t3;


void MeshEditor::repairMesh() {
   PMP::remove_degenerate_faces(editedMesh);
   PMP::remove_isolated_vertices(editedMesh);
   // Automatically fills holes
//    fillHoles();
}

void MeshEditor::analyzeMesh() {
    std::cout << "Original Mesh - Vertices: " << num_vertices(originalMesh) << ", Edges: " << num_edges(originalMesh) << std::endl;
    std::cout << "Edited Mesh - Vertices: " << num_vertices(editedMesh) << ", Edges: " << num_edges(editedMesh) << std::endl;
}

void MeshEditor::smoothMesh() {
   PMP::tangential_relaxation(
       editedMesh,
       CGAL::parameters::number_of_iterations(10)
   );
}

void MeshEditor::removeSmallComponents(std::size_t size_threshold) {
    PMP::keep_large_connected_components(
        editedMesh,
        size_threshold
    );
}

void MeshEditor::fillHoles() {
   // Get property map that marks halfedges (edges around holes)
   Mesh::Property_map<Halfedge_descriptor, bool> hiob = editedMesh.property_map<Halfedge_descriptor, bool>("h:is_on_border").first;

   // Vector storing boundary halfedges
   std::vector<Halfedge_descriptor> border;
   // Iterate over all halfedges
   BOOST_FOREACH(Halfedge_descriptor h, halfedges(editedMesh)) {
       // Check if halfedge is marked as on the border, add it to the border vector.
       if (hiob[h])
           border.push_back(h);
   }

   // Iterate over the collected border halfedges.
   BOOST_FOREACH(Halfedge_descriptor h, border) {
       // Check again to make sure the halfedge is still on the border.
       if (hiob[h]) {
           // Call triangulate_hole to fill the hole and get the face output iterator
           auto face_output_it = CGAL::Polygon_mesh_processing::triangulate_hole(
               editedMesh, 
               h, 
               CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, editedMesh))
           );
       }
   }
}

void MeshEditor::consolidateVertices() {
    // Convert Surface_mesh to polygon soup
    std::vector<Point> points;
    std::vector<std::vector<std::size_t>> polygons;
    PMP::polygon_mesh_to_polygon_soup(editedMesh, points, polygons);

    // Merge duplicate polygons and points in the polygon soup
    PMP::merge_duplicate_polygons_in_polygon_soup(points, polygons);
    PMP::merge_duplicate_points_in_polygon_soup(points, polygons);

    // Remove isolated points in the polygon soup
    PMP::remove_isolated_points_in_polygon_soup(points, polygons);

    // Repair the polygon soup
    PMP::repair_polygon_soup(points, polygons);

    // Step 5: Clear the editedMesh to prepare for re-conversion
    editedMesh.clear();

    // Step 6: Convert the cleaned polygon soup back into Surface_mesh
    PMP::polygon_soup_to_polygon_mesh(points, polygons, editedMesh);

    if (editedMesh.is_empty()) {
        std::cerr << "Failed to convert polygon soup back to a surface mesh." << std::endl;
        reset();
        // Handle conversion failure
    }
}

void MeshEditor::setOriginalMesh(const Mesh& mesh) {
    originalMesh = mesh;
    editedMesh = mesh;  // Reset editedMesh
}

void MeshEditor::reset()
{
    editedMesh = originalMesh;
}

void MeshEditor::reconstruction()
{
   PointCloud pc;
   pc.surfaceMeshToPointCloud(editedMesh);

   // Process
   pc.simplifyPointCloud(0.05);
//    std::cout << pc.points.size() << std::endl;
   pc.removeOutliers(6);
//    std::cout << pc.points.size() << std::endl;
   pc.structurePointCloud();
//    std::cout << pc.points.size() << std::endl;

   // Reconstruct
   editedMesh.clear();
   Mesh newMesh = pc.reconstructSurfaceMesh(editedMesh);
   if (!newMesh.is_empty()) {
       std::cerr << "Surface mesh reconstruction failed." << std::endl;
       reset();
   }
   setEditedMesh(newMesh);

}

// std::vector<Mesh> MeshEditor::segmentMesh()
// {
//     // Create a property map for the SDF values
//     auto sdf_map = editedMesh.add_property_map<Mesh::face_index, double>("f:sdf", 0.0).first;

//     // Compute the SDF values using CGAL
//     CGAL::sdf_values(editedMesh, sdf_map);

//     // Create a property map for segment IDs
//     auto segment_map = editedMesh.add_property_map<Mesh::face_index, std::size_t>("f:segment_id", 0).first;

//     // Perform segmentation
//     std::size_t num_segments = CGAL::segmentation_from_sdf_values(editedMesh, sdf_map, segment_map);

//     std::cout << "Mesh segmented into " << num_segments << " segments." << std::endl;
// }

std::vector<Mesh> MeshEditor::segmentMesh(Mesh& mesh) {

    // Create a property map for the SDF values
    auto sdf_map = mesh.add_property_map<Mesh::face_index, double>("f:sdf", 0.0).first;
    CGAL::sdf_values(mesh, sdf_map);

    // Create a property map for segment IDs
    auto segment_map = mesh.add_property_map<Mesh::face_index, std::size_t>("f:segment_id", 0).first;

    // Perform segmentation
    std::size_t num_segments = CGAL::segmentation_from_sdf_values(mesh, sdf_map, segment_map);
    //std::cout << "Mesh segmented into " << num_segments << " segments." << std::endl;

    // Prepare a vector of Mesh, one for each component
    std::vector<Mesh> segment_meshes(num_segments, Mesh());

    // Maps to store vertex indices of new meshes
    std::vector<std::map<Mesh::Vertex_index, Mesh::Vertex_index>> vertex_maps(num_segments);

    // Map each face to its corresponding segment mesh
    for (auto f : faces(mesh)) {
        std::size_t segment_id = segment_map[f];
        Mesh& segment = segment_meshes[segment_id];
        auto h = halfedge(f, mesh);

        std::vector<Mesh::Vertex_index> face_vertex_indices;

        // Iterate through the vertices of the face
        CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
        for (boost::tie(vbegin, vend) = vertices_around_face(h, mesh); vbegin != vend; ++vbegin) {
            // Use a map to avoid adding the same vertex multiple times
            auto insert_result = vertex_maps[segment_id].emplace(*vbegin, Mesh::null_vertex());
            if (insert_result.second) {  // Vertex was not already present
                insert_result.first->second = segment_meshes[segment_id].add_vertex(mesh.point(*vbegin));
            }
            face_vertex_indices.push_back(insert_result.first->second);
        }

        // Add the face to the correct component mesh
        segment_meshes[segment_id].add_face(face_vertex_indices);
    }

    return segment_meshes;
}

std::vector<Mesh> MeshEditor::segmentation(Mesh& originalMesh) 
{
    // Process the mesh to find connected components
    std::vector<std::size_t> component_ids(num_faces(originalMesh), -1); // Initialize with -1 (invalid)
    auto component_map = boost::make_iterator_property_map(component_ids.begin(), get(CGAL::face_index, originalMesh));
    std::size_t num_components = CGAL::Polygon_mesh_processing::connected_components(
        originalMesh,
        component_map,
        CGAL::Polygon_mesh_processing::parameters::all_default());

    // Prepare a vector of Mesh, one for each component
    std::vector<Mesh> componentMeshes(num_components, Mesh());

    // Maps to store vertex indices of new meshes
    std::vector<std::map<Mesh::Vertex_index, Mesh::Vertex_index>> vertex_maps(num_components);

    // Populate each mesh with faces from each component
    for (const auto& f : faces(originalMesh)) {
        std::size_t component_id = component_map[f];
        std::vector<Mesh::Vertex_index> face_vertex_indices;

        // Iterate through the vertices of the face
        CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
        for (boost::tie(vbegin, vend) = vertices_around_face(halfedge(f, originalMesh), originalMesh); vbegin != vend; ++vbegin) {
            // Use a map to avoid adding the same vertex multiple times
            auto insert_result = vertex_maps[component_id].emplace(*vbegin, Mesh::null_vertex());
            if (insert_result.second) {  // Vertex was not already present
                insert_result.first->second = componentMeshes[component_id].add_vertex(originalMesh.point(*vbegin));
            }
            face_vertex_indices.push_back(insert_result.first->second);
        }

        // Add the face to the correct component mesh
        componentMeshes[component_id].add_face(face_vertex_indices);
    }

    return componentMeshes;
}


void MeshEditor::setEditedMesh(const Mesh &mesh)
{
    editedMesh = mesh;
}



////////////////////////////////////////////////

void MeshEditor::PointCloud::estimateNormals(size_t k = 12)
{   

    // Creates a property map for vertex points
    auto points_property_map = points.point_map();
    // Creates a property map for normals 
    auto normals_property_map = points.normal_map();

    // Estimate the normals
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(points, k,
                                CGAL::parameters::point_map(points_property_map).normal_map(normals_property_map));


    // Orient normals consistently
    CGAL::mst_orient_normals(points, k,
                                CGAL::parameters::point_map(points_property_map).normal_map(normals_property_map));
}

void MeshEditor::PointCloud::simplifyPointCloud(double epsilon = 0.015)
{
    // This creates a property map for vertex points 
    auto points_property_map = points.point_map();

    // Get iterator for points to be removed
    auto iterator_to_first_to_remove = CGAL::grid_simplify_point_set(
        points, epsilon, CGAL::parameters::point_map(points_property_map)
    );

    // Erase points
    points.remove(iterator_to_first_to_remove, points.end());
    // Remove from memory
    points.collect_garbage();

}

void MeshEditor::PointCloud::smoothPointCloud(double threshold = 0.3, size_t k = 12)
{
   double error;
   int iter = 10;
   // Creates a property map for vertex points
   auto points_property_map = points.point_map();
   // Creates a property map for normals 
   auto normals_property_map = points.normal_map();

   for(int i = 0; i < iter; i++)
   {
       error = CGAL::bilateral_smooth_point_set<CGAL::Sequential_tag>(
           points, k, 
           CGAL::parameters::point_map(points_property_map).normal_map(normals_property_map)
       );

       if(error <= threshold)
           break;
   }
}

void MeshEditor::PointCloud::removeOutliers(size_t k = 12)
{
   auto iterator_to_first_to_remove = CGAL::remove_outliers<CGAL::Sequential_tag>
   (points, k,
    CGAL::parameters::threshold_percent (5.). 
    threshold_distance (2. * average_spacing)); // Point with distance above 2*average_spacing are considered outliers

   // Erase points
   points.remove(iterator_to_first_to_remove, points.end());
//    std::cout<< "Removing Values" << std::endl;
   // Remove from memory
   points.collect_garbage();
//    std::cout<< "Collecting Garbage" << std::endl;
}

void MeshEditor::PointCloud::structurePointCloud()
{
    estimateNormals();

    // Backup original points and normals
    std::vector<std::pair<Point, Vector>> originalPoints;
    originalPoints.reserve(points.size());

    for (auto it = points.begin(); it != points.end(); ++it) {
        Point p = get(points.point_map(), *it);
        Vector n = get(points.normal_map(), *it);
        originalPoints.emplace_back(p, n);
    }

    // Shape detection
    Efficient_ransac ransac;
    ransac.set_input(originalPoints);
    ransac.add_shape_factory<Plane>();
    ransac.detect();

    Efficient_ransac::Plane_range planes = ransac.planes();

    std::vector<std::pair<Point,Vector>> newPoints;

    CGAL::structure_point_set(originalPoints,
                            planes,
                            std::back_inserter(newPoints),
                            0.015, // epsilon for structuring points
                            CGAL::parameters::point_map(Point_map())
                                             .normal_map(Normal_map())
                                             .plane_map(CGAL::Shape_detection::Plane_map<Traits>())
                                             .plane_index_map(CGAL::Shape_detection::Point_to_shape_index_map<Traits>(points, planes)));
    // std::cout << "Done with structure point set" << std::endl;
    points.clear();
    points.add_normal_map();
    points.reserve(newPoints.size());
    // std::cout << newPoints.size() << std::endl;

    // After populating newPoints
    for (const auto& p : newPoints) {
        auto idx = points.insert(p.first);  // Insert the point, receive a handle
        points.normal_map()[*idx] = p.second; // Use the handle to set the normal
    }   

}

void MeshEditor::PointCloud::computeAverageSpacing(size_t k = 12)
{
    // This creates a property map for vertex points 
    auto points_property_map = points.point_map();

    // Computes average spacing
    average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, k,
                                CGAL::parameters::point_map(points_property_map));
}

Mesh MeshEditor::PointCloud::reconstructSurfaceMesh(Mesh& mesh)
{   
    // Compute the average spacing
    computeAverageSpacing();
    std::cout << "Computed average spacing: " << average_spacing<< std::endl;
    // Compute the normals
    estimateNormals();
    std::cout << "Computed normals" << std::endl;
    // Smooth based on final normals
    // smoothPointCloud();

    // Backup original points and normals
    std::vector<std::pair<Point, Vector>> originalPoints;
    originalPoints.reserve(points.size());

    for (auto it = points.begin(); it != points.end(); ++it) {
        Point p = get(points.point_map(), *it);
        Vector n = get(points.normal_map(), *it);
        originalPoints.emplace_back(p, n);
    }
    
    // Create Poisson reconstruction function using the estimated normals
    Reconstruction reconstruction(originalPoints.begin(), originalPoints.end(), Point_map(), Normal_map());
    std::cout << "Initialized reconstruction" << std::endl;
    // Compute implicit function of surface
    if (!reconstruction.compute_implicit_function())
        std::cout << "Failed computing the implicit function" << std::endl;

    // Define the surface - a 3D implicit surface
    Point inner_point = reconstruction.get_inner_point();
    Sphere bsphere = reconstruction.bounding_sphere();
    // double radius = std::sqrt(bsphere.squared_radius());
    // double surface_radius = 5.0 * radius;  // Choose a larger sphere radius
    // Surface_3 surface(reconstruction, Sphere(inner_point, surface_radius), 1e-6);
    Triangulation tr; // 3D-Delaunay triangulation
    C2t3 c2t3(tr);
    Surface surface(reconstruction, bsphere, 1e-6);

    // Define the meshing criteria
    CGAL::Surface_mesh_default_criteria_3<Triangulation> criteria(30.0, 
                                                                  average_spacing*2, 
                                                                  average_spacing*0.1);

    // Generate the surface mesh
    CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_with_boundary_tag());

    Mesh output_mesh;
    CGAL::facets_in_complex_2_to_triangle_mesh(c2t3, output_mesh);

    mesh.clear();
    mesh = output_mesh;

    return mesh;
}

// void MeshEditor::PointCloud::surfaceMeshToPointCloud(const Mesh &mesh)
// {
//     // Clear any existing points in the PointSet
//     points.clear();

//     // Get the property map for vertex points in the surface mesh
//     auto vpm = get(CGAL::vertex_point, mesh);

//     // Iterate over all vertices in the mesh and insert their points into the PointSet
//     for (auto v : vertices(mesh)) {
//         // Extract the point from the vertex descriptor
//         const Point& p = get(vpm, v);
//         // Insert the point into the PointSet
//         points.insert(p);
//     }

//     points.add_normal_map();
//     computeAverageSpacing();
// }

void MeshEditor::PointCloud::surfaceMeshToPointCloud(const Mesh &mesh)
{
    // Clear any existing points in the PointSet
    points.clear();

    // Get the property map for vertex points and normals in the surface mesh
    auto vpm = get(CGAL::vertex_point, mesh);
    auto npm = points.add_normal_map();

    // If you need to calculate normals (not given by mesh)
    std::map<Mesh::Vertex_index, Vector> vertex_normals;

    // Iterate through faces to compute normals (if they are not already stored)
    for (auto f : faces(mesh)) {
        CGAL::Halfedge_around_face_circulator<Mesh> fcirc(mesh.halfedge(f), mesh), done(fcirc);
        Point p0 = get(vpm, source(*fcirc, mesh));
        ++fcirc;
        Point p1 = get(vpm, source(*fcirc, mesh));
        ++fcirc;
        Point p2 = get(vpm, source(*fcirc, mesh));

        // Compute the normal of the face
        Vector fnormal = CGAL::cross_product(p2 - p1, p1 - p0);
        fnormal = fnormal / std::sqrt(fnormal * fnormal); // Normalize the vector

        // Assign face normal to each vertex
        fcirc = done;
        do {
            vertex_normals[source(*fcirc, mesh)] += fnormal; // Sum up adjacent face normals
        } while (++fcirc != done);
    }

    // Normalize the vertex normals
    for (auto& v_n : vertex_normals) {
        Vector n = v_n.second;
        n = n / std::sqrt(n * n); // Normalize the vector
        v_n.second = n;
    }

    // Iterate over all vertices in the mesh and insert their points and normals into the PointSet
    for (auto v : vertices(mesh)) {
        const Point& p = get(vpm, v);
        const Vector& n = vertex_normals[v];
        // Insert the point and normal into the PointSet
        points.insert(p, n);
    }

    computeAverageSpacing();
}
