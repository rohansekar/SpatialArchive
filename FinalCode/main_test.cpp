#include <iostream>
#include <string>
#include "mesheditor.hpp"  // Your MeshEditor definitions

// Function prototypes
bool testSimpleMesh();
bool testComplexMesh();
bool testemptyMesh();

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " [test_name]" << std::endl;
        return 1;
    }

    std::string test_name(argv[1]);
    bool result = false;

    if (test_name == "simple") {
        result = testSimpleMesh();
    } else if (test_name == "complex") {
        result = testComplexMesh();
    } else if (test_name == "degenerate") {
        result = testemptyMesh();
    } else {
        std::cerr << "Unknown test: " << test_name << std::endl;
        return 1;
    }

    if (result) {
        std::cout << "Test passed." << std::endl;
        return 0;  // Success
    } else {
        std::cerr << "Test failed." << std::endl;
        return 1;  // Failure
    }
}

bool testSimpleMesh() {
    Mesh mesh;
    MeshEditor editor;

    // Create a simple triangle mesh
    auto v1 = mesh.add_vertex(Point(0, 0, 0));
    auto v2 = mesh.add_vertex(Point(1, 0, 0));
    auto v3 = mesh.add_vertex(Point(0, 1, 0));
    mesh.add_face(v1, v2, v3);

    // Run segmentation
    auto segmentMeshes = editor.segmentation(mesh);

    // Check if the segmentation results in exactly one component
    if (segmentMeshes.size() == 1 && !segmentMeshes[0].is_empty()) {
        return true;
    } else {
        std::cerr << "Expected exactly one non-empty segment." << std::endl;
        return false;
    }
}

bool testComplexMesh() {
    Mesh mesh;
    MeshEditor editor;

    // Create a mesh with two disconnected triangles
    auto v1 = mesh.add_vertex(Point(0, 0, 0));
    auto v2 = mesh.add_vertex(Point(1, 0, 0));
    auto v3 = mesh.add_vertex(Point(0, 1, 0));
    auto v4 = mesh.add_vertex(Point(2, 0, 0));
    auto v5 = mesh.add_vertex(Point(3, 0, 0));
    auto v6 = mesh.add_vertex(Point(2, 1, 0));

    mesh.add_face(v1, v2, v3);
    mesh.add_face(v4, v5, v6);

    // Run segmentation
    auto segmentMeshes = editor.segmentation(mesh);

    // Check if the segmentation results in exactly two components
    if (segmentMeshes.size() == 2 && !segmentMeshes[0].is_empty() && !segmentMeshes[1].is_empty()) {
        return true;
    } else {
        std::cerr << "Expected exactly two non-empty segments." << std::endl;
        return false;
    }
}

bool testemptyMesh() {
    Mesh mesh;
    MeshEditor editor;

    // Attempt to segment an empty mesh
    auto segmentMeshes = editor.segmentation(mesh);

    // Check if the function correctly handles an empty mesh
    if (segmentMeshes.empty()) {
        return true;
    } else {
        std::cerr << "Expected no segments from an empty mesh." << std::endl;
        return false;
    }
}
