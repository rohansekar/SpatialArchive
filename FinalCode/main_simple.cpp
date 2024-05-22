#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <fstream>
#include <vector>
#include <array>
#include <iostream>
#include <CGAL/IO/STL.h>
#include <boost/filesystem.hpp>

// For MongoDB Client
#include "MongoDBClient.h"
#include <string>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/array.hpp>

// For stlviewer
#include "fssimplewindow.h"
#include "ysclass.h"
#include "stlviewer.h"
#include "mesheditor.hpp"
#include <random>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;

template <typename T>
void addDataToStream(std::ostringstream &stream, const T &data)
{
    stream.write(reinterpret_cast<const char *>(&data), sizeof(data));
}

// Function to create binary STL data in memory
std::vector<char> createBinarySTL(const std::vector<GLfloat> &vertices, const std::vector<GLfloat> &normals)
{
    std::ostringstream out(std::ios::binary);

    // Write header
    char header[80] = {}; // Clear header space
    out.write(header, sizeof(header));

    // Write number of triangles
    uint32_t numTriangles = vertices.size() / 9;
    addDataToStream(out, numTriangles);

    for (size_t i = 0, n = 0; i < vertices.size(); i += 9, n += 3)
    {
        // Write the normal
        out.write(reinterpret_cast<const char *>(&normals[n]), 3 * sizeof(GLfloat));
        // Write the vertices
        out.write(reinterpret_cast<const char *>(&vertices[i]), 9 * sizeof(GLfloat));
        // Write attribute byte count
        uint16_t attributeByteCount = 0;
        addDataToStream(out, attributeByteCount);
    }

    std::string buf = out.str();
    return std::vector<char>(buf.begin(), buf.end());
}

std::vector<uint8_t> write_surface_mesh_to_stl(const Mesh &mesh)
{
    std::stringstream output(std::ios::out | std::ios::binary);

    // Extract points and faces for the write_STL function
    std::vector<Point> points;
    std::vector<std::array<std::size_t, 3>> faces;

    // Get the points directly from the mesh
    points.reserve(mesh.number_of_vertices());
    for (auto v : mesh.vertices())
    {
        points.push_back(mesh.point(v));
    }

    // Iterate over the faces of the mesh to get the indices of the vertices
    faces.reserve(mesh.number_of_faces());
    for (auto f : mesh.faces())
    {
        std::vector<Mesh::Vertex_index> vertices;
        CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
        boost::tie(vbegin, vend) = vertices_around_face(mesh.halfedge(f), mesh);
        for (auto vit = vbegin; vit != vend; ++vit)
        {
            vertices.push_back(*vit);
        }
        // Assumes that we have a triangular mesh
        if (vertices.size() == 3)
        {
            std::array<std::size_t, 3> face_indices = {
                static_cast<std::size_t>(vertices[0]),
                static_cast<std::size_t>(vertices[1]),
                static_cast<std::size_t>(vertices[2])};
            faces.push_back(face_indices);
        }
    }

    // Use the write_STL function provided by CGAL to write into the stringstream
    if (!CGAL::IO::write_STL(output, points, faces))
    {
        std::cerr << "Failed to serialize STL data." << std::endl;
        return {}; // Return an empty vector if serialization fails
    }

    // Convert stringstream to std::vector<int8_t>
    std::string str = output.str();
    return std::vector<uint8_t>(str.begin(), str.end());
}

// Update function to take an input stream instead of a filename
bool read_stl_to_surface_mesh(std::istream &input, Mesh &mesh)
{
    std::vector<Point> points;
    std::vector<std::array<std::size_t, 3>> facets;

    if (!CGAL::IO::read_STL(input, points, facets))
    {
        std::cerr << "Failed to read STL data." << std::endl;
        return false;
    }

    // Add vertices to the mesh and store the vertex descriptors
    std::vector<Mesh::Vertex_index> vertex_descriptors;
    vertex_descriptors.reserve(points.size());
    for (const Point &point : points)
    {
        vertex_descriptors.push_back(mesh.add_vertex(point));
    }

    // Now let's populate the mesh with faces
    for (const auto &facet : facets)
    {
        Mesh::Vertex_index v0 = vertex_descriptors[facet[0]];
        Mesh::Vertex_index v1 = vertex_descriptors[facet[1]];
        Mesh::Vertex_index v2 = vertex_descriptors[facet[2]];
        mesh.add_face(v0, v1, v2);
    }

    return true;
}

void prepareMeshData(const Mesh &mesh, std::vector<GLfloat> &vtx, std::vector<GLfloat> &nom, std::vector<GLfloat> &col)
{
    int vertexCount = 0; // For counting vertices

    for (auto f : mesh.faces())
    {
        CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
        std::vector<Point> vertices;

        // Collect vertices of the face
        for (boost::tie(vbegin, vend) = vertices_around_face(mesh.halfedge(f), mesh); vbegin != vend; ++vbegin)
        {
            vertices.push_back(mesh.point(*vbegin));
        }

        if (vertices.size() == 3)
        { // Ensure the face is a triangle
            // Calculate the normal using the cross product
            Kernel::Vector_3 normal = CGAL::cross_product(vertices[1] - vertices[0], vertices[2] - vertices[0]);
            normal = normal / std::sqrt(normal.squared_length()); // Normalize the vector

            // Store the normal and vertex coordinates
            for (int i = 0; i < 3; i++)
            {
                // Store normals
                nom.push_back(static_cast<GLfloat>(normal.x()));
                nom.push_back(static_cast<GLfloat>(normal.y()));
                nom.push_back(static_cast<GLfloat>(normal.z()));

                // Store vertex coordinates
                vtx.push_back(static_cast<GLfloat>(vertices[i].x()));
                vtx.push_back(static_cast<GLfloat>(vertices[i].y()));
                vtx.push_back(static_cast<GLfloat>(vertices[i].z()));
            }

            vertexCount += 3; // Increment the vertex count by three for each triangle processed
        }
    }

    for (int i = 0; i < vtx.size() / 3; ++i)
    {
        col.push_back(0);
        col.push_back(1);
        col.push_back(0);
        col.push_back(1);
    }

    // Output the number of vertices and normals for verification
    // std::cout << "Number of normals calculated: " << nom.size() / 3 << std::endl;
    // std::cout << "Number of vertices processed: " << vtx.size() / 3 << std::endl;
}

void populateColors(std::vector<GLfloat> &col, int numColors)
{
    std::random_device rd;                            // Random device for seeding
    std::mt19937 eng(rd());                           // Mersenne Twister random generator
    std::uniform_real_distribution<> distr(0.0, 1.0); // Distribution from 0 to 1

    col.reserve(numColors * 3); // Reserve space for efficiency

    float red = static_cast<GLfloat>(distr(eng));
    float green = static_cast<GLfloat>(distr(eng));
    float blue = static_cast<GLfloat>(distr(eng));

    for (int i = 0; i < numColors; ++i)
    {
        // Generate a new set of RGB values for each iteration

        col.push_back(red);   // Red component
        col.push_back(green); // Green component
        col.push_back(blue);  // Blue component
        col.push_back(1.0f);
    }
}

void segmentationMeshData(const Mesh &mesh, std::vector<GLfloat> &vtx, std::vector<GLfloat> &nom, std::vector<GLfloat> &col)
{
    int vertexCount = 0; // For counting vertices

    for (auto f : mesh.faces())
    {
        CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
        std::vector<Point> vertices;

        // Collect vertices of the face
        for (boost::tie(vbegin, vend) = vertices_around_face(mesh.halfedge(f), mesh); vbegin != vend; ++vbegin)
        {
            vertices.push_back(mesh.point(*vbegin));
        }

        if (vertices.size() == 3)
        { // Ensure the face is a triangle
            // Calculate the normal using the cross product
            Kernel::Vector_3 normal = CGAL::cross_product(vertices[1] - vertices[0], vertices[2] - vertices[0]);
            normal = normal / std::sqrt(normal.squared_length()); // Normalize the vector

            // Store the normal and vertex coordinates
            for (int i = 0; i < 3; i++)
            {
                // Store normals
                nom.push_back(static_cast<GLfloat>(normal.x()));
                nom.push_back(static_cast<GLfloat>(normal.y()));
                nom.push_back(static_cast<GLfloat>(normal.z()));

                // Store vertex coordinates
                vtx.push_back(static_cast<GLfloat>(vertices[i].x()));
                vtx.push_back(static_cast<GLfloat>(vertices[i].y()));
                vtx.push_back(static_cast<GLfloat>(vertices[i].z()));
            }

            vertexCount += 3; // Increment the vertex count by three for each triangle processed
        }
    }

    populateColors(col, vtx.size() / 3);
}

void display(std::vector<GLfloat> vtx, std::vector<GLfloat> nom, std::vector<GLfloat> col)
{
    if (0 < vtx.size())
    {
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer(4, GL_FLOAT, 0, col.data());
        glVertexPointer(3, GL_FLOAT, 0, vtx.data());
        glNormalPointer(GL_FLOAT, 0, nom.data());
        glDrawArrays(GL_TRIANGLES, 0, vtx.size() / 3);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
    }
}

int callViewer(std::vector<GLfloat> vtx, std::vector<GLfloat> nom, std::vector<GLfloat> col) // callViewer(const Mesh& mesh)
{

    YsVec3 minmax[2];
    GetBoundingBox(minmax, vtx);

    YsMatrix4x4 Rc;
    YsVec3 target = (minmax[0] + minmax[1]) / 2.0;
    double dist = (minmax[1] - minmax[0]).GetLength();

    std::cout << "Press \"Esc\" in graphics window to input into terminal" << std::endl;

    for (;;)
    {
        FsPollDevice();
        auto key = FsInkey();
        if (FSKEY_ESC == key)
        {
            break;
        }

        if (0 != FsGetKeyState(FSKEY_LEFT))
        {
            Rc.RotateXZ(0.01);
        }
        if (0 != FsGetKeyState(FSKEY_RIGHT))
        {
            Rc.RotateXZ(-0.01);
        }
        if (0 != FsGetKeyState(FSKEY_UP))
        {
            Rc.RotateZY(0.01);
        }
        if (0 != FsGetKeyState(FSKEY_DOWN))
        {
            Rc.RotateZY(-0.01);
        }

        int lb, mb, rb, mx, my;
        auto evt = FsGetMouseEvent(lb, mb, rb, mx, my);
        if (FSMOUSEEVENT_LBUTTONDOWN == evt)
        {
        }

        int wid, hei;
        FsGetWindowSize(wid, hei);
        double aspect = (double)wid / (double)hei;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_DEPTH_TEST);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0, aspect, 0.1, 100.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        GLfloat lightDir[] = {0, 1 / sqrt(2.0f), 1 / sqrt(2.0f), 0};
        glLightfv(GL_LIGHT0, GL_POSITION, lightDir);
        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        auto Rinv = Rc;
        Rinv.Invert();

        YsMatrix4x4 modelView;
        GLdouble modelViewD[16];
        modelView.Translate(YsVec3(0, 0, -dist));
        modelView *= Rinv;
        modelView.Translate(-target);

        modelView.GetOpenGlCompatibleMatrix(modelViewD);
        glMultMatrixd(modelViewD);

        display(vtx, nom, col);

        FsSwapBuffers();

        FsSleep(10);
    }

    return 0;
}

void generateBasicColors(std::vector<GLfloat> &col)
{
    // Clear the existing colors in the vector
    col.clear();

    // Reserve space for efficiency
    col.reserve(7 * 4); // 7 colors, each with 4 components (RGBA)

    // Predefined colors with RGBA components
    std::vector<std::vector<GLfloat>> colors =
        {
            {1.0f, 0.0f, 0.0f, 1.0f}, // Red
            {0.0f, 1.0f, 0.0f, 1.0f}, // Green
            {0.0f, 0.0f, 1.0f, 1.0f}, // Blue
            {1.0f, 1.0f, 0.0f, 1.0f}, // Yellow
            {1.0f, 0.0f, 1.0f, 1.0f}, // Purple
            {0.0f, 1.0f, 1.0f, 1.0f}, // Cyan
            {1.0f, 1.0f, 1.0f, 1.0f}  // White
        };

    // Add the predefined colors to the 'col' vector
    for (const auto &color : colors)
    {
        col.insert(col.end(), color.begin(), color.end()); // Push back all 4 RGBA components
    }
}

void printFeatures()
{
    std::cout << "\nWhat function do you want to implement?:" << std::endl;
    std::cout << "\"S\" for Segmentation" << std::endl;
    std::cout << "\"T\" for New Segmentation" << std::endl;
    std::cout << "\"A\" for Removing Artifacts" << std::endl;
    // std::cout << "\"F\" for Filling Holes" << std::endl;
    std::cout << "\"P\" for Poisson Reconstruction" << std::endl;
    std::cout << "\"R\" for Repairing Mesh" << std::endl;
    // std::cout << "\"C\" for Consolidating Vertices" << std::endl;
    std::cout << "\"I\" for Mesh Information" << std::endl;
    std::cout << "\"X\" for Reseting Mesh" << std::endl;
}

void executeCommand(std::string c,
                    MeshEditor &editor,
                    Mesh &mesh,
                    std::vector<GLfloat> &vtx,
                    std::vector<GLfloat> &nom,
                    std::vector<GLfloat> &col)
{
    if (c == "S")
    {
        std::vector<Mesh> meshVec = editor.segmentation(mesh);

        vtx.clear();
        nom.clear();
        col.clear();
        for (size_t i = 0; i < meshVec.size(); ++i)
        {
            segmentationMeshData(meshVec[i], vtx, nom, col);
        }

        callViewer(vtx, nom, col);
    }
    if (c == "T")
    {
        std::cout << "This will take some time. Please wait." << std::endl;
        std::vector<Mesh> meshVec = editor.segmentMesh(mesh);

        vtx.clear();
        nom.clear();
        col.clear();
        for (size_t i = 0; i < meshVec.size(); ++i)
        {
            segmentationMeshData(meshVec[i], vtx, nom, col);
        }

        callViewer(vtx, nom, col);
    }
    if (c == "A")
    {
        std::size_t threshold;
        std::cout << "Enter a threshold for artifact size: ";
        std::cin >> threshold;

        std::cout << std::endl
                  << "Removing Artifacts" << std::endl;
        editor.removeSmallComponents(threshold);

        vtx.clear();
        nom.clear();
        col.clear();

        Mesh newMesh = editor.getEditedMesh();
        prepareMeshData(newMesh, vtx, nom, col);
        callViewer(vtx, nom, col);
    }
    // if (c == "F")
    // {
    //     std::cout << "Filling Holes" << std::endl;
    //     editor.fillHoles();

    //     vtx.clear();
    //     nom.clear();
    //     col.clear();

    //     Mesh newMesh = editor.getEditedMesh();
    //     prepareMeshData(newMesh, vtx, nom, col);
    //     callViewer(vtx, nom, col);
    // }
    if (c == "P")
    {
        std::cout << "Conducting Poisson Reconstruction" << std::endl;
        editor.reconstruction();

        vtx.clear();
        nom.clear();
        col.clear();

        Mesh newMesh = editor.getEditedMesh();
        prepareMeshData(newMesh, vtx, nom, col);
        callViewer(vtx, nom, col);
    }
    if (c == "R")
    {
        std::cout << "Repairing Mesh" << std::endl;
        editor.repairMesh();

        vtx.clear();
        nom.clear();
        col.clear();

        Mesh newMesh = editor.getEditedMesh();
        prepareMeshData(newMesh, vtx, nom, col);
        callViewer(vtx, nom, col);
    }
    // if (c == "C")
    // {
    //     std::cout << "Consolidating Vertices" << std::endl;
    //     editor.consolidateVertices();

    //     vtx.clear();
    //     nom.clear();
    //     col.clear();

    //     Mesh newMesh = editor.getEditedMesh();
    //     prepareMeshData(newMesh, vtx, nom, col);
    //     callViewer(vtx, nom, col);
    // }
    if (c == "I")
    {
        std::cout << "Mesh Analysis:" << std::endl;
        editor.analyzeMesh();
    }
    if (c == "X")
    {
        std::cout << "Reseting to Original Mesh" << std::endl;
        editor.reset();

        vtx.clear();
        nom.clear();
        col.clear();

        Mesh newMesh = editor.getEditedMesh();
        prepareMeshData(newMesh, vtx, nom, col);
        callViewer(vtx, nom, col);
    }
}

//////////////////////////////////////////////////////////////////////////////

int main()
{

    MongoDBClient dbManager; // Create an instance of MongoDBManager
    FsOpenWindow(0, 0, 800, 600, 1);
    Mesh mesh;

    for (;;)
    {
        // Display existing documents
        std::cout << "\n"
                  << std::endl;
        dbManager.displayDocuments();

        std::vector<GLfloat> vtx, nom, col;

        // User interaction for specific operations
        std::cout << "\nEnter an ID or Timestamp to download the corresponding mesh data:" << std::endl;
        std::cout << "Or enter \"Save\" to save current mesh or \"Exit\" to exit app : " << std::endl;
        std::string input;
        std::getline(std::cin, input); // Get user input

        if (input == "Exit" || input == "exit")
        {
            std::cout << "Exiting program ..." << std::endl;
            return 0;
        }

        if (input == "Save" || input == "save")
        {
            if (vtx.empty() || nom.empty())
            {
                std::cout << "Saving mesh ..." << std::endl;
                std::vector<char> binarySTL = createBinarySTL(vtx, nom);
                dbManager.uploadBinarySTLtoMongoDB(binarySTL, "editedMesh");
                continue;
            }
        }

        auto optional_binary_stl = dbManager.downloadMeshData(input); // need to check if this is null or not

        if (!optional_binary_stl)
        {
            std::cerr << "Failed to download mesh data." << std::endl;
            return 1; // Exit if no data is found
        }

        // Retrieve the vector from the optional
        std::vector<uint8_t> &binary_stl = *optional_binary_stl;
        // Setup a stream buffer using Boost iostreams to read binary data into CGAL
        namespace bio = boost::iostreams;
        bio::stream<bio::array_source> stream(reinterpret_cast<const char *>(binary_stl.data()), binary_stl.size());

        // Read the STL file and populate the surface mesh
        if (!read_stl_to_surface_mesh(stream, mesh))
        {
            return 1; // Error occurred during reading
        }
        MeshEditor editor(mesh);

        prepareMeshData(mesh, vtx, nom, col);
        callViewer(vtx, nom, col);

        bool con = true;
        while (con)
        {
            printFeatures();
            std::string color_input;
            std::getline(std::cin, color_input); // Get user input
            executeCommand(color_input, editor, mesh, vtx, nom, col);

            std::cout << "\nDo you want to implement another function or pick a new mesh:" << std::endl;
            std::cout << "Press Key for another segementation or \"New Mesh\" or \"Exit\" : " << std::endl;
            std::string choice_input;
            std::getline(std::cin, choice_input); // Get user input

            if (choice_input == "New Mesh" || choice_input == "new mesh")
            {
                con = false;
            }
            if (choice_input == "Exit" || choice_input == "exit")
            {
                std::cout << "Exiting program ..." << std::endl;
                return 0;
            }
        }
    }

    return 0;
}
