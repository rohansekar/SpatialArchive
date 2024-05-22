using UnityEngine;
using UnityEngine.Networking;
using System.Collections;
using System.IO;
using System.Text;
using Microsoft.MixedReality.Toolkit.SpatialAwareness;
using Microsoft.MixedReality.Toolkit;
using System; // This is generally not necessary but included here for clarity
using System.Collections.Generic;

public class NewBinary : MonoBehaviour
{
    public void ExportScene()
    {
        StartCoroutine(SendMeshDataToServer());
    }

    IEnumerator SendMeshDataToServer()
    {
        // Assuming SerializeMeshData() is a method that serializes your mesh data into a byte array.
        string meshName = "BedRoom"; // Example mesh name
        byte[] meshData = SerializeMeshData();
        Debug.Log($"Mesh Data Length: {meshData.Length}");

        //string serverURL = "http://localhost:5000/upload";
        string serverURL = "http://192.168.0.168:5000/upload";
        UnityWebRequest request = new UnityWebRequest(serverURL, "POST");
        //request.uploadHandler = (UploadHandler)new UploadHandlerRaw(jsonPayload);
        request.uploadHandler = new UploadHandlerRaw(meshData);
        request.downloadHandler = (DownloadHandler)new DownloadHandlerBuffer();
        request.SetRequestHeader("Content-Type", "application/octet-stream");

        yield return request.SendWebRequest();

        if (request.isNetworkError || request.isHttpError)
        {
            Debug.LogError(request.error);
        }
        else
        {
            Debug.Log("Mesh data sent successfully.");
        }
    }

    public byte[] SerializeMeshData()
    {
        List<MeshFilter> meshes = new List<MeshFilter>();

        var spatialAwarenessSystem = CoreServices.SpatialAwarenessSystem;
        spatialAwarenessSystem.SuspendObservers();

        var observer = CoreServices.GetSpatialAwarenessSystemDataProvider<IMixedRealitySpatialAwarenessMeshObserver>();

        // Serialize the mesh data to a memory stream instead of a file
        using (var ms = new MemoryStream())
        {
            using (var writer = new BinaryWriter(ms))
            {
                // Write STL header
                writer.Write(new byte[80]);

                int triangleCount = 0; // Counter for total number of triangles

                // Collect mesh filters
                foreach (var meshObject in observer.Meshes.Values)
                {
                    meshes.Add(meshObject.Filter);
                }

                // Count triangles
                foreach (MeshFilter meshFilter in meshes)
                {
                    triangleCount += meshFilter.sharedMesh.triangles.Length / 3;
                }
                writer.Write((uint)triangleCount);

                // Write each triangle
                foreach (MeshFilter meshFilter in meshes)
                {
                    Mesh mesh = meshFilter.sharedMesh;

                    for (int i = 0; i < mesh.triangles.Length; i += 3)
                    {
                        Vector3 normal = Vector3.Cross(
                            mesh.vertices[mesh.triangles[i + 1]] - mesh.vertices[mesh.triangles[i]],
                            mesh.vertices[mesh.triangles[i + 2]] - mesh.vertices[mesh.triangles[i]]).normalized;

                        writer.Write(normal.x);
                        writer.Write(normal.y);
                        writer.Write(normal.z);

                        for (int j = 0; j < 3; j++)
                        {
                            Vector3 vertex = mesh.vertices[mesh.triangles[i + j]];
                            writer.Write(vertex.x);
                            writer.Write(vertex.y);
                            writer.Write(vertex.z);
                        }

                        writer.Write((ushort)0); // Attribute byte count
                    }
                }

                writer.Flush();
                ms.Seek(0, SeekOrigin.Begin);

                // Convert the STL string to a byte array
                return ms.ToArray();
            }

        }

    }
}