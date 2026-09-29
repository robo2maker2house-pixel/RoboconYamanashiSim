using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class TurtleBotMeshRepair
{
    private const string MeshRoot =
        "Assets/turtlebot3_description/urdf/" +
        "turtlebot3_description/meshes";

    [MenuItem("Tools/TurtleBot3/Repair Missing Meshes")]
    private static void RepairMissingMeshes()
    {
        MeshFilter[] filters = Object.FindObjectsByType<MeshFilter>(
            FindObjectsInactive.Include,
            FindObjectsSortMode.None);

        int repaired = 0;

        foreach (MeshFilter filter in filters)
        {
            if (filter.sharedMesh != null)
                continue;

            string objectName = filter.gameObject.name;
            string[] guids = AssetDatabase.FindAssets(
                $"t:Mesh {objectName}",
                new[] { MeshRoot });

            foreach (string guid in guids)
            {
                string path = AssetDatabase.GUIDToAssetPath(guid);
                string fileName =
                    Path.GetFileNameWithoutExtension(path);

                if (fileName != objectName)
                    continue;

                Mesh mesh = AssetDatabase.LoadAssetAtPath<Mesh>(path);

                if (mesh == null)
                    continue;

                Undo.RecordObject(filter, "Repair TurtleBot Mesh");
                filter.sharedMesh = mesh;
                EditorUtility.SetDirty(filter);
                PrefabUtility.RecordPrefabInstancePropertyModifications(
                    filter);

                repaired++;
                break;
            }
        }

        if (repaired > 0)
        {
            EditorSceneManager.MarkSceneDirty(
                EditorSceneManager.GetActiveScene());

            Debug.Log(
                $"TurtleBot3: {repaired}ŒÂ‚ÌMeshQÆ‚ğC•œ‚µ‚Ü‚µ‚½B");
        }
        else
        {
            Debug.LogWarning(
                "TurtleBot3: C•œ‘ÎÛ‚ÌMissing Mesh‚ª‚ ‚è‚Ü‚¹‚ñB");
        }
    }
}