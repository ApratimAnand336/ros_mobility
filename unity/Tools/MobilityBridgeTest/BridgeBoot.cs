using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// Builds a flat-ground scene with the coupled rover (MobilityBridge auto-attaches at Play)
/// and enters play mode. Used by run_bridge_test.sh in a throwaway project only.
public static class BridgeBoot
{
    public static void Run()
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
        ground.name = "Ground";
        ground.transform.localScale = new Vector3(400f, 1f, 400f);
        ground.transform.position = new Vector3(0f, -0.5f, 0f);

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/PreFabs/urdf_full_withdifferential_coupled.prefab");
        var rover = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        float minY = float.PositiveInfinity;
        foreach (var r in rover.GetComponentsInChildren<Renderer>()) minY = Mathf.Min(minY, r.bounds.min.y);
        rover.transform.position += Vector3.up * (0.01f - minY);

        new GameObject("BridgeTestRunner").AddComponent<BridgeTestRunner>();
        EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene(), "Assets/Test/BridgeFlat.unity");
        EditorApplication.EnterPlaymode();
    }
}
