using UnityEngine;
using UnityEngine.Events;

namespace Anaglyph.DepthKit.Meshing
{
	[RequireComponent(typeof(MeshCollider), typeof(MeshFilter), typeof(MeshChunk))]
	public class MeshChunkCollider : MonoBehaviour
    {
		private MeshChunk meshChunk;
		private MeshCollider meshCollider;

        void Awake()
        {
			meshChunk = GetComponent<MeshChunk>();
			meshCollider = GetComponent<MeshCollider>();
			meshChunk.onMeshUpdated.AddListener(OnMeshUpdated);
		}

        private void OnMeshUpdated(Mesh mesh)
		{
			// Creal and re-assign mesh to update collider
			meshCollider.sharedMesh = null;
			meshCollider.sharedMesh = mesh;
		}
    }
}
