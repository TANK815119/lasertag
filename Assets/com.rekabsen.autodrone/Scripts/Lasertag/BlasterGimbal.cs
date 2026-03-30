using Rekabsen.AutoDrone;
using UnityEngine;

namespace Rakebsen.Autodrone
{
	public class BlasterGimbal : MonoBehaviour
	{
		[SerializeField] private Transform boltEmitter;
		[SerializeField] private Transform pivot;
		[SerializeField] private VoxelAvoidanceRaytrace voxelAvoidance;

		// Start is called once before the first execution of Update after the MonoBehaviour is created
		void Start()
		{
			if (voxelAvoidance == null) Debug.LogError("BlasterGimbal: No VoxelAvoidanceRaytrace assigned.");
			if (boltEmitter == null) Debug.LogError("BlasterGimbal: No bolt emitter assigned.");
			if (pivot == null) Debug.LogError("BlasterGimbal: No pivot assigned.");
		}

		// Update is called once per frame
		void Update()
		{
			// Find the vector from the pivot to the target and make the blaster look at it via the local x and y axis(not z)
			Vector3 vector = voxelAvoidance.GetPOI().transform.position - pivot.position;

			Transform blasterParent = transform.parent; // Store current parent to restore later
			transform.parent = boltEmitter; // Make bolt emitter the parent so that it moves with it

			float yawError = Vector3.SignedAngle(boltEmitter.forward, vector, boltEmitter.parent.up); //error in degrees - y-axis rotationally and horizontal in drone space
			float pitchError = Vector3.SignedAngle(boltEmitter.forward, vector, boltEmitter.parent.right); //error in degrees - x-axis rotationally and vertical in drone space

			boltEmitter.RotateAround(pivot.position, boltEmitter.parent.up, yawError);
			boltEmitter.RotateAround(pivot.position, boltEmitter.parent.right, pitchError);

			boltEmitter.localRotation = Quaternion.Euler(boltEmitter.localRotation.eulerAngles.x, boltEmitter.localRotation.eulerAngles.y, 0f); // Lock the z rotation to prevent roll

			transform.parent = blasterParent; // Restore original parentage
		}
	}
}