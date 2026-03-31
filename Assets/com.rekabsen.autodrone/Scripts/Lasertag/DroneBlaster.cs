using System.Net.NetworkInformation;
using Anaglyph.Lasertag.Logistics;
using Rekabsen.AutoDrone;
using Unity.Netcode;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.InputSystem;

namespace Rakebsen.Autodrone
{
	public class DroneBlaster : MonoBehaviour
	{
		[SerializeField] private int fixedUpdatesPerFire = 5;
		private int fixedUpdateTilNextFire = 0;

		[SerializeField] private GameObject boltPrefab = null;
		[SerializeField] private Transform emitFromTransform = null;
		public UnityEvent onFire = new();

		private bool firing;

		[SerializeField] private VoxelAvoidanceRaytrace voxelAvoidanceRaytrace;
		[SerializeField] private string targetTag = "Player";
		[SerializeField] [Tooltip("Max shot angle variance in degrees")] private float shotVariance = 5f; // Add some variance to the shot to prevent perfect accuracy

		private void FixedUpdate()
		{
			if (!NetworkManager.Singleton.IsConnectedClient)
				return;

			// Only fire if the blaster is aimed at the target
			if (Physics.Raycast(emitFromTransform.transform.position, emitFromTransform.transform.forward, out RaycastHit hitInfo))
			{
				firing = hitInfo.collider.gameObject.CompareTag(targetTag);
				//Debug.Log($"DroneBlaster: Raycast hit {hitInfo.collider.gameObject.name}, with tag {hitInfo.collider.tag}");
			}
			else
			{
				firing = false;
			}

			if (firing)
			{
				fixedUpdateTilNextFire -= 1;

				if (fixedUpdateTilNextFire <= 0)
				{
					Fire();
					fixedUpdateTilNextFire = fixedUpdatesPerFire;
				}
			}
			else
			{
				fixedUpdateTilNextFire = 0;
			}

		}

		public void Fire()
		{
			if (!NetworkManager.Singleton.IsConnectedClient)
				return;

			// var e = emitFromTransform;
			// NetworkObject.InstantiateAndSpawn(boltPrefab, NetworkManager.Singleton,
			// 	position: e.position, rotation: e.rotation,
			// 	ownerClientId: NetworkManager.Singleton.LocalClientId);

			NetworkObject n = NetworkObjectPool.Instance.GetNetworkObject(
				boltPrefab, emitFromTransform.position, emitFromTransform.rotation);

			n.transform.Rotate(Random.Range(-shotVariance, shotVariance),
				Random.Range(-shotVariance, shotVariance),
				Random.Range(-shotVariance, shotVariance));		

			n.SpawnWithOwnership(NetworkManager.Singleton.LocalClientId);

			onFire.Invoke();
		}
	}
}