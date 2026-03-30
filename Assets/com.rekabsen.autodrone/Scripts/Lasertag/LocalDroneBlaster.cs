using Anaglyph.Lasertag.Logistics;
using Unity.Netcode;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.InputSystem;

namespace Rakebsen.Autodrone
{
	public class LocalDroneBlaster : MonoBehaviour
	{
		[SerializeField] private int fixedUpdatesPerFire = 5;
		private int fixedUpdateTilNextFire = 0;

		[SerializeField] private GameObject boltPrefab = null;
		[SerializeField] private Transform emitFromTransform = null;
		public UnityEvent onFire = new();

		[SerializeField] private bool firing = true;

		private void FixedUpdate()
		{
			if (!NetworkManager.Singleton.IsConnectedClient)
				return;

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

			n.SpawnWithOwnership(NetworkManager.Singleton.LocalClientId);

			onFire.Invoke();
		}
	}
}