using System.Collections.Generic;
using JetBrains.Annotations;
using NUnit.Framework;
using Rekabsen.AutoDrone;
using UnityEngine;

namespace Rekabsen.AutoDrone
{
	public class DroneSpawner : MonoBehaviour
	{
		[SerializeField] private GameObject dronePrefab;
		[SerializeField] private GameObject droneMiniPrefab;
		[SerializeField] private GameObject droneBlasterPrefab;
		[SerializeField] private float droneSpawnInterval = 10f;
		[SerializeField] private int maxDrones = 2;
		[SerializeField] private Transform poi;
		private List<GameObject> spawnedDrones = new();

		// Start is called once before the first execution of Update after the MonoBehaviour is created
		void Start()
		{
			// For the purpose of not being in a wall, amke the spawn position the innitial head position
			this.transform.position = poi.position;
			DroneSpawnLoop();
		}

		private async void DroneSpawnLoop()
		{
			while (enabled)
			{
				await Awaitable.WaitForSecondsAsync(droneSpawnInterval);
				SpawnDrone();
			}
		}

		private void SpawnDrone()
		{
			if (DroneCount() >= maxDrones)
			{
				Debug.Log("Maximum drone count reached. Skipping spawn.");
				return;
			}

			// 50% chance to spawn a mini drone instead of a regular one
			GameObject drone = null;
			float random = Random.value;
			if (random < 0.33f)
			{
				drone = Instantiate(dronePrefab, transform.position, Quaternion.identity);
			}
			else if (random < 0.66f)
			{
				drone = Instantiate(droneMiniPrefab, transform.position, Quaternion.identity);
			}
			else
			{
				drone = Instantiate(droneBlasterPrefab, transform.position, Quaternion.identity);
			}

			if (drone == null)
			{
				Debug.LogError("Failed to spawn drone. Prefab might be missing.");
				return;
			}

			spawnedDrones.Add(drone);
			if (drone.TryGetComponent(out VoxelAvoidanceRaytrace pathfinding))
			{
				pathfinding.SetPOI(poi);
			}
			else
			{
				Debug.LogWarning("DroneSpawner is missing a VoxelAvoidanceRaytrace component for pathfinding.");
			}
		}

		private int DroneCount()
		{
			spawnedDrones.RemoveAll(d => d == null); // Clean up destroyed drones
			return spawnedDrones.Count;
		}
	}
}