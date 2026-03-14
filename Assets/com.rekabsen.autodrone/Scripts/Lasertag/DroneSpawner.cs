using System.Collections.Generic;
using JetBrains.Annotations;
using NUnit.Framework;
using Rekabsen.AutoDrone;
using UnityEngine;

public class DroneSpawner : MonoBehaviour
{
	[SerializeField] private GameObject dronePrefab;
	[SerializeField] private float droneSpawnInterval = 10f;
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
		if (DroneCount() >= 3)
		{
			Debug.Log("Maximum drone count reached. Skipping spawn.");
			return;
		}

		GameObject drone = Instantiate(dronePrefab, transform.position, Quaternion.identity);
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
