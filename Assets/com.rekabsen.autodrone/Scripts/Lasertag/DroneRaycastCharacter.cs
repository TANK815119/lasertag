using Anaglyph.Lasertag;
using UnityEngine;

namespace Rekabsen.AutoDrone
{
	public class DroneRaycastCharacter : NonPlayerCharacter
	{
		[SerializeField] private VoxelAvoidanceRaytrace voxelAvoidance;
		[SerializeField] private float impactForceMultiplier = 1f;

		public void OnShot(Bullet.DamageData damageData)
		{
			TakeDamage(damageData.damage);

			// Add an impact force using the drone's position relative to the player
			// In the ftuture, modify DamageData to include more projectile information

			GameObject poi = voxelAvoidance.GetPOI();
			if (poi != null)
			{
				Vector3 impactDirection = (transform.position - poi.transform.position).normalized;
				voxelAvoidance.transform.GetComponentInChildren<Rigidbody>().AddForce(impactDirection * damageData.damage * impactForceMultiplier, ForceMode.Impulse);
			}
			else
			{
				Debug.LogWarning("DroneRaycastCharacter has no POI set for applying impact force.");
			}
		}

		protected override void Die()
		{
			voxelAvoidance.detonate();
		}
	}
}