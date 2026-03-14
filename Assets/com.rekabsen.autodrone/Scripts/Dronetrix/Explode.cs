using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Anaglyph.Lasertag.Logistics;
using Anaglyph.Lasertag;

namespace Rekabsen.AutoDrone
{
    public class Explode : MonoBehaviour
    {
        [SerializeField] private float explosionRadius = 5f;
        [SerializeField] private float explosionForce = 700f;
		[SerializeField] private float maxDamage = 200f;

		private Bullet.DamageData damageData;

		// Start is called before the first frame update
		void Start()
        {
            TriggerExplosion(transform.position, explosionRadius, explosionForce);
        }

        void TriggerExplosion(Vector3 position, float radius, float force)
        {
            Collider[] colliders = Physics.OverlapSphere(position, radius);
            foreach (Collider col in colliders)
            {
				// Attempt to damage
				damageData = new Bullet.DamageData
				{
					playerID = 0, // Zero for now, should be NetworkPlayerID of the drone's owner
					damage = calculateDamage(this.transform, col.transform)
				};

				col.transform.root.BroadcastMessage("OnShot", damageData, SendMessageOptions.DontRequireReceiver);

				// Add force
				Rigidbody rb = col.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.AddExplosionForce(force, position, radius, 1f, ForceMode.Impulse);
                }
            }
        }

		private float calculateDamage(Transform explosionOrigin, Transform victim)
		{
			float distance = Vector3.Distance(explosionOrigin.position, victim.position);
			return maxDamage / distance * distance; // Simple inverse square falloff
		}
	}
}