using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Rekabsen.AutoDrone
{
    public class Explode : MonoBehaviour
    {
        [SerializeField] private float explosionRadius = 5f;
        [SerializeField] private float explosionForce = 700f;

        // Start is called before the first frame update
        void Start()
        {
            TriggerExplosion(transform.position, explosionRadius, explosionForce);
        }

        void TriggerExplosion(Vector3 position, float radius, float force)
        {
            Collider[] colliders = Physics.OverlapSphere(position, radius);
            foreach (Collider hit in colliders)
            {
                Rigidbody rb = hit.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.AddExplosionForce(force, position, radius, 1f, ForceMode.Impulse);
                }
            }
        }
    }
}