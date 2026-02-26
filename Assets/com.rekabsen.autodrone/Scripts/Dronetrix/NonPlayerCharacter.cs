using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Rekabsen.AutoDrone
{
    public class NonPlayerCharacter : MonoBehaviour
    {
        [SerializeField] private float health;
        public List<Collider> Body { get; set; }
        // Start is called before the first frame update
        void Start()
        {
            Body = WarheadListener.FindAllComponentsInHierarchy<Collider>(gameObject);
        }

        public void TakeDamage(float damage)
        {
            health -= damage;
            if (health <= 0)
            {
                Die();
            }
        }

        protected virtual void Die()
        {
            // Handle character death (e.g., play animation, remove from game)
        }
    }
}