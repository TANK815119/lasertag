using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//this script's enitre prupose is to despawn spent casings
//its a disty and expensive use of Instantiate and Destroy for sometthing
//rapid like a bullet, buit this works for a test object
//look into object "pools" for something more effective

namespace Rekabsen.AutoDrone
{
    public class SpentCasing : MonoBehaviour
    {
        [SerializeField] private float collisionTimer = 0.25f;
        [SerializeField] private float despawnTimer = 10f;
        [SerializeField] private bool immortal = false;

        private bool colEnabled = true;
        // Start is called before the first frame update
        void Start()
        {
            if (gameObject.TryGetComponent(out Collider col))
            {
                col.enabled = false;
                colEnabled = false;
            }

        }

        // Update is called once per frame
        void Update()
        {
            //despawning
            despawnTimer -= Time.deltaTime;
            if (despawnTimer < 0f && !immortal)
            {
                Destroy(gameObject);
            }

            //collision
            if (colEnabled == false)
            {
                collisionTimer -= Time.deltaTime;
                if (collisionTimer < 0f)
                {
                    if (gameObject.TryGetComponent(out Collider col))
                    {
                        col.enabled = true;
                        colEnabled = true;
                    }
                }
            }
        }
    }
}