using System.Collections;
using System.Collections.Generic;
using JetBrains.Annotations;
using UnityEngine;
using UnityEngine.Events;

namespace Rekabsen.AutoDrone
{
    [RequireComponent(typeof(Collider))]
    public class WarheadListener : MonoBehaviour
    {
        private List<Collider> validColliders;
        private Transform POI;

        public UnityEvent OnCollision;

        public WarheadListener(Transform POI)
        {
            SetPOI(POI);
        }

        public void SetPOI(Transform POI)
        {
            this.POI = POI;

            //find all colliders in the POI hierarchy   
            validColliders = FindAllComponentsInHierarchy<Collider>(POI.gameObject);

            //case in which there are no colliders
            if (validColliders.Count == 0)
            {
                Debug.LogWarning("WarheadListener: No colliders found in the POI hierarchy.");
                return;
            }

            //case in which there are colliders and rigidbody
            if (validColliders.Count != 0 && validColliders[0].attachedRigidbody != null)
            {
                validColliders = FindAllComponentsInHierarchy<Collider>(validColliders[0].attachedRigidbody.gameObject);
                return;
            }

            //case in which there are colliders but no rigidbody
            if (validColliders.Count != 0 && validColliders[0].attachedRigidbody == null)
            {
                //do nothing i guess
                return;
            }
        }

        public static List<T> FindAllComponentsInHierarchy<T>(GameObject root) where T : Component
        {
            List<T> components = new List<T>();

            // Get component from root if it exists
            T rootComponent = root.GetComponent<T>();
            if (rootComponent != null)
            {
                components.Add(rootComponent);
            }

            // Recursively search through all children
            foreach (Transform child in root.transform)
            {
                components.AddRange(FindAllComponentsInHierarchy<T>(child.gameObject));
            }

            return components;
        }

        private void OnTriggerEnter(Collider other)
        {
            if (validColliders.Contains(other))
            {
                OnCollision?.Invoke();
            }
        }
    }
}