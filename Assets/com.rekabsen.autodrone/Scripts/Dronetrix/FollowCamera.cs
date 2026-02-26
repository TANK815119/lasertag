using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Rekabsen.AutoDrone
{
    public class CameraFollow : MonoBehaviour
    {
        [SerializeField] private Transform target;
        [SerializeField] private Transform drone;
        [SerializeField] private Vector3 cameraOffset;
        [SerializeField] private float lerpSpeed = 3f; //interp amount per second

        private Transform newTransform;

        private void Start()
        {
            GameObject gameObj = new GameObject();
            newTransform = gameObj.transform;
        }

        // Update is called once per frame
        void Update()
        {
            //position
            Vector3 droneToTarget = (target.position - drone.position).normalized;
            newTransform.forward = droneToTarget;
            newTransform.position = drone.position;
            newTransform.position += droneToTarget * cameraOffset.z;
            newTransform.position += Vector3.up * cameraOffset.y;

            //rotation
            Vector3 cameraToTarget = (target.position - newTransform.position).normalized;
            newTransform.forward = cameraToTarget;

            //interpolate between transform and newTransform
            transform.position = Vector3.Lerp(transform.position, newTransform.position, lerpSpeed * Time.deltaTime);
            transform.rotation = Quaternion.Lerp(transform.rotation, newTransform.rotation, lerpSpeed * Time.deltaTime);
        }
    }
}