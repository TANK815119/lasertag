using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using UnityEngine;
using UnityEngine.Events;

namespace Rekabsen.AutoDrone
{
    public class DroneAudio : MonoBehaviour
    {
        [SerializeField] private AudioSource audioSource;
        [SerializeField] private Drone_3DTarget_Scaled drone;
        [SerializeField] private float minPitch = 0.75f;
        [SerializeField] private float midPitch = 1f;
        [SerializeField] private float maxPitch = 1.5f;
        [SerializeField] private float audioLerpSpeed = 0.1f;
        [SerializeField] private Transform FRProp;
        [SerializeField] private Transform BRProp;
        [SerializeField] private Transform FLProp;
        [SerializeField] private Transform BLProp;
        [SerializeField] private float maxPropRPM = 50000f;

        private Transform[] propTrans;

        // Start is called before the first frame update
        void Start()
        {
            drone.OnActionRecievedCalled.AddListener(ModulateAudio);
            drone.OnPropThrustCalled.AddListener(ModulatePropSpeed);
            propTrans = new Transform[] { FRProp, BRProp, FLProp, BLProp };
        }

        private void ModulateAudio(float lift) //thurst is between -1 and 1
        {
            //modulate drone audio to be louder and higher pitched at greater lift
            //and quieter and lower pitched at lower lift
            float volume = (lift + 1f) / 2f; //0 at 0 lift, 1 and max lift

            float pitch = 1f;
            if (lift <= 0)
            {
                pitch = Mathf.Lerp(minPitch, midPitch, lift + 1); // gravity compensation for negative thrust
            }
            else
            {
                pitch = Mathf.Lerp(midPitch, maxPitch, lift); // regular thrust for positive values
            }

            audioSource.volume = Mathf.Lerp(audioSource.volume, volume, audioLerpSpeed);
            audioSource.pitch = Mathf.Lerp(audioSource.pitch, pitch, audioLerpSpeed);
        }

        private void ModulatePropSpeed(float[] propThrusts, float maxPropThrust)
        {
            for (int i = 0; i < propTrans.Length; i++)
            {
                float RPM = (propThrusts[i] / maxPropThrust) * maxPropRPM; //scale thrust to RPM
                float speed = RPM / 60f * 360f; //degrees per second
                int direction = (i == 2 || i == 1) ? 1 : -1; //alternate direction for adjacent props
                propTrans[i].localRotation = Quaternion.Euler(propTrans[i].localEulerAngles.x, propTrans[i].localEulerAngles.y + speed * Time.deltaTime, propTrans[i].localEulerAngles.z);
                //propTrans[i].Rotate(propTrans[i].parent.up * speed * Time.deltaTime);
            }
        }

        private void OnDestroy()
        {
            drone.OnActionRecievedCalled.RemoveListener(ModulateAudio);
            drone.OnPropThrustCalled.RemoveListener(ModulatePropSpeed);

        }
    }
}