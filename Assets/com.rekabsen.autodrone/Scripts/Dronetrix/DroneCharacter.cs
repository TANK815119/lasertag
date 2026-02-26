using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Rekabsen.AutoDrone
{
    public class DroneCharacter : NonPlayerCharacter
    {
        [SerializeField] private VoxelAvoidance voxelAvoidance;

        protected override void Die()
        {
            voxelAvoidance.detonate();
        }
    }
}