using System;
using System.Net.NetworkInformation;
using System.Runtime.CompilerServices;
using TMPro;
using Unity.MLAgents.Integrations.Match3;
using UnityEngine;
using UnityEngine.Animations;
using UnityEngine.UIElements;

namespace Rekabsen.AutoDrone
{
    public class ObjectAvoidance : MonoBehaviour
    {
        [SerializeField] private Transform target; //Sets up a smaller target that stays in front of the drone to guide it
        [SerializeField] private Transform poi; //Sets up an end goal, which the smaller target navigates to
        [SerializeField] private float turnModMinusVal; //Sets turn mod value
        [SerializeField] private float turnModPlusVal;
        [SerializeField] private float speed; //Sets the initial speed value

        // Start is called before the first frame update
        void Start()
        {
            turnMod = 1f;
            Vector3 rotation = (poi.position - transform.position).normalized;
            target.forward = rotation;
            target.rotation = Quaternion.Euler(0f, target.eulerAngles.y, 0f);
        }

        Vector3 distance;
        float turnMod = 1f;

        // Update is called once per frame
        void Update()
        {
            speed *= turnMod;
            distance = poi.position - target.position;
            Vector3 rotation = (poi.position - transform.position).normalized;
            target.forward = rotation;
            target.rotation = Quaternion.Euler(0f, target.eulerAngles.y, 0f);
            if (CheckFront())
            {
                target.Translate(Vector3.forward * speed * 1.5f * Time.deltaTime);
            }
            else
            {
                if (DroneCheck() == 2)
                {
                    transform.Translate(Vector3.right * speed * Time.deltaTime);
                }
                else if (DroneCheck() == 1)
                {
                    transform.Translate(Vector3.left * speed * Time.deltaTime);
                }
                if (CheckRight())
                {
                    Debug.Log("Check Right");
                    turnMod -= turnModMinusVal * Time.deltaTime;
                    target.Translate(Vector3.right * speed * 1.5f * Time.deltaTime);
                }
                else if (CheckLeft())
                {
                    Debug.Log("Check Left");
                    turnMod -= turnModMinusVal * Time.deltaTime;
                    target.Translate(Vector3.left * speed * 1.5f * Time.deltaTime);
                }
                else if (CheckUp())
                {
                    Debug.Log("Check Up");
                    turnMod -= turnModMinusVal * Time.deltaTime;
                    target.Translate(Vector3.up * speed * 1.5f * Time.deltaTime);
                }
                else if (CheckDown())
                {
                    Debug.Log("Check Down");
                    turnMod -= turnModMinusVal * Time.deltaTime;
                    target.Translate(Vector3.down * speed * 1.5f * Time.deltaTime);
                }
                else
                {
                    //Loiter();
                }
            }
            turnMod += turnModPlusVal * Time.deltaTime;
        }

        int DroneCheck()
        {
            Debug.DrawRay(transform.position, transform.forward, Color.red, 2f);
            if (Physics.Raycast(transform.position, transform.forward, 2f))
            {
                if (Physics.Raycast(transform.position, transform.right, 2f)) //Returns 1 if there is something to the right
                {
                    return 1;
                }
                else //Returns 2 if there is something to the left
                {
                    return 2;
                }
            }
            return 0; //Returns 0 if the front is clear
        }

        bool CheckFront()
        {
            Debug.DrawRay(target.position, target.forward, Color.red, 2f);
            //return Physics.SphereCast(target.position, .25f, target.forward, out RaycastHit hitInfo, 2f);

            if (Physics.Raycast(target.position, target.forward, 2f))
                return false;
            else return true;

        }

        bool CheckRight()
        {
            Debug.DrawRay(target.position, transform.right, Color.green, 2f);
            if (Physics.Raycast(target.position, target.right, 2f))
                return false;
            else return true;
            //return Physics.SphereCast(target.position, .25f, target.right, out RaycastHit hitInfo, 2f);
        }
        bool CheckLeft()
        {
            Debug.DrawRay(target.position, -transform.right, Color.yellow, 2f);
            if (Physics.Raycast(target.position, -target.right, 2f))
                return false;
            else return true;
            //return Physics.SphereCast(target.position, .25f, -target.right, out RaycastHit hitInfo, 2f);
        }

        bool CheckUp()
        {
            Debug.DrawRay(target.position, transform.up, Color.magenta, 2f);
            if (Physics.Raycast(target.position, target.up, 2f))
                return false;
            else return true;
            //return Physics.SphereCast(target.position, .25f, target.up, out RaycastHit hitInfo, 2f);
        }

        bool CheckDown()
        {
            Debug.DrawRay(target.position, -transform.up, Color.blue, 2f);
            if (Physics.Raycast(target.position, -target.up, 2f))
                return false;
            else return true;
            //return Physics.SphereCast(target.position, .25f, -target.up, out RaycastHit hitInfo, 2f);
        }

        void Loiter()
        {
            Debug.Log("Loitering");
            turnMod = 0;
            target.position = transform.position;
        }
    }
    /**
     * update()
     *  float speed
     *  drone pos
     *  poipos
     *  targetpos
     *  float turnMod
     *  if(checkFront)
     *      target moves 2 meters towards poi
     *  else
     *      checkRight -- if yes move right 2 meters, turnMod += .1
     *      checkLeft
     *      checkUp
     *      checkDown
     *      speed = |target - pos| * turnMod
     *      turnMod -= .01
     *      
    
    /*
         * { 
        Vector3 targetPosition = target.position; //Sets a vector for the position of the target
        Vector3 dronePosition = transform.position; //Sets a vector for the position of the drone
        Vector3 poiPosition = poi.position; //Sets a vector for the position of the POI
        float angle = 0; //Sets the initial angle of raycasts being sent out to 180 degrees AKA straight out
        int depthLimiter = 0;

        Vector3 droneDirection = poi.position - transform.position; //Sets a vector for the current direction of the drone
        target.forward = droneDirection; //Sets the target to face towards the direction of the drone
        target.rotation = Quaternion.Euler(0f, target.eulerAngles.y, 0f); //Sets the target to rotate with the drone, making it so the drone always faces "forward"

        if (!IfBlocked(dronePosition, transform.forward, 0f))
            //!IfFrontBlocked(dronePosition, transform.forward)) //Sets the target to be at the position of the POI if the front is not blocked, so the drone moves forward 
        {
            target.position = poiPosition;
        } 
        while(depthLimiter < 999 && angle <= 180 && IfBlocked(dronePosition, transform.forward, angle)) //Enters while loop if the angle is greater than zero and something is blocking the drone
        {
            depthLimiter++;
            if (CheckLeft(angle, dronePosition)) //If/else tree checks which direction is clear for the drone to move to in order to avoid the obstacle
            {
                target.position = transform.position + ((Quaternion.AngleAxis(-1 * angle, target.forward) * targetPosition).normalized * 2f);
            }
            else if(CheckRight(angle, dronePosition))
            {
                target.position = transform.position + ((Quaternion.AngleAxis(angle, target.forward) * targetPosition).normalized * 2f);
            }
            else if(CheckUp(angle, dronePosition))
            {
                target.position = transform.position + ((Quaternion.AngleAxis(angle, target.up) * targetPosition).normalized * 2f);
            }
            else if(CheckDown(angle, dronePosition))
            {
                target.position = transform.position + ((Quaternion.AngleAxis(-1*angle, target.up) * targetPosition).normalized * 2f);
            }
            else //If no direction is clear, changes the angle in order to find a new direction until it is shooting directly in front
            {
                Debug.Log("The angle is " + angle);
                angle += 5f;
            }
        }

    }

    bool IfBlocked(Vector3 origin, Vector3 direction, float angle)
    {
        if (IfFrontBlocked(origin, direction) && !CheckLeft(angle, origin) && !CheckRight(angle, direction) && CheckUp(angle, origin) && CheckDown(angle, direction))
            return true;
        else
            return false;
    }

    bool IfFrontBlocked(Vector3 origin, Vector3 direction) //Sends out a raycast right in front of the drone to check for an obstacle. Returns true if there is an obstacle and false otherwise.
    {
        Debug.DrawRay(origin, transform.forward, Color.red, 2f);
        //return Physics.Raycast(origin, direction, 2f);
        return Physics.SphereCast(origin, .5f, direction, out RaycastHit hitInfo, 2f);
    }

    bool CheckLeft(float angle, Vector3 origin) //Sends out a raycast at angle to the left of the drone to check for an obstacle. Returns false if there is an obstacle and true otherwise.
    {
        //Debug.Log("Check Left"); //Prints for debugging purposes
        Vector3 vector = new Vector3();
        vector = Quaternion.AngleAxis(-1*angle, transform.up) * transform.forward;
        Debug.DrawRay(origin, vector, Color.green, 2f);
        if (Physics.SphereCast(origin, .5f, vector, out RaycastHit hitInfo, 2f)) //(Physics.Raycast(origin, vector, 2f)) //Checks if the 2 meter raycast from the drone at a specific angle hits something 
        {
            return false;
        }
        return true;
    }
     
    bool CheckRight(float angle, Vector3 origin) //Sends out a raycast at angle to the right of the drone to check for an obstacle. Returns false if there is an obstacle and true otherwise.
    {
        //Debug.Log("Check Right"); //Prints for debugging purposes
        Vector3 vector = new Vector3();
        vector = Quaternion.AngleAxis(angle, transform.up) * transform.forward;
        Debug.DrawRay(origin, vector, Color.blue, 2f);
        if (Physics.SphereCast(origin, .5f, vector, out RaycastHit hitInfo, 2f)) //(Physics.Raycast(origin, vector, 2f)) //Checks if the 2 meter raycast from the drone at a specific angle hits something 
        {
            return false;
        }
        return true;
    }

    bool CheckUp(float angle, Vector3 origin) //Sends out a raycast at angle above the drone to check for an obstacle. Returns false if there is an obstacle and true otherwise.
    {
        //Debug.Log("Check Up"); //Prints for debugging purposes
        Vector3 vector = new Vector3();
        vector = Quaternion.AngleAxis(angle, transform.right) * transform.forward;
        Debug.DrawRay(origin, vector, Color.magenta, 2f);
        if (Physics.SphereCast(origin, .5f, vector, out RaycastHit hitInfo, 2f)) //(Physics.Raycast(origin, vector, 2f)) //Checks if the 2 meter raycast from the drone at a specific angle hits something 
        {
            return false;
        }
        return true;
    }

    bool CheckDown(float angle, Vector3 origin) //Sends out a raycast at angle below the drone to check for an obstacle. Returns false if there is an obstacle and true otherwise.
    {
        //Debug.Log("Check Down"); //Prints for debugging purposes
        Vector3 vector = new Vector3();
        vector = Quaternion.AngleAxis(-1*angle, transform.right) * transform.forward;
        Debug.DrawRay(origin, vector, Color.yellow, 2f);
        if (Physics.SphereCast(origin, .5f, vector, out RaycastHit hitInfo, 2f)) //(Physics.Raycast(origin, vector, 2f)) //Checks if the 2 meter raycast from the drone at a specific angle hits something 
        {
            return false;
        }
        return true;
    }
*/
}