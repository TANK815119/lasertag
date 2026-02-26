using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Policies;
using System.Net;

namespace Rekabsen.AutoDrone
{
    public class VoxelAvoidance : MonoBehaviour
    {
        [SerializeField] private WarheadListener listener;
        [SerializeField] private Transform poi;
        [SerializeField] private float poiRange = 2f;
        [SerializeField] private LayerMask layerMask;
        [SerializeField] private GameObject explosionPrefab;
        [SerializeField] private Transform target;
        [SerializeField] private Transform drone;
        [SerializeField] private int farDepth = 5; //number of octree subdivisions
        [SerializeField] private int closeDepth = 5; //number of octree subdivisions
        [SerializeField] private int maxLoopDepth = 512; //max iterations of A* before giving up
        private int depth = 0;
        [SerializeField] bool showGizmo = false;
        [SerializeField] float proximalDist = 10f;
        [SerializeField] float superProximalDist = 0.25f;
        [SerializeField] float occlusionCostCoefficient = 10f;
        [SerializeField] float occlusionCostFloor = 5f;
        [SerializeField] float occlusionTolerance = 0.05f;
        [SerializeField] float rayCost = 10f;
        [SerializeField] float proximalTick = 2.5f;
        [SerializeField] float distalTick = 15f;
        [SerializeField] float lowVelocityTheshold = 0.1f;
        [SerializeField] float searchAreaCoefficient = 3f;
        [SerializeField] bool canDetonate = false;
        //[SerializeField] int pathDepthLimit = 64;

        private OctreeNode[] leafNodes;
        private List<OctreeNode> pathNodes;
        private OctreeNode root;

        //private List<Collider> droneCols;
        private Rigidbody droneBody;
        [SerializeField] private Collider droneBox;
        private Drone_3DTarget_Scaled droneBasal;
        private bool proximal = false;
        private bool sighted = false;
        private bool detonated = false;
        private float timer = 0f;
        private int unStuckAttempts = 0;

        private void Start()
        {
            depth = farDepth;
            droneBody = droneBox.attachedRigidbody;
            droneBasal = drone.GetComponent<Drone_3DTarget_Scaled>();


            if (listener != null)
            {

                listener.SetPOI(poi);
                listener.OnCollision.AddListener(() => { detonated = true; }); //no cleanup necessary, since entire hierachy destroyed
            }
            else
            {
                Debug.LogWarning("VoxelAvoidance: No warhead listener assigned");
            }
        }

        private void Update()
        {
            //if the poi is null, make a temp one
            AttemptFixPOI();

            //detonates id within radius or stuck
            if (AttemptDetonation()) { return; }

            //multiple techniques to un-stuck if not moving by forcing repath
            AttemptUnstuckProcedure();

            //changes pathing form distal to proximal and back
            AttemptProximityRecalculation();

            //shortcuts pathing if there is a straight lint to POI
            if (AttemptLineOfSightShortCut()) { return; }

            //recalculates pathing periodically
            PathRecalculationTimer();

            //actually recalculates pathing if previous pathing is discarded
            AttemptPathRecalculation();

            //progress the pathing
            if (pathNodes.Count > 1)
            {
                target.position = pathNodes[0].bounds.center;
                target.forward = poi.position - drone.position;
            }
            else
            {
                leafNodes = null;
                pathNodes = null;

                target.position = poi.position;
                target.forward = poi.position - drone.position;
            }

            //stack-based pathfinding
            //move target to top node
            //once drone gets sorta near node, pop
            if (pathNodes != null && pathNodes.Count > 1 && droneBox.bounds.Intersects(pathNodes[0].bounds))
            {
                pathNodes.RemoveAt(0);
            }
        }

        private void AttemptFixPOI()
        {
            if (poi == null)
            {
                GameObject tempPOI = new GameObject("tempPOI");
                tempPOI.name = "tempPOI";
                tempPOI.transform.parent = this.transform;
                tempPOI.transform.localPosition = Vector3.zero;
                poi = tempPOI.transform;
            }
        }

        private bool AttemptDetonation()
        {
            if (Vector3.Distance(drone.position, poi.position) < superProximalDist && canDetonate)
            {
                detonated = true;
            }

            if (detonated)
            {
                Explode();
                droneBasal.SetMaxThrust(0f);
                showGizmo = false;
                Destroy(drone.GetComponent<DecisionRequester>());
                Destroy(droneBasal);
                Destroy(drone.GetComponent<BehaviorParameters>());
                Destroy(this.gameObject);
                return true;
            }

            return false;
        }

        private void AttemptUnstuckProcedure()
        {
            //"stuck" cooldown
            if (droneBody.linearVelocity.magnitude < lowVelocityTheshold)
            {
                if (unStuckAttempts > 5)
                {
                    detonated = true; //hard coded limit for unstuck attempts is 5
                }
                if (timer > distalTick)
                {
                    unStuckAttempts++;
                    leafNodes = null;
                    pathNodes = null;
                    timer = 0f;
                }

                sighted = false;
            }
            else
            {
                unStuckAttempts = 0;
            }
        }

        private void AttemptProximityRecalculation()
        {
            //recalculate once quite close
            if (!proximal && Vector3.Distance(drone.position, poi.position) < proximalDist)
            {
                depth = closeDepth;
                leafNodes = null;
                pathNodes = null;
                proximal = true;
                timer = 0f;
            }

            //allow transition to distal
            if (proximal && Vector3.Distance(drone.position, poi.position) > proximalDist)
            {
                depth = farDepth;
                leafNodes = null;
                pathNodes = null;
                proximal = false;
                timer = 0f;
            }
        }

        private bool AttemptLineOfSightShortCut()
        {
            //just shortcut the whole pathfindign if you can see the poi while proximal
            if (sighted)
            {
                target.position = poi.position;
                target.forward = poi.position - drone.position;
                return true;
            }
            if (proximal && LineOfSight(drone.position, poi.position, true)) //after to avoid stuck & sighted state
            {
                sighted = true;
            }

            return false;
        }

        private void PathRecalculationTimer()
        {
            //periodic path recalculatioins via discarding the path
            timer += Time.deltaTime;
            //float distalCoef = Vector3.Distance(drone.position, poi.position) / 2f * proximalDist;
            if (leafNodes != null)
            {
                float distalCoef = leafNodes[0].bounds.extents.x;
                if ((proximal && timer > proximalTick) || (!proximal && timer > distalTick * distalCoef))
                {
                    leafNodes = null;
                    pathNodes = null;
                    timer = 0f;
                }
            }
        }

        private void AttemptPathRecalculation() //only does something when previous path has been discarded
        {
            if (leafNodes == null)
            {
                //generate all the boudns via octree subdivision
                root = new OctreeNode(InnitialBound(), null);
                root.colliders = Physics.OverlapBox(root.bounds.center, root.bounds.extents);

                GenerateeOctree(depth, root);

                //take the very smallest bounds for nivation algorith,
                //draw em(are used in voxel brute search elsewhere)
                leafNodes = new OctreeNode[CubeCount(depth) - CubeCount(depth - 1)];
                PopulateLeafNodes(root, 0);
            }

            if (pathNodes == null)
            {
                //find nodes that make up path to poi
                OctreeNode startNode = BruteVoxelSearch(drone.position, true);
                OctreeNode endNode = (proximal) ? BruteVoxelSearch(poi.position, true, false, true) : BruteVoxelSearch(poi.position, false, true, true);
                pathNodes = VoxelStar(startNode, endNode);
            }
        }

        // Optimized A* using a Priority Queue.
        private List<OctreeNode> VoxelStar(OctreeNode startNode, OctreeNode endNode)
        {
            if (startNode == null || endNode == null)
            {
                Debug.LogWarning("Invalid start or end node");
                return null;
            }

            PriorityQueue<OctreeNode> openSet = new PriorityQueue<OctreeNode>();
            HashSet<OctreeNode> closedSet = new HashSet<OctreeNode>();

            startNode.gCost = 0f;
            startNode.hCost = GridDistance(startNode, endNode);
            openSet.Enqueue(startNode, startNode.fCost);
            int depth = 0; //depth limit counter

            while (openSet.Count > 0 && depth < maxLoopDepth)
            {
                OctreeNode current = openSet.Dequeue();
                closedSet.Add(current);
                depth++;

                if (current == endNode)
                {
                    return ReconstructPath(current);
                }

                foreach (OctreeNode neighbor in FindSuccessors(current))
                {
                    if (closedSet.Contains(neighbor))
                    {
                        continue;
                    }

                    // Add extra cost if there is occlusion and the direct ray is blocked.
                    float rayExtraCost = (neighbor.occlusion > 0f || current.occlusion > 0f) &&
                                         !LineOfSight(neighbor.bounds.center, current.bounds.center, false)
                                         ? rayCost : 0f;
                    float tentativeGCost = current.gCost + 1f + rayExtraCost;

                    if (tentativeGCost < neighbor.gCost || !openSet.Contains(neighbor))
                    {
                        neighbor.gCost = tentativeGCost;
                        neighbor.hCost = GridDistance(neighbor, endNode);
                        neighbor.connection = current;
                        if (openSet.Contains(neighbor))
                        {
                            openSet.UpdatePriority(neighbor, neighbor.fCost);
                        }
                        else
                        {
                            openSet.Enqueue(neighbor, neighbor.fCost);
                        }
                    }
                }
            }

            Debug.LogWarning("Pathfinding failed to find target");
            return new List<OctreeNode>(closedSet);
        }

        private List<OctreeNode> ReconstructPath(OctreeNode endNode)
        {
            List<OctreeNode> path = new List<OctreeNode>();
            OctreeNode current = endNode;
            int count = 0;
            while (current != null)
            {
                path.Insert(0, current);
                current = current.connection;
                if (++count > maxLoopDepth)
                {
                    Debug.LogError("Path reconstruction depth limit reached");
                    break;
                }
            }
            return path;
        }

        //private bool LineOfSight(Vector3 a, Vector3 b, bool ignoreNearEnd = false, Collider colliderToIgnore = null)
        //{
        //    Vector3 direction = (a - b).normalized;
        //    float distance = Vector3.Distance(a, b);

        //    // If no collider to ignore is specified, use a single Raycast
        //    if (colliderToIgnore == null && !ignoreNearEnd)
        //    {
        //        return !Physics.Raycast(b, direction, distance);
        //    }

        //    // Use RaycastAll to get all hits
        //    RaycastHit[] hits = Physics.RaycastAll(b, direction, distance);

        //    // Check each hit; if any hit is not the ignored collider, there's an obstruction
        //    foreach (RaycastHit hit in hits)
        //    {
        //        if (colliderToIgnore != null && hit.collider.Equals(colliderToIgnore))
        //        {
        //            return false; // Obstruction found, ignoring the specified collider
        //        }
        //        if(ignoreNearEnd && Vector3.Distance(b, hit.collider.bounds.center) > 0.5f)
        //        {
        //            return false; // Obstruction near endpoint found, ignoring the specified collider
        //        }
        //    }
        //    return true; // No obstructions found (other than the ignored collider, if hit)
        //}

        private bool LineOfSight(Vector3 a, Vector3 b, bool ignoreNearEnd = false)
        {
            Vector3 direction = (a - b).normalized;
            float distance = Vector3.Distance(a, b);
            RaycastHit[] hits = Physics.RaycastAll(b, direction, distance, layerMask);
            foreach (RaycastHit hit in hits)
            {
                if (!ignoreNearEnd)
                {
                    return false; // Obstruction found and not the specified colliders
                }
                if (ignoreNearEnd && Vector3.Distance(b, hit.point) > poiRange)
                {
                    Debug.Log("Line Of sight failed at " + hit.collider.gameObject.name, hit.collider.gameObject);
                    return false; // Obstruction near endpoint found, ignoring the specified collider
                }
            }

            //Debug.Log("Line Of sight passed");
            return true;
        }

        private float GridDistance(OctreeNode node, OctreeNode endNode)
        {
            //assumes only surface-traversals(6) are possible
            Vector3 euclidOffset = endNode.bounds.center - node.bounds.center;
            Vector3 gridOffset = euclidOffset / node.bounds.extents.x; //assumes cubes
            return Mathf.Abs(gridOffset.x) + Mathf.Abs(gridOffset.y) + Mathf.Abs(gridOffset.z);

        }

        private IEnumerable<OctreeNode> FindSuccessors(OctreeNode current)
        {
            //calculate the offset for every coordinate(all cubes)
            float voxelOffset = current.bounds.extents.x * 2f;

            // Pre-allocate array for 6 neighbors (one per face)
            List<OctreeNode> successors = new List<OctreeNode>();

            // Current node's center
            Vector3 center = current.bounds.center;

            //find the octree node asociated with each
            //of 6 directions(one for each space)

            // Define the 6 directional offsets
            Vector3[] offsets = new Vector3[]
            {
        Vector3.up,    // Up
        Vector3.down,  // Down
        Vector3.left,  // Left
        Vector3.right, // Right
        Vector3.forward, // Front
        Vector3.back   // Back
            };

            // Check each direction
            foreach (Vector3 offset in offsets)
            {
                Vector3 neighborCenter = center + offset * voxelOffset;

                // Search for the neighbor node
                OctreeNode neighbor = BruteVoxelSearch(neighborCenter);
                if (neighbor != null && !neighbor.Equals(current))
                {
                    //make a deep copy and add to array
                    //successors[index++] = DeepCopyOctreeNode(neighbor);
                    successors.Add(neighbor);
                }
            }

            return successors;
        }

        //private OctreeNode treeVoxelSearch(Vector3 point, OctreeNode currNode)
        //{
        //    //o(m) seach where 
        //}

        private OctreeNode BruteVoxelSearch(Vector3 point, bool droneLineOfSight = false, bool clearVoxel = false, bool ignoreNear = false)
        {
            if (leafNodes.Length == 0) return null;
            Debug.Assert(leafNodes[0] != null);

            OctreeNode closestNode = null;
            float closestDistance = float.MaxValue;

            for (int i = 0; i < leafNodes.Length; i++)
            {
                if (droneLineOfSight)
                {
                    if (!LineOfSight(leafNodes[i].bounds.center, point, ignoreNear)) continue;
                }
                else if (clearVoxel)
                {
                    if (leafNodes[i].occlusion > 0f) continue;
                }

                float currDistance = Vector3.Distance(point, leafNodes[i].bounds.center);
                if (currDistance < closestDistance)
                {
                    closestNode = leafNodes[i];
                    closestDistance = currDistance;
                }
            }

            return closestNode;
        }

        private Bounds InnitialBound()
        {
            //find offset between drone and poi
            //find which axis is largest
            Vector3 offset = poi.position - drone.position;
            float[] maxes = { Mathf.Abs(offset.x), Mathf.Abs(offset.y), Mathf.Abs(offset.z) };
            float scale = Mathf.Max(maxes);


            //generate the bounds from scale/2 and middle of offset
            Vector3 center = poi.position; //drone.position + offset / 2f;
            Vector3 size = new Vector3(proximalDist, proximalDist, proximalDist) * searchAreaCoefficient; //new Vector3(scale, scale, scale) * 2f; 
            Bounds innitBounds = new Bounds(center, size);

            if (!proximal)
            {
                Vector3 median = drone.position + offset / 2f;
                //if(scale * 2f < occlusionCostCoefficient) //@TODO think about using another parameter for this if
                //{
                //    scale = occlusionCostCoefficient / 2f;
                //}
                Vector3 medSize = new Vector3(scale, scale, scale) * searchAreaCoefficient;
                innitBounds = new Bounds(median, medSize);
            }

            return innitBounds;
        }

        private void GenerateeOctree(int divCount, OctreeNode parent)
        {
            //base case
            if (divCount < 1)
            {
                return;
            }

            //generate the subdivided bounds
            Bounds[] subBounds = SubdivideBounds(parent.bounds);

            //turn the 8 bounds into 8 child octree nodes
            OctreeNode[] children = new OctreeNode[8];
            for (int i = 0; i < children.Length; i++)
            {
                children[i] = new OctreeNode(subBounds[i], parent);

                //calcualte bounds overlap
                children[i].colliders = FilterBox(parent.colliders, children[i].bounds);
                children[i].occlusion = CalculateOcclusion(children[i].colliders, children[i].bounds);
                children[i].occlusionCostCoefficient = occlusionCostCoefficient;
                children[i].occlusionCostFloor = occlusionCostFloor;
                children[i].occlusionTolerance = occlusionTolerance;
            }
            parent.children = children;

            //subdivide each of the bounds into 8 more bounds
            for (int i = 0; i < children.Length; i++)
            {
                GenerateeOctree(divCount - 1, children[i]);
            }
        }

        private Collider[] FilterBox(Collider[] parentColliders, Bounds bounds)
        {
            //loop through all colliders and check for occlusion
            List<Collider> overlapping = new List<Collider>();
            foreach (Collider collider in parentColliders)
            {
                if (bounds.Intersects(collider.bounds))
                {
                    overlapping.Add(collider);
                }
            }

            return overlapping.ToArray();
        }

        private float CalculateOcclusion(Collider[] colliders, Bounds bounds)
        {
            //calculat how much of a bound's volume is occluded by colliders within it
            float boundsVolume = (bounds.extents * 2f).x * (bounds.extents * 2f).y * (bounds.extents * 2f).z;
            float occludedVolume = 0f;
            foreach (Collider collider in colliders)
            {
                occludedVolume += IntersectVolume(bounds, collider.bounds);
            }

            return occludedVolume / boundsVolume;
        }

        private float IntersectVolume(Bounds a, Bounds b) //assumes they intersect
        {
            // Get min/max of intersection
            Vector3 min = Vector3.Max(a.min, b.min);
            Vector3 max = Vector3.Min(a.max, b.max);

            // Volume = width * height * depth
            return (max.x - min.x) * (max.y - min.y) * (max.z - min.z);
        }

        private Bounds[] SubdivideBounds(Bounds bound)
        {
            //octree array
            Bounds[] octBounds = new Bounds[8];

            // Calculate the size for all 8 cubes (half of the original in each dimension)
            Vector3 subSize = bound.size * 0.5f;

            // Calculate the offset for positioning sub-cubes (quarter of the original size)
            Vector3 offset = subSize * 0.5f;

            // Original center for reference
            Vector3 center = bound.center;

            // Find the center of each sub-cube in the octree and create bounds
            // Octree order: 
            // 0: (-x, -y, -z), 1: (+x, -y, -z), 2: (-x, +y, -z), 3: (+x, +y, -z)
            // 4: (-x, -y, +z), 5: (+x, -y, +z), 6: (-x, +y, +z), 7: (+x, +y, +z)
            octBounds[0] = new Bounds(center + new Vector3(-offset.x, -offset.y, -offset.z), subSize);
            octBounds[1] = new Bounds(center + new Vector3(offset.x, -offset.y, -offset.z), subSize);
            octBounds[2] = new Bounds(center + new Vector3(-offset.x, offset.y, -offset.z), subSize);
            octBounds[3] = new Bounds(center + new Vector3(offset.x, offset.y, -offset.z), subSize);
            octBounds[4] = new Bounds(center + new Vector3(-offset.x, -offset.y, offset.z), subSize);
            octBounds[5] = new Bounds(center + new Vector3(offset.x, -offset.y, offset.z), subSize);
            octBounds[6] = new Bounds(center + new Vector3(-offset.x, offset.y, offset.z), subSize);
            octBounds[7] = new Bounds(center + new Vector3(offset.x, offset.y, offset.z), subSize);

            return octBounds;
        }

        private int PopulateLeafNodes(OctreeNode node, int drawIndex) //i <3 recursion
        {
            Debug.Assert(node != null);
            //leaf base case
            if (node.children == null)
            {
                //add to drawBounds array
                leafNodes[drawIndex] = node;

                //add 1 to drawIndex essentially
                return 1;
            }

            //traverse downwards
            int slotsFilled = 0;
            for (int i = 0; i < node.children.Length; i++)
            {
                slotsFilled += PopulateLeafNodes(node.children[i], drawIndex + slotsFilled);
            }

            return slotsFilled;
        }

        private int CubeCount(int divCount)
        {
            //cubeCount where n subdivisions = 8^0 + 8^1 + 8^2 + ... + 8^n
            //base case
            if (divCount < 1)
            {
                if (divCount == 0)
                {
                    return 1; //return 1 if count is 0
                }
                return 0; //return 0 if count is negative
            }

            int count = (int)Mathf.Pow(8f, (float)divCount);

            count += CubeCount(divCount - 1);

            return count;
        }

        public void SetPOI(Transform poiInput)
        {
            poi = poiInput;
        }

        public void setPOIRange(float range)
        {
            poiRange = range;
        }

        public void detonate()
        {
            detonated = true;
        }

        private void Explode()
        {
            GameObject explosion = Instantiate(explosionPrefab, droneBody.position, Quaternion.identity);
        }

        private void OnDrawGizmos()
        {
            if (sighted)
            {
                return;
            }

            if (showGizmo)
            {
                DrawLeafs();
                DrawPath();
            }
        }

        private void DrawLeafs()
        {
            //make sure there's thit to draw
            if (leafNodes == null)
            {
                return;
            }

            //draw a wireframe cube for every bounds in drawbounds
            for (int i = 0; i < leafNodes.Length; i++)
            {
                //check for null object and break
                if (leafNodes[i] == null)
                {
                    Debug.LogWarning("drawBounds array is not fully filled with bounds");
                    break;
                }

                float red = leafNodes[i].occlusion;
                float green = 0f;
                float blue = 1 - leafNodes[i].occlusion;
                float alpha = Mathf.Sqrt(leafNodes[i].occlusion);

                if (alpha < 0.025f)
                {
                    alpha = 0.025f;
                }

                Gizmos.color = new Color(red, green, blue, alpha);
                Gizmos.DrawWireCube(leafNodes[i].bounds.center, leafNodes[i].bounds.size);
            }
        }

        private void DrawPath()
        {
            //make sure there's thit to draw
            if (pathNodes == null)
            {
                return;
            }

            //draw a wireframe cube for every bounds in drawbounds
            for (int i = 0; i < pathNodes.Count; i++)
            {
                //check for null object and break
                if (pathNodes[i] == null)
                {
                    Debug.LogWarning("drawBounds array is not fully filled with bounds");
                    break;
                }

                Gizmos.color = new Color(1f, 1f, 0f);
                Gizmos.DrawWireCube(pathNodes[i].bounds.center, pathNodes[i].bounds.size);
            }
        }

        // --- Nested Helper Classes ---

        // Octree node class holds bounds, cost values, and connection info.
        private class OctreeNode
        {
            public Bounds bounds;
            public float occlusion = -1f;
            public OctreeNode[] children = null;
            public OctreeNode parent;
            public Collider[] colliders = null;
            public float gCost = float.MaxValue;
            public float hCost = 0f;
            public float rCost = 0f;
            public OctreeNode connection = null;
            public float occlusionCostCoefficient = 10f;
            public float occlusionCostFloor = 5f;
            public float occlusionTolerance = 0.05f;
            public float oCost => occlusion > occlusionTolerance ? occlusion * occlusionCostCoefficient + occlusionCostFloor : occlusion * occlusionCostCoefficient;
            public float fCost => gCost + hCost + oCost + rCost;

            public OctreeNode(Bounds b, OctreeNode parent)
            {
                bounds = b;
                this.parent = parent;
            }
        }

        // A simple priority queue implementation to support efficient A* search.
        private class PriorityQueue<T>
        {
            private List<KeyValuePair<T, float>> elements = new List<KeyValuePair<T, float>>();
            public int Count => elements.Count;

            public void Enqueue(T item, float priority)
            {
                elements.Add(new KeyValuePair<T, float>(item, priority));
            }

            public T Dequeue()
            {
                int bestIndex = 0;
                float bestPriority = elements[0].Value;
                for (int i = 1; i < elements.Count; i++)
                {
                    if (elements[i].Value < bestPriority)
                    {
                        bestPriority = elements[i].Value;
                        bestIndex = i;
                    }
                }
                T bestItem = elements[bestIndex].Key;
                elements.RemoveAt(bestIndex);
                return bestItem;
            }

            public bool Contains(T item)
            {
                foreach (var pair in elements)
                {
                    if (EqualityComparer<T>.Default.Equals(pair.Key, item))
                        return true;
                }
                return false;
            }

            public void UpdatePriority(T item, float newPriority)
            {
                for (int i = 0; i < elements.Count; i++)
                {
                    if (EqualityComparer<T>.Default.Equals(elements[i].Key, item))
                    {
                        elements[i] = new KeyValuePair<T, float>(item, newPriority);
                        break;
                    }
                }
            }
        }
    }
}