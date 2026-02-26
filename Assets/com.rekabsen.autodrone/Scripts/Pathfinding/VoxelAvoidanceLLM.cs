using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Rekabsen.AutoDrone
{
    public class VoxelAvoidanceLLM : MonoBehaviour
    {
        [SerializeField] private Transform poi;
        [SerializeField] private Transform target;
        [SerializeField] private Transform drone;
        [SerializeField] private int farDepth = 5; // number of octree subdivisions (distal)
        [SerializeField] private int closeDepth = 5; // number of octree subdivisions (proximal)
        private int depth = 0;
        [SerializeField] bool showGizmo = false;
        [SerializeField] float proximalDist = 10f;
        [SerializeField] float occlusionCostCoefficient = 10f;
        [SerializeField] float occlusionCostFloor = 5f;
        [SerializeField] float occlusionTolerance = 0.05f;
        [SerializeField] float rayCost = 10f;
        [SerializeField] float proximalTick = 2.5f;
        [SerializeField] float distalTick = 15f;
        [SerializeField] float lowVelocityThreshold = 0.1f;

        private OctreeNode[] leafNodes;
        private List<OctreeNode> pathNodes;
        private OctreeNode root;

        [SerializeField] private Collider droneCol;
        private Rigidbody droneBody;
        private bool proximal = false;
        private float timer = 0f;

        private void Start()
        {
            depth = farDepth;
            droneBody = droneCol.attachedRigidbody;
        }

        private void Update()
        {
            // If drone is stuck (low velocity) for a specified time, force regeneration.
            if (droneBody.linearVelocity.magnitude < lowVelocityThreshold)
            {
                if (timer > proximalTick)
                {
                    ResetOctree();
                }
            }

            // Switch octree resolution based on proximity to the point-of-interest (poi).
            bool inProximity = Vector3.Distance(drone.position, poi.position) < proximalDist;
            if (!proximal && inProximity)
            {
                depth = closeDepth;
                ResetOctree();
                proximal = true;
            }
            else if (proximal && !inProximity)
            {
                depth = farDepth;
                ResetOctree();
                proximal = false;
                timer = 0f;
            }

            timer += Time.deltaTime;
            if ((proximal && timer > proximalTick) || (!proximal && timer > distalTick))
            {
                ResetOctree();
            }

            // Generate the octree only once until it is invalidated.
            if (leafNodes == null)
            {
                root = new OctreeNode(GetInitialBounds(), null);
                root.colliders = Physics.OverlapBox(root.bounds.center, root.bounds.extents);
                GenerateOctree(depth, root);
                int leafCount = CubeCount(depth) - CubeCount(depth - 1);
                leafNodes = new OctreeNode[leafCount];
                PopulateLeafNodes(root, 0);
            }

            // Perform A* pathfinding using our optimized method.
            if (pathNodes == null)
            {
                OctreeNode startNode = GetClosestLeafNode(drone.position, requireLineOfSight: true);
                OctreeNode endNode = inProximity
                    ? GetClosestLeafNode(poi.position, requireLineOfSight: true, requireClear: false)
                    : GetClosestLeafNode(poi.position, requireClear: true);
                pathNodes = VoxelStar(startNode, endNode);
            }

            // Update the target position along the computed path.
            if (pathNodes != null && pathNodes.Count > 1)
            {
                target.position = pathNodes[0].bounds.center;
                target.forward = poi.position - drone.position;
            }
            else
            {
                target.position = poi.position;
                target.forward = poi.position - drone.position;
            }

            // Pop off reached nodes to update the target path.
            if (pathNodes != null && pathNodes.Count > 1 && droneCol.bounds.Intersects(pathNodes[0].bounds))
            {
                pathNodes.RemoveAt(0);
            }
        }

        private void ResetOctree()
        {
            leafNodes = null;
            pathNodes = null;
            timer = 0f;
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

            while (openSet.Count > 0)
            {
                OctreeNode current = openSet.Dequeue();
                closedSet.Add(current);

                if (current == endNode)
                {
                    return ReconstructPath(current);
                }

                foreach (OctreeNode neighbor in FindSuccessors(current))
                {
                    if (closedSet.Contains(neighbor))
                        continue;

                    // Add extra cost if there is occlusion and the direct ray is blocked.
                    float rayExtraCost = (neighbor.occlusion > 0f || current.occlusion > 0f) &&
                                         !LineOfSight(neighbor.bounds.center, current.bounds.center, droneCol)
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
                if (++count > 1000)
                {
                    Debug.LogError("Path reconstruction depth limit reached");
                    break;
                }
            }
            return path;
        }

        // Expand the 6 direct neighbors (faces) of the current voxel.
        private IEnumerable<OctreeNode> FindSuccessors(OctreeNode current)
        {
            List<OctreeNode> successors = new List<OctreeNode>();
            float voxelSize = current.bounds.extents.x * 2f; // assume cubes
            Vector3[] directions = { Vector3.up, Vector3.down, Vector3.left, Vector3.right, Vector3.forward, Vector3.back };

            foreach (Vector3 dir in directions)
            {
                Vector3 neighborCenter = current.bounds.center + dir * voxelSize;
                OctreeNode neighbor = GetClosestLeafNode(neighborCenter);
                if (neighbor != null && neighbor != current)
                {
                    successors.Add(neighbor);
                }
            }
            return successors;
        }

        // Consolidated leaf-node search function.
        private OctreeNode GetClosestLeafNode(Vector3 point, bool requireLineOfSight = false, bool requireClear = false)
        {
            if (leafNodes == null || leafNodes.Length == 0)
                return null;

            OctreeNode closest = null;
            float closestDist = float.MaxValue;
            foreach (OctreeNode node in leafNodes)
            {
                if (requireLineOfSight && !LineOfSight(node.bounds.center, point, droneCol))
                    continue;
                if (requireClear && node.occlusion > 0f)
                    continue;

                float dist = Vector3.Distance(point, node.bounds.center);
                if (dist < closestDist)
                {
                    closestDist = dist;
                    closest = node;
                }
            }
            return closest;
        }

        // Simplified line-of-sight test with optional collider ignore.
        private bool LineOfSight(Vector3 a, Vector3 b, Collider colliderToIgnore = null)
        {
            Vector3 direction = (a - b).normalized;
            float distance = Vector3.Distance(a, b);
            RaycastHit[] hits = Physics.RaycastAll(b, direction, distance);
            foreach (RaycastHit hit in hits)
            {
                if (colliderToIgnore == null || !hit.collider.Equals(colliderToIgnore))
                    return false;
            }
            return true;
        }

        // Uses Manhattan distance on grid�aligned offsets.
        private float GridDistance(OctreeNode from, OctreeNode to)
        {
            Vector3 diff = to.bounds.center - from.bounds.center;
            Vector3 gridOffset = diff / from.bounds.extents.x;
            return Mathf.Abs(gridOffset.x) + Mathf.Abs(gridOffset.y) + Mathf.Abs(gridOffset.z);
        }

        // Sets the initial bounding volume for the octree.
        private Bounds GetInitialBounds()
        {
            Vector3 offset = poi.position - drone.position;
            float scale = Mathf.Max(Mathf.Abs(offset.x), Mathf.Abs(offset.y), Mathf.Abs(offset.z));
            Vector3 center;
            Vector3 size;
            if (proximal)
            {
                center = poi.position;
                size = Vector3.one * proximalDist * 2f;
            }
            else
            {
                center = drone.position + offset * 0.5f;
                scale = scale * 2f < occlusionCostCoefficient ? occlusionCostCoefficient / 2f : scale;
                size = Vector3.one * scale * 2f;
            }
            return new Bounds(center, size);
        }

        // Recursively subdivides the parent bounds into 8 octants.
        private void GenerateOctree(int divCount, OctreeNode parent)
        {
            if (divCount < 1)
                return;

            Bounds[] subBounds = SubdivideBounds(parent.bounds);
            parent.children = new OctreeNode[8];

            for (int i = 0; i < 8; i++)
            {
                parent.children[i] = new OctreeNode(subBounds[i], parent);
                parent.children[i].colliders = FilterColliders(parent.colliders, parent.children[i].bounds);
                parent.children[i].occlusion = CalculateOcclusion(parent.children[i].colliders, parent.children[i].bounds);
                parent.children[i].occlusionCostCoefficient = occlusionCostCoefficient;
                parent.children[i].occlusionCostFloor = occlusionCostFloor;
                parent.children[i].occlusionTolerance = occlusionTolerance;
                GenerateOctree(divCount - 1, parent.children[i]);
            }
        }

        // Filter colliders that intersect a given bounds.
        private Collider[] FilterColliders(Collider[] parentColliders, Bounds bounds)
        {
            List<Collider> colliders = new List<Collider>();
            foreach (Collider col in parentColliders)
            {
                if (bounds.Intersects(col.bounds))
                    colliders.Add(col);
            }
            return colliders.ToArray();
        }

        // Compute the fraction of a bound�s volume occluded by colliders.
        private float CalculateOcclusion(Collider[] colliders, Bounds bounds)
        {
            float boundsVolume = bounds.size.x * bounds.size.y * bounds.size.z;
            float occludedVolume = 0f;
            foreach (Collider col in colliders)
            {
                occludedVolume += GetIntersectVolume(bounds, col.bounds);
            }
            return occludedVolume / boundsVolume;
        }

        // Computes the intersecting volume between two bounds.
        private float GetIntersectVolume(Bounds a, Bounds b)
        {
            Vector3 min = Vector3.Max(a.min, b.min);
            Vector3 max = Vector3.Min(a.max, b.max);
            return Mathf.Max(0, max.x - min.x) * Mathf.Max(0, max.y - min.y) * Mathf.Max(0, max.z - min.z);
        }

        // Subdivides a bound into 8 smaller bounds.
        private Bounds[] SubdivideBounds(Bounds bound)
        {
            Bounds[] octBounds = new Bounds[8];
            Vector3 subSize = bound.size * 0.5f;
            Vector3 offset = subSize * 0.5f;
            Vector3 center = bound.center;

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

        // Populates the leafNodes array with the lowest-level nodes (no children).
        private int PopulateLeafNodes(OctreeNode node, int index)
        {
            if (node.children == null)
            {
                leafNodes[index] = node;
                return 1;
            }
            int count = 0;
            foreach (var child in node.children)
            {
                count += PopulateLeafNodes(child, index + count);
            }
            return count;
        }

        // Returns the cumulative cube count for a given subdivision depth.
        private int CubeCount(int divCount)
        {
            if (divCount < 1)
                return divCount == 0 ? 1 : 0;
            return (int)Mathf.Pow(8, divCount) + CubeCount(divCount - 1);
        }

        private void OnDrawGizmos()
        {
            if (!showGizmo)
                return;
            DrawLeafNodes();
            DrawPath();
        }

        private void DrawLeafNodes()
        {
            if (leafNodes == null)
                return;
            foreach (OctreeNode node in leafNodes)
            {
                if (node == null)
                    continue;
                float alpha = Mathf.Max(0.025f, Mathf.Sqrt(node.occlusion));
                Gizmos.color = new Color(node.occlusion, 0f, 1f - node.occlusion, alpha);
                Gizmos.DrawWireCube(node.bounds.center, node.bounds.size);
            }
        }

        private void DrawPath()
        {
            if (pathNodes == null)
                return;
            Gizmos.color = Color.yellow;
            foreach (OctreeNode node in pathNodes)
            {
                if (node == null)
                    continue;
                Gizmos.DrawWireCube(node.bounds.center, node.bounds.size);
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