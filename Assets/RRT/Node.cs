using System.Collections.Generic;
using UnityEngine;

namespace RRT
{
    public class Node
    {
        /// <summary>
        ///     Nodes which are children of this node in the tree structure, hence are lower in the sub direct subtree
        /// </summary>
        private readonly List<Node> _children = new List<Node>();

        /// <summary>
        ///     Distance between this nodes position and its parents postion
        /// </summary>
        private float _distanceToParent;


        /// <summary>
        ///     Counts the amount of failed attempts to reach a possible new node from this node.
        ///     This is used to eliminate possible local minima, by the strategies with 'WithReduction'.
        /// </summary>
        private int _failureCount;

        /// <summary>
        ///     Cost from the root to this node.
        ///     Is the sum of all edge distances connecting the start node to this node
        ///     This is only calculated when using the withCost functions when adding nodes, otherwise 0 is returned.
        /// </summary>
        public float Cost;

        /// <summary>
        ///     Node Constructor to create node at the given postion
        /// </summary>
        /// <param name="pos"> Position of node in space</param>
        public Node(Vector3 pos)
        {
            Position = pos;
        }

        /// <summary>
        ///     Position of the node in 3D Space
        /// </summary>
        public Vector3 Position { get; }

        /// <summary>
        ///     Node which is parent to this node in the tree structure
        /// </summary>
        public Node Parent { get; private set; }

        /// <summary>
        ///     Returns the Node which is closest to the given target node. This can be a child node or the node itself.
        ///     The closes node is searched recursively, as every child node returns the closest node of it's child nodes.
        /// </summary>
        /// <param name="target"> Target node, which the node should be closest to</param>
        /// <returns>Node closest to the given target node</returns>
        public Node GetClosestNodeInChildren(Vector3 target)
        {
            //use square magnitude since it is sufficient for length comparision but more performant since it skips calculating the square root of the magnitude
            var closestSqrDistance = (Position - target).sqrMagnitude;
            var closestNode = this;
            foreach (var child in _children)
            {
                var tempNode = child.GetClosestNodeInChildren(target);
                var tempSqrDistance = (tempNode.Position - target).sqrMagnitude;
                if (tempSqrDistance < closestSqrDistance)
                {
                    closestSqrDistance = tempSqrDistance;
                    closestNode = tempNode;
                }
            }

            return closestNode;
        }

        /// <summary>
        ///     Adds a child-node to this node. And set the parent node of the child node to this node
        /// </summary>
        /// <param name="child">Node which should be added as a child to this node</param>
        public void AddChild(Node child)
        {
            child.Parent = this;
            _children.Add(child);
        }

        /// <summary>
        ///     Adds a child-node to this node. And set the parent node of the child node to this node.
        ///     This function includes calculating the path cost to this node.
        ///     This is required for all algorithms which rely on the path cost, like RRT*.
        /// </summary>
        /// <param name="child">Node which should be added as a child to this node</param>
        public void AddChildWithCost(Node child)
        {
            child.Parent = this;
            child._distanceToParent = (Position - child.Position).magnitude;
            child.Cost = child._distanceToParent + Cost;
            child.UpdateChildCosts();
            _children.Add(child);
        }

        /// <summary>
        ///     Returns all nodes in the subtree which are within the given radius around the given node
        /// </summary>
        /// <param name="newNode"> act as centre of the search area (sphere) with the given radius</param>
        /// <param name="radius"> the radius of the search area (sphere)</param>
        /// <returns>List of all nodes in the subtree which are within the given radius around the given node</returns>
        public List<Node> GetNeighboursWithinRadius(Node newNode, float radius)
        {
            return GetNeighboursInRadiusSqr(newNode, radius * radius);
        }

        /// <summary>
        ///     Returns all nodes in the subtree which are within the given radiusSqr around the given node.
        ///     The search for neighbour nodes is performed recursively, searching the whole subtree.
        /// </summary>
        /// <param name="newNode"> act as centre of the search area (sphere) with the given radius</param>
        /// <param name="radiusSqr"> the squared radius of the search area (sphere)</param>
        /// <returns>List of all nodes in the subtree which are within the given squared radius around the given node</returns>
        private List<Node> GetNeighboursInRadiusSqr(Node newNode, float radiusSqr)
        {
            //use square magnitude since it is sufficient for length comparision but more performant since it skips calculating the square root of the magnitude
            var closeNeighbourNodes = new List<Node>();

            // add this node if it's within the radius
            if ((Position - newNode.Position).sqrMagnitude < radiusSqr)
                closeNeighbourNodes.Add(this);

            // search subtree
            foreach (var child in _children)
                closeNeighbourNodes.AddRange(child.GetNeighboursInRadiusSqr(newNode, radiusSqr));
            return closeNeighbourNodes;
        }

        /// <summary>
        ///     Updated the Cost of every child node since the parent cost has changed
        /// </summary>
        private void UpdateChildCosts()
        {
            foreach (var child in _children)
            {
                child.Cost = child._distanceToParent + Cost;
                child.UpdateChildCosts();
            }
        }

        /// <summary>
        ///     Prunes the tree by removing all children where the combined distance to the end and start position
        ///     is higher then the currently shortest found path. As there can't be a path traversing these nodes which is shorter.
        /// </summary>
        /// <param name="posStart">postion where the found path starts</param>
        /// <param name="posEnd"> postion where the found path ends</param>
        /// <param name="minPathCost"> the cost of the currently shortest path. Should be the unweighted length</param>
        public void PruneNodes(Vector3 posStart, Vector3 posEnd, float minPathCost)
        {
            //actual pruning
            _children.RemoveAll(c => (c.Position - posEnd).magnitude + (c.Position - posStart).magnitude > minPathCost);

            //prune recursively
            foreach (var child in _children)
                child.PruneNodes(posStart, posEnd, minPathCost);
        }

        /// <summary>
        ///     Returns a list of line coordinates (start and end position). This can be used to draw the tree with gizmos or other
        ///     techniques.
        ///     The List contains lines from this node to every child node. This function is recursive, so the lines from the
        ///     children are added to the list.
        /// </summary>
        /// <returns>Line coordinates which represent the subtree of this node in 3D space</returns>
        public List<(Vector3, Vector3)> GetLinesToChildren()
        {
            var list = new List<(Vector3, Vector3)>();
            foreach (var child in _children)
            {
                list.Add((Position, child.Position));
                list.AddRange(child.GetLinesToChildren());
            }

            return list;
        }

        /// <summary>
        ///     Removes all children of this node
        /// </summary>
        public void RemoveAllChildNodes()
        {
            _children.Clear();
        }

        /// <summary>
        ///     Removes the given child from this nodes children.
        /// </summary>
        /// <param name="child">Child which should be removed</param>
        /// <returns>true if removal was successful</returns>
        public bool RemoveChild(Node child)
        {
            return _children.Remove(child);
        }

        /// <summary>
        ///     Increments the failure count of this node.
        ///     If the failure count exceeds the maxAllowedFailures, the node is removed from the tree and the failure count of the
        ///     parent node is increased as well.
        /// </summary>
        /// <param name="maxAllowedFailures">maximum amount of failed attempts to add new child node to this node</param>
        public void IncrementFailureCount(int maxAllowedFailures)
        {
            _failureCount++;
            // return if the failureCount is below the limit or if this is the root node
            if (_failureCount <= maxAllowedFailures || Parent == null) return;

            //Debug.Log("Remove Node due to local minima");
            Parent._children.Remove(this);
            Parent.IncrementFailureCount(maxAllowedFailures);
        }
    }
}