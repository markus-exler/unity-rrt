using System.Collections.Generic;
using UnityEngine;

namespace RRT
{
    /// <summary>
    ///     Tree stat structure used to store and manipulate the RRT. It consists of nodes and allows access to the root and
    ///     target node.
    /// </summary>
    public class Tree
    {
        private Node _targetNode;

        /// <summary>
        ///     Constructor creating the root node at the given position
        /// </summary>
        /// <param name="startPos">
        ///     Position of the root of this search tree. Should be the position of the start postion for path
        ///     finding in nearly all cases
        /// </param>
        public Tree(Vector3 startPos)
        {
            RootNode = new Node(startPos);
        }

        /// <summary>
        ///     Root node of the RRT Tree.
        ///     Should be the at the postion of the start position
        /// </summary>
        public Node RootNode { get; }

        /// <summary>
        ///     Representing whether the Tree currently contains a path from the start to the target
        /// </summary>
        public bool HasFoundPath { get; private set; }

        /// <summary>
        ///     The node which is at the target position
        ///     Can be used to generate the final path. With a recursive bottom up approach by traversing along its parents to the
        ///     root
        /// </summary>
        public Node TargetNode
        {
            get => _targetNode;
            set
            {
                _targetNode = value;
                HasFoundPath = true;
            }
        }

        /// <summary>
        ///     Adds the given to child node to the children of the given parent node in the tree.
        ///     And returns the child Node
        /// </summary>
        /// <param name="parentNode"></param>
        /// <param name="childNode"></param>
        /// <returns></returns>
        public void AddChildNodeToParentNode(Node parentNode, Node childNode)
        {
            parentNode.AddChild(childNode);
        }


        /// <summary>
        ///     Adds the given to child node to the children of the given parent node in the tree.
        ///     And returns the child Node
        ///     This function includes calculating the path cost to this node.
        ///     This is required for all algorithms which rely on the path cost, like RRT*.
        /// </summary>
        /// <param name="parentNode"></param>
        /// <param name="childNode"></param>
        /// <returns>the child node</returns>
        public void AddChildNodeToParentNodeWithCost(Node parentNode, Node childNode)
        {
            parentNode.AddChildWithCost(childNode);
        }

        /// <summary>
        ///     Finds the node in the tree which is closest to the given postion in 3D space and returns that node.
        /// </summary>
        /// <param name="pos">position in 3D space which the searched node should be closest to</param>
        /// <returns>The node in the tree which is closest to the given position</returns>
        public Node GetClosestNode(Vector3 pos)
        {
            return RootNode.GetClosestNodeInChildren(pos);
        }

        /// <summary>
        ///     Returns all nodes in the tree which are within the given radius around the given node
        /// </summary>
        /// <param name="newNode"> act as centre of the search area (sphere) with the given radius</param>
        /// <param name="radius"> the radius of the search area (sphere)</param>
        /// <returns>List of all nodes in the tree which are within the given radius around the given node</returns>
        public List<Node> GetNeighboursInRadius(Node newNode, float radius)
        {
            return RootNode.GetNeighboursWithinRadius(newNode, radius);
        }

        /// <summary>
        ///     Prunes the tree by removing all the nodes in the tree and its children where the combined distance to the end and
        ///     start position
        ///     is higher then the currently shortest found path. As there can't be a path traversing these nodes which is shorter.
        ///     Only works when nodes where added using the function withCost
        /// </summary>
        public void Prune()
        {
            if (_targetNode == null)
                return;

            RootNode.PruneNodes(RootNode.Position, TargetNode.Position, TargetNode.Cost);
        }

        /// <summary>
        ///     Removes all nodes from the tree, except the root node
        /// </summary>
        public void Clear()
        {
            RootNode.RemoveAllChildNodes();
        }
    }
}