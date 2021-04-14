using System.Linq;
using UnityEngine;

namespace RRT.RRTStrategies
{
    /// <summary>
    ///     Implementation of the basic RRT* algorithm. This strategy allows to find the shortest possible path when running
    ///     the search long enough.
    ///     This is because the existing path is improved when a better one is found and the tree is restructured throughout
    ///     the search process.
    ///     A new node is added as child to the parent node (which is within the given radius) which enables to reach the new
    ///     node with the lowes cost possible.
    ///     The tree is also restructured if neighbour nodes within the given radius are better reachable trough the new node.
    /// </summary>
    public class RRTStar : RRTSearchStrategy
    {
        /// <summary>
        ///     The radius within which neighbours are searched around the new node
        /// </summary>
        private readonly float _radius;

        /// <summary>
        ///     Constructor for the RRT* algorithm.
        ///     This strategy allows to find the shortest possible path when running the search long enough.
        ///     This is because the existing path is improved when a better one is found and the tree is restructured throughout
        ///     the search process.
        /// </summary>
        /// <param name="rrtConfig">Base RRT configuration</param>
        /// <param name="radius">radius in which a shorter possible parent nodes will be searched for each node</param>
        public RRTStar(RRTConfig rrtConfig, float radius = 1) : base(rrtConfig)
        {
            _radius = radius;
        }

        /// <inheritdoc />
        public override Node AddOneNodeToTree()
        {
            var randomPos = GetRandomPosition();
            return AddOneNodeToTreeAtPosition(randomPos);
        }

        /// <inheritdoc />
        protected override Node AddOneNodeToTreeAtPosition(Vector3 position)
        {
            var closestNode = Config.Tree.GetClosestNode(position);

            var direction = position - closestNode.Position;
            var directionMagnitude = direction.magnitude;
            if (directionMagnitude == 0) //the random point already exists in the tree, hence don't add a new one
                return null;
            var lengthMultiplier = Config.MAXBranchLength < directionMagnitude
                ? Config.MAXBranchLength
                : directionMagnitude;
            var newNode = new Node(closestNode.Position + direction / directionMagnitude * lengthMultiplier);

            if (IsNotCollidingWithObstacle(closestNode.Position, direction, lengthMultiplier))
            {
                var minNode = closestNode;
                var minCost = minNode.Cost + (newNode.Position - minNode.Position).magnitude;
                var neighbourNodes = Tree.GetNeighboursInRadius(newNode, _radius);

                // Find closest parent cost wise (within the given radius around the new node).
                // The current minNode which is closest to the newNode might not be the shortest path to reach the new node from the start node, considering the path cost.
                // Hence the new node is added as child to the neighbour node which offers the shortest possible path to the new node, starting from the root node.
                foreach (var n in neighbourNodes)
                {
                    var dir = newNode.Position - n.Position;
                    var dirMagnitude = dir.magnitude;
                    if (n.Cost + dirMagnitude < minCost && IsNotCollidingWithObstacle(n.Position, dir, dirMagnitude))
                    {
                        minNode = n;
                        minCost = n.Cost + dirMagnitude;
                    }
                }

                Tree.AddChildNodeToParentNodeWithCost(minNode, newNode);

                // Check if neighbours cost might be lower traversing through the new node, if so restructure the tree, by making the neighbour node a child of the new node.
                foreach (var node in
                    from n in neighbourNodes
                    let dir = n.Position - newNode.Position
                    let dirMagnitude = dir.magnitude
                    where newNode.Cost + dirMagnitude < n.Cost &&
                          IsNotCollidingWithObstacle(newNode.Position, dir, dirMagnitude)
                    select n)
                {
                    // make the neighbour node child of the new node
                    node.Parent.RemoveChild(node);
                    Tree.AddChildNodeToParentNodeWithCost(newNode, node);
                }

                var dir2 = newNode.Position - minNode.Position;
                if (IsCollidingWithTarget(minNode.Position, dir2, dir2.magnitude) && !Tree.HasFoundPath)
                {
                    Debug.Log("Found Path");
                    Tree.TargetNode = newNode.Position == PosTarget ? newNode : AddOneNodeToTreeAtPosition(PosTarget);
                }

                return newNode;
            }

            return null;
        }
    }
}