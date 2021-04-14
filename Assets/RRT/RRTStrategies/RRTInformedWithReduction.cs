using UnityEngine;

namespace RRT.RRTStrategies
{
    /// <summary>
    ///     Implementation of an adapted informed RRT search algorithm.
    ///     In in this strategy nodes which fail to extend to new nodes frequently are removed from the search tree,in this
    ///     case the failure count of the parent node is also incremented.
    ///     This reduces the risk of local minima and improves the search performance for complex cases.
    ///     Every X random position is not random, but placed at the position of the target, which results in faster
    ///     convergence.
    /// </summary>
    public class RRTInformedWithReduction : RRTInformed
    {
        /// <summary>
        ///     This is the limit of how often trying to add a child node to a node can fail, before removing the node from the
        ///     search tree.
        /// </summary>
        private readonly int _maxAllowedFailures;

        /// <summary>
        ///     Constructor for the Informed RRT with reduction algorithm. It uses a targetBias. This number indicated how often
        ///     the random point shouldn't be random, but placed at the position of the target.
        ///     So if targetBias = 10, every 10th random position will be at the position of the target, hence not random.
        ///     Using a targetBias has the advantage of faster overall convergence
        ///     In in this strategy nodes which fail to extend to new nodes frequently are removed from the search
        ///     tree.
        ///     This reduces the risk of local minima and improves the search performance for complex cases.
        /// </summary>
        /// <param name="rrtConfig">Base RRT configuration</param>
        /// <param name="targetBias">
        ///     This number indicated how often the random point shouldn't be random, but placed at the
        ///     position of the target.So if targetBias = 10, every 10th random position will be at the position of the target,
        ///     hence not random for this case.
        /// </param>
        /// <param name="maxAllowedFailures">
        ///     the limit of how often trying to add a child node to a node can fail, before removing
        ///     the node from the search tree
        /// </param>
        public RRTInformedWithReduction(RRTConfig rrtConfig, int targetBias, int maxAllowedFailures = 3) : base(
            rrtConfig, targetBias)
        {
            _maxAllowedFailures = maxAllowedFailures;
        }

        /// <inheritdoc />
        protected override Node AddOneNodeToTreeAtPosition(Vector3 position)
        {
            var closestNode = Tree.GetClosestNode(position);

            var direction = position - closestNode.Position;
            //if the nodes are at the same position return null as there should be only one node at one position.
            if (direction == Vector3.zero) return null;

            var directionMagnitude = direction.magnitude;
            // create a new node at a position which is not further away from the closest node than MaxRRTBranchLength
            var lengthMultiplier = Config.MAXBranchLength < directionMagnitude
                ? Config.MAXBranchLength
                : directionMagnitude;
            var newNode = new Node(closestNode.Position + direction / directionMagnitude * lengthMultiplier);

            //Check whether the direct way between the closest and new node is collision free.
            if (!IsNotCollidingWithObstacle(closestNode.Position, direction, lengthMultiplier))
            {
                // if the new node is not reachable, increment the failure count of the parent node and return null
                closestNode.IncrementFailureCount(_maxAllowedFailures);
                return null;
            }

            Tree.AddChildNodeToParentNode(closestNode, newNode);
            // If the direct line between the new node and the closest node collides with the target go, set the new node as target node, since the target is found now.
            if (IsCollidingWithTarget(closestNode.Position, direction, lengthMultiplier))
                Tree.TargetNode = newNode;
            return newNode;
        }
    }
}