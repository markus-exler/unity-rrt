using UnityEngine;

namespace RRT.RRTStrategies
{
    /// <summary>
    ///     Implementation of the most basic RRT search algorithm without any optimization.
    ///     Nodes are added to the tree in a completely random manner, hence the position of a new node is always random.
    /// </summary>
    public class RRTBasic : RRTSearchStrategy
    {
        /// <summary>
        ///     Constructor fot the basic RRT strategy.
        ///     Nodes are added to the tree in a completely random manner, hence the position of a new node is always random.
        /// </summary>
        /// <param name="rrtConfig">Base RRT configuration</param>
        public RRTBasic(RRTConfig rrtConfig) : base(rrtConfig)
        {
        }

        /// <inheritdoc />
        public override Node AddOneNodeToTree()
        {
            return AddOneNodeToTreeAtPosition(GetRandomPosition());
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

            //Check whether the direct way between the closest and new node is collision free
            if (!IsNotCollidingWithObstacle(closestNode.Position, direction, lengthMultiplier)) return null;
            Tree.AddChildNodeToParentNode(closestNode, newNode);

            // If the direct line between the new node and the closest node collides with the target go, set the new node as target node, since the target is found now.
            if (IsCollidingWithTarget(closestNode.Position, direction, lengthMultiplier))
                Tree.TargetNode = newNode;
            return newNode;
        }
    }
}