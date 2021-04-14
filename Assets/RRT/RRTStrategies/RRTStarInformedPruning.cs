using UnityEngine;

namespace RRT.RRTStrategies
{
    /// <summary>
    ///     Adapted version of the informed RRT* search strategy.
    ///     It prunes the tree when a path is found, every node and its children is removed,
    ///     where the combined distance to the end and start position is higher then the currently shortest found path.
    ///     Works like the RRTStarInformed, but prunes the tree everytime a new shorter path is found
    /// </summary>
    public class RRTStarInformedPruning : RRTStarInformed
    {
        /// <summary>
        ///     Constructor for the Pruning Informed RRT* algorithm. Which is an adapted version of the informed RRT* search.
        ///     The difference is, that it prunes the tree everytime a new shorter path is found.
        /// </summary>
        /// <param name="rrtConfig">Base RRT configuration</param>
        /// <param name="radius">radius in which a shorter possible parent nodes will be searched for each node</param>
        /// <param name="targetBias">
        ///     This number indicated how often the random point shouldn't be random, but placed at the
        ///     postion of the target.So if targetBias = 10, every 10th random position will be at the position of the target,
        ///     hence not random for this case.
        /// </param>
        public RRTStarInformedPruning(RRTConfig rrtConfig, float radius = 1, int targetBias = 20) : base(rrtConfig,
            radius, targetBias)
        {
        }

        /// <inheritdoc />
        protected override Node AddOneNodeToTreeAtPosition(Vector3 position)
        {
            var bestPathCost = Tree.HasFoundPath ? Tree.TargetNode.Cost : float.MaxValue;
            var newNode = base.AddOneNodeToTreeAtPosition(position);
            if (Tree.HasFoundPath && bestPathCost > Tree.TargetNode.Cost)
            {
                Debug.Log("Shorter Path found with length = " + Tree.TargetNode.Cost);
                // always prune tree if the found path length has improved
                Tree.Prune();
            }

            return newNode;
        }
    }
}