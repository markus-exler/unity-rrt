using UnityEngine;

namespace RRT.RRTStrategies
{
    /// <summary>
    ///     This is an adapted version of the RRTStartInformedPruning strategy.
    ///     In addition, after a path is found, only points which could improve the found path are added to the tree. Hence
    ///     these random points lay within a ellipse along the path. This is often also referred to as the informed RRT*
    ///     This is an implementation of a technique proposed in this research paper
    ///     https://www.ri.cmu.edu/pub_files/2014/9/TR-2013-JDG003.pdf
    ///     But here the tree is also pruned, to reduce the tree size, which improves the performance for searching neighbour
    ///     nodes in the tree structure.
    /// </summary>
    public class RRTStarInformedPruningEllipse : RRTStarInformedPruning
    {
        /// <summary>
        ///     Constructor for the Ellipse Pruning Informed RRT* algorithm. Which is an adapted version of the
        ///     RRTStartInformedPruning search.
        ///     The difference is, after a path is found, only points which could improve the found path are added to the tree.
        ///     Hence these random points lay within a
        ///     ellipse along the path.
        /// </summary>
        /// <param name="rrtConfig">Base RRT configuration</param>
        /// <param name="radius">radius in which a shorter possible parent nodes will be searched for each node</param>
        /// <param name="targetBias">
        ///     This number indicated how often the random point shouldn't be random, but placed at the
        ///     postion of the target.So if targetBias = 10, every 10th random position will be at the position of the target,
        ///     hence not random for this case.
        /// </param>
        public RRTStarInformedPruningEllipse(RRTConfig rrtConfig, float radius = 1, int targetBias = 20) : base(
            rrtConfig, radius, targetBias)
        {
        }

        /// <inheritdoc />
        public override Node AddOneNodeToTree()
        {
            //Set the random pos to the targetPosition and change it to a random position if the _executionCount is dividable by the _targetBias
            // resulting in every Xth (_targetBias) execution placed the random point at the target position.
            var position = PosTarget;
            if (Tree.HasFoundPath)
                //find a random point within the ellipse
                do
                {
                    position = GetRandomPosition();
                } while (!IsPointInEllipse(position));
            else if (ExecutionCounter % TargetBias != 0) position = GetRandomPosition();

            ExecutionCounter++;
            return AddOneNodeToTreeAtPosition(position);
        }

        /// <summary>
        ///     Checks whether the random point is within the ellipse constraint.
        ///     Therefore the distance from the point to the TargetNode + the distance from the point to the root node must be less
        ///     or equal than the current minimum path length.
        /// </summary>
        /// <param name="point">point to check whether it is within the ellipse</param>
        /// <returns> true if the given point lays within the ellipse, else false</returns>
        private bool IsPointInEllipse(Vector3 point)
        {
            return (point - Tree.RootNode.Position).magnitude + (point - Tree.TargetNode.Position).magnitude <=
                   Tree.TargetNode.Cost;
        }
    }
}