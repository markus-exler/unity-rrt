namespace RRT.RRTStrategies
{
    /// <summary>
    ///     Same as the RRT* but with a target bias as long as no path is found, this improves the time until the first
    ///     possible path is found.
    ///     So it's a combination of the RRTStar and the InformedRRT
    /// </summary>
    public class RRTStarInformed : RRTStar
    {
        /// <summary>
        ///     Defines how often the random position shouldn't be random, but the position of the target.
        ///     So if the TargetBias is 10 every 10th random position will be the target position.
        ///     If the TargetBias is 1 every position will be the target position, causing the rtt to explore in a straight line
        ///     towards the target.
        /// </summary>
        protected readonly int TargetBias;

        /// <summary>
        ///     Keeps count of every attempt of adding a new node. Used to control when an random point should ne replaces be the
        ///     target position.
        /// </summary>
        protected int ExecutionCounter;

        /// <summary>
        ///     Constructor for the Informed RRT* algorithm. Which is a combination of the informed RRT with target bias and the
        ///     basic RRT* algorithm.
        /// </summary>
        /// <param name="rrtConfig">Base RRT configuration</param>
        /// <param name="radius">radius in which a shorter possible parent nodes will be searched for each node</param>
        /// <param name="targetBias">
        ///     This number indicated how often the random point shouldn't be random, but placed at the
        ///     postion of the target.So if targetBias = 10, every 10th random position will be at the position of the target,
        ///     hence not random for this case.
        /// </param>
        public RRTStarInformed(RRTConfig rrtConfig, float radius = 1, int targetBias = 20) : base(rrtConfig, radius)
        {
            TargetBias = targetBias;
        }

        /// <inheritdoc />
        public override Node AddOneNodeToTree()
        {
            //Set the random pos to the targetPosition and change it to a random position if the _executionCount is dividable by the _targetBias
            // resulting in every Xth (targetBias) execution placed the random point at the target position.
            var randomPos = PosTarget;
            if (ExecutionCounter % TargetBias != 0 || Tree.HasFoundPath)
                randomPos = GetRandomPosition();
            ExecutionCounter++;

            return AddOneNodeToTreeAtPosition(randomPos);
        }
    }
}