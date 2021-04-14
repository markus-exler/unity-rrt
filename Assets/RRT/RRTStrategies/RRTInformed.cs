namespace RRT.RRTStrategies
{
	/// <summary>
	///     Implementation of the informed RRT search algorithm. This uses a search bias which can be set in the
	///     constructor.
	///     Every Xth random position for the new node is not random, but placed at the position of the target, which results
	///     in faster
	///     convergence.
	/// </summary>
	public class RRTInformed : RRTBasic
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
        ///     Constructor for the Informed RRT algorithm. It uses a target bias. This target bias indicates how often the random
        ///     point shouldn't be random, but placed at the postion of the target.
        ///     So if targetBias = 10, every 10th random position will be at the position of the target, hence not random.
        ///     Using a targetBias has the advantage of faster overall search convergence.
        /// </summary>
        /// <param name="rrtConfig">Base RRT configuration</param>
        /// <param name="targetBias">
        ///     This number indicated how often the random point shouldn't be random, but placed at the
        ///     postion of the target. So if targetBias = 10, every 10th random position will be at the position of the target,
        ///     hence not random for this case.
        /// </param>
        public RRTInformed(RRTConfig rrtConfig, int targetBias = 20) : base(rrtConfig)
        {
            TargetBias = targetBias;
        }

        /// <inheritdoc />
        public override Node AddOneNodeToTree()
        {
            //Set the random pos to the targetPosition and change it to a random position if the _executionCount is dividable by the TargetBias
            // resulting in every Xth (_targetBias) execution placed the random point at the target position.
            var randomPos = PosTarget;
            if (ExecutionCounter % TargetBias != 0)
                randomPos = GetRandomPosition();
            ExecutionCounter++;

            return AddOneNodeToTreeAtPosition(randomPos);
        }
    }
}