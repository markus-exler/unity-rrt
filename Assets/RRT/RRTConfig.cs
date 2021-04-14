using UnityEngine;

namespace RRT
{
    /// <summary>
    ///     Configuration class which contains all the base parameters required to run the RRT search.
    ///     Used to pass the configuration to the RRT strategy more easily.
    /// </summary>
    public class RRTConfig
    {
        /// <summary>
        ///     Constructor for the configuration class which contains all the base parameters required to run the RRT search.
        ///     Used to pass the configuration to the RRT strategy more easily.
        /// </summary>
        /// <param name="treeRRT">Tree which represents the RRT</param>
        /// <param name="searchAreaMax">Position of the edge (maximum) which defines the upper limit of the search area</param>
        /// <param name="searchAreaMin">Position of the edge (minimum) which defines the lower limit of the search area.</param>
        /// <param name="obstacleLayerMask">Layer which contains the obstacles the RRT should avoid</param>
        /// <param name="goTarget">The target game object</param>
        /// <param name="maxBranchLength">The maximum branch length of the RRT</param>
        /// <param name="onlySearch2D">Boolean, whether only the search should only performed in 2D (true)</param>
        public RRTConfig(Tree treeRRT, Vector3 searchAreaMax, Vector3 searchAreaMin, LayerMask obstacleLayerMask,
            GameObject goTarget, float maxBranchLength, bool onlySearch2D)
        {
            Tree = treeRRT;
            SearchAreaMax = searchAreaMax;
            SearchAreaMin = searchAreaMin;
            ObstacleLayerMask = obstacleLayerMask;
            GOTarget = goTarget;
            MAXBranchLength = maxBranchLength;
            OnlySearch2D = onlySearch2D;
        }

        /// <summary>
        ///     Tree which represents the RRT
        /// </summary>
        public Tree Tree { get; }

        /// <summary>
        ///     Position of the edge (maximum) which defines the upper limit of the search area
        /// </summary>
        public Vector3 SearchAreaMax { get; }

        /// <summary>
        ///     Position of the edge (minimum) which defines the lower limit of the search area
        /// </summary>
        public Vector3 SearchAreaMin { get; }

        /// <summary>
        ///     Layer which contains the obstacles the RRT should avoid
        /// </summary>
        public LayerMask ObstacleLayerMask { get; }

        /// <summary>
        ///     The target game object
        /// </summary>
        public GameObject GOTarget { get; }

        /// <summary>
        ///     The maximum branch length of the RRT
        /// </summary>
        public float MAXBranchLength { get; }

        /// <summary>
        ///     Boolean, whether only the search should only performed in 2D (true)
        /// </summary>
        public bool OnlySearch2D { get; }
    }
}