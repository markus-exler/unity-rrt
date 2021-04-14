using UnityEngine;

namespace RRT.RRTStrategies
{
    /// <summary>
    ///     RRT search strategy interface to build and run RRT strategies
    /// </summary>
    public abstract class RRTSearchStrategy
    {
        /// <summary>
        ///     RRT base config
        /// </summary>
        protected readonly RRTConfig Config;

        /// <summary>
        ///     Position of the target game object in the scene
        /// </summary>
        protected readonly Vector3 PosTarget;

        /// <summary>
        ///     Tree used to store and manipulate the RRT
        /// </summary>
        protected readonly Tree Tree;

        /// <summary>
        ///     Base constructor
        /// </summary>
        /// <param name="rrtConfig">Base RRT configuration</param>
        protected RRTSearchStrategy(RRTConfig rrtConfig)
        {
            Config = rrtConfig;
            PosTarget = Config.GOTarget.transform.position;
            Tree = Config.Tree;
        }

        /// <summary>
        ///     Initiates adding a new node to the search tree.
        ///     Where the new node will be added or where it is placed in the 3D space depends on the concrete RRTSearchStrategy.
        ///     This function mainly serves the purpose of determining the new node postion according to the strategy and then
        ///     calling
        ///     AddOneNodeToTreeAtPosition for actually adding a new node.
        /// </summary>
        /// <returns>returns the added node or null if no node was added</returns>
        public abstract Node AddOneNodeToTree();

        /// <summary>
        ///     Adds a new node to the tree at the given position, if the new node position is reachable.
        ///     Otherwise no node is added and null is returned.
        /// </summary>
        /// <param name="position"> position at which the new node should be placed</param>
        /// <returns>return the added node or null if no node was added</returns>
        protected abstract Node AddOneNodeToTreeAtPosition(Vector3 position);

        /// <summary>
        ///     Returns a random postion in the 3D space which is within the given x,y,z MaxMin Values
        ///     If the Tree is only supposed to search in 2D space ( RRTConfig.OnlySearch2DSpace == true) the z variable will
        ///     always be RRT.zOffset
        /// </summary>
        /// <returns>random position in the 3D space which is within the given x,y,z MaxMin Values</returns>
        protected Vector3 GetRandomPosition()
        {
            return new Vector3(Random.Range(Config.SearchAreaMin.x, Config.SearchAreaMax.x),
                Random.Range(Config.SearchAreaMin.y, Config.SearchAreaMax.y),
                Config.OnlySearch2D
                    ? Config.Tree.RootNode.Position.z
                    : Random.Range(Config.SearchAreaMin.z, Config.SearchAreaMax.z));
        }

        /// <summary>
        ///     Checks whether the the direct line (raycast) from posStart with the given direction and directionMagnitude does not
        ///     collide with an obstacle.
        ///     If it collides false is returned, if it does not collide, true is returned.
        /// </summary>
        /// <param name="posStart"> start position from which the raycast is "shot"</param>
        /// <param name="direction"> direction in which the raycast is "shot"</param>
        /// <param name="directionMagnitude"> the length of the raycast</param>
        /// <returns>
        ///     whether the the direct line (raycast) from posStart with the given direction and directionMagnitude does not
        ///     collide with an obstacle
        /// </returns>
        protected bool IsNotCollidingWithObstacle(Vector3 posStart, Vector3 direction, float directionMagnitude)
        {
            var isColliding = Physics.Raycast(posStart, direction, out var hit, directionMagnitude,
                Config.ObstacleLayerMask);
            return !isColliding || hit.collider.gameObject.Equals(Config.GOTarget);
        }

        /// <summary>
        ///     Checks whether the the direct line (raycast) from posStart with the given direction and directionMagnitude does
        ///     collide with the target GameObject.
        ///     If it collides true is returned, if it does not collide, false is returned.
        /// </summary>
        /// <param name="posStart"> start position from which the raycast is "shot"</param>
        /// <param name="direction"> direction in which the raycast is "shot"</param>
        /// <param name="directionMagnitude"> the length of the raycast</param>
        /// <returns>
        ///     whether the the direct line (raycast) from posStart with the given direction and directionMagnitude does
        ///     collide with the target GameObject
        /// </returns>
        protected bool IsCollidingWithTarget(Vector3 posStart, Vector3 direction, float directionMagnitude)
        {
            var isColliding = Physics.Raycast(posStart, direction, out var hit, directionMagnitude);
            return isColliding && hit.collider.gameObject.Equals(Config.GOTarget);
        }
    }
}