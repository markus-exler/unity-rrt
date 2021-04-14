using UnityEngine;

namespace RRT.RRTStrategies
{
    /// <summary>
    ///     Adapted version of the informed RRT, with the goal of improving the search speed in 3D and creating 2D paths in 3D
    ///     environments.
    ///     Works like the normal informed RRT algorithm, but tries to reach the target in a straight line first.
    ///     If reaching the target in a straight line fails, a search for a path on the horizontal and vertical plane defined
    ///     by the connecting line from the start to the goal position is attempted. Until the maximum allowed nodes in the
    ///     plane is reached or a path is found.
    ///     If these two / three attempts had no success, a normal informed RRT strategy is used until the path is found or the
    ///     maximum amount of allowed nodes is reached.
    ///     The planar search faces do not exactly fit the search area constraint, as the search area is defined by a radius in
    ///     these cases. This radius however depends on the set search area.
    /// </summary>
    public class RRTInformedPlanar : RRTInformed
    {
        /// <summary>
        ///     The maximum allowed amount of attempts in trying to add a new node in the horizontal or vertical plane. This
        ///     maximum
        ///     is for each plane individually not summarized
        ///     This maximum is calculated in the constructor and depends on the distance between the target and the end joint, the
        ///     MaxBranchLength and the Factor2DPlaneSearch.
        /// </summary>
        private readonly int _max2DIterationsPerPlane;

        /// <summary>
        ///     Radius used for the creation of the random point on the plane.
        /// </summary>
        private readonly float _maxRadius;

        /// <summary>
        ///     The normal vector of the horizontal plane going along the vector connecting the last joint and the target position.
        ///     Is perpendicular to to vertical plane.
        /// </summary>
        private readonly Vector3 _planeNormalHorizontal;

        /// <summary>
        ///     The normal vector of the vertical plane going along the vector connecting the last joint and the target position.
        ///     Is perpendicular to the horizontal plane.
        /// </summary>
        private readonly Vector3 _planeNormalVertical;

        /// <summary>
        ///     As long as this  is true, the RRT will explore directly towards the goal.
        /// </summary>
        private bool _goStraight = true;

        /// <summary>
        ///     The amount of attempts to add a new node to the horizontal plane.
        /// </summary>
        private int _horizontalPlaneIterations;

        /// <summary>
        ///     Field reserved for holding the new Node which will be created
        /// </summary>
        private Node _newNode;

        /// <summary>
        ///     Field reserved for holding the position at which a new node should be created
        /// </summary>
        private Vector3 _randomPos;

        /// <summary>
        ///     The amount of attempts to add a new node to the vertical plane.
        /// </summary>
        private int _verticalPlaneIterations;

        /// <summary>
        ///     Constructor for the Planar Informed RRT algorithm.
        ///     It uses a more adapted strategy with various attempts:
        ///     1) try to reach the target in a straight line.
        ///     2) If reaching the target in a straight line fails, a search for a path on the vertical and then on the horizontal
        ///     plane, defined by the connecting line from the
        ///     start to the goal position is attempted. Until the maximum allowed nodes in the plane is reached or a path is
        ///     found.
        ///     3) If these two / three attempts had no success, the normal informed RRT strategy is used until a path is found or
        ///     the maximum amount of allowed nodes is reached.
        /// </summary>
        /// <param name="rrtConfig">Base RRT configuration</param>
        /// <param name="targetBias">
        ///     This number indicated how often the random point shouldn't be random, but placed at the
        ///     position of the target.So if targetBias = 10, every 10th random position will be at the position of the target,
        ///     hence
        ///     not random for this case.
        /// </param>
        /// <param name="factor2DSearch">
        ///     influences the amount of attempted nodes added in the vertical ot horizontal plane.
        ///     Should be >1. Increasing the factor increases the amount of nodes in the planar search.
        /// </param>
        public RRTInformedPlanar(RRTConfig rrtConfig, int targetBias, float factor2DSearch = 10.0f) : base(rrtConfig,
            targetBias)
        {
            _planeNormalHorizontal = GetHorizontalPlaneNormalVector(Tree.RootNode.Position, PosTarget);
            _planeNormalVertical = GetVerticalPlaneNormalVector(Tree.RootNode.Position, PosTarget);
            // set the maximum amount of nodes per plane depending on the distance from start to finish
            _max2DIterationsPerPlane =
                (int) ((Tree.RootNode.Position - PosTarget).magnitude / rrtConfig.MAXBranchLength * factor2DSearch);

            _maxRadius = (rrtConfig.SearchAreaMax - rrtConfig.SearchAreaMin).magnitude;
        }

        /// <inheritdoc />
        public override Node AddOneNodeToTree()
        {
            //Set the random pos to the targetPosition and change it to a random position if the ExecutionCount is dividable by the TargetBias
            // resulting in every Xth (TargetBias) execution placed the random point at the target position.
            _randomPos = PosTarget;
            if (!_goStraight && ExecutionCounter % TargetBias != 0)
            {
                //going straight for the target was not successful, so continue with the following logic.
                if (_verticalPlaneIterations < _max2DIterationsPerPlane)
                {
                    //iterate tree only in the vertical 2 dimensional plane
                    if (_verticalPlaneIterations == 0) Tree.Clear();
                    _randomPos = GetRandomPointOnPlane(Tree.RootNode.Position, _planeNormalVertical,
                        _maxRadius);

                    _verticalPlaneIterations++;
                }
                else if (_horizontalPlaneIterations < _max2DIterationsPerPlane)
                {
                    //iterate tree only in the horizontal 2 dimensional plane
                    if (_horizontalPlaneIterations == 0) Tree.Clear();
                    _randomPos = GetRandomPointOnPlane(Tree.RootNode.Position, _planeNormalHorizontal,
                        _maxRadius);

                    _horizontalPlaneIterations++;
                }
                else
                {
                    // use random point if the point shouldn't be biased
                    _randomPos = GetRandomPosition();
                }
            }

            ExecutionCounter++;

            _newNode = AddOneNodeToTreeAtPosition(_randomPos);
            // if a node couldn't be added due to collision, then do not go straight towards the goal anymore.
            _goStraight = _newNode != null && _goStraight;

            return _newNode;
        }

        /// <summary>
        ///     Returns the normal vector of the plane which goes along the vector connecting the start and target position.
        ///     This plane is oriented vertically in space.
        /// </summary>
        /// <param name="start">the start position on the vector along which the plane should be created</param>
        /// <param name="target">the target/end position on the vector along which the plane should be created</param>
        /// <returns>Normal vector of the vertical plane</returns>
        private Vector3 GetVerticalPlaneNormalVector(Vector3 start, Vector3 target)
        {
            //(target- start) and Vector3.up (0,1,0) form a triangle in the vertically oriented plane, hence the cross product is the normal of this plane.
            //Only works if the (start-target) vector doesn't have the direction (0,1,0)
            var normal = Vector3.Cross(target - start, Vector3.up);

            //vector connecting start and target has the direction (0,1,0) => normal to two vectors pointing in the same direction is not possible, so simply return the z axis as a normal
            //As the z axis is perpendicular to the y axis by definition. X axis could also be returned.
            return normal.Equals(Vector3.zero) ? Vector3.forward : normal.normalized;
        }

        /// <summary>
        ///     Returns the normal vector of the plane which goes along the vector connecting the start and target position.
        ///     This plane is oriented horizontally in space. It is perpendicular to the given vector connection the start and
        ///     target and the vertical
        ///     plane
        /// </summary>
        /// <param name="start"> start position</param>
        /// <param name="target"> target position</param>
        /// <returns>Normal vector of the horizontal plane</returns>
        private Vector3 GetHorizontalPlaneNormalVector(Vector3 start, Vector3 target)
        {
            //(target- start) and (target + Vector3.up - start) form a triangle in the vertically oriented plane, hence the cross product is the normal of this plane.
            return Vector3.Cross(target - start, GetVerticalPlaneNormalVector(start, target)).normalized;
        }

        /// <summary>
        ///     Returns a random point on the plane with the given normal vector.
        /// </summary>
        /// <param name="position">Position on the plane in 3D space, acts as anchor</param>
        /// <param name="normal">Normal vector of the plane on which the random point should be</param>
        /// <param name="radius">Radius on which the random point around the position should be created</param>
        /// <returns>Random point on the plane with the given normal vector</returns>
        private Vector3 GetRandomPointOnPlane(Vector3 position, Vector3 normal, float radius)
        {
            Vector3 posRandom;
            do
            {
                posRandom = Vector3.Cross(Random.insideUnitSphere, normal);
            } while (posRandom == Vector3.zero);

            posRandom *= radius;
            posRandom += position;
            return posRandom;
        }
    }
}