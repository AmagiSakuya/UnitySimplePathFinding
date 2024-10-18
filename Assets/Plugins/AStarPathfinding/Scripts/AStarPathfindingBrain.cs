using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;
using UnityEditor;

namespace AmagiSakuya.AstarPathFinding
{
    public class AStarPathfindingBrain : MonoBehaviour
    {
        [Header("网格地图数据")]
        public AStarPathfindingMapGridAssets grid;
        [Header("寻路起始点")]
        public Transform from;
        public Transform to;

        [Header("寻路设置")]
        [Tooltip("是否在Update里执行寻路")] public bool findScriptFromToInUpdate;
        [Tooltip("在Update里执行寻路的间隔帧")] public int frameIntervalInUpdatePathFinding = 30;
        [Tooltip("寻路结果LineRenderer高度偏移")] public float resultPathsHeightOffset = 0;

        [Header("使用A*改良算法JPS")]
        public bool useJPSPathFinding;
        public bool useJPSPlusPathFinding;

        [Header("寻路结果输出")]
        public LineRenderer pathLineRenderer;

        [Header("网格制作")]
        public float gridSize = 1f;
        public BoxCollider coverageBoxTrigger;
        public LayerMask groundLayer;
        public LayerMask itemlayer;
        public Material gridItemMat;
        public bool createObstacleItemOnGenerate;
        public float itemAlpha = 0.1f;

        [Header("Debug")]
        [SerializeField] bool debugConsumptionTime;

        List<MapGridInfo> _path;
        private Thread _pathfindingThread;
        private SynchronizationContext _mainThreadContext;
        // 标记线程是否正在运行
        private bool isThreadRunning = false;

        private void Awake()
        {
            _mainThreadContext = SynchronizationContext.Current;
        }

        void Start()
        {
            ClearPath();
        }
        void OnDestroy()
        {
            ClearPath();
        }

        void Update()
        {
            if (frameIntervalInUpdatePathFinding <= 0)
            {
                FindAndSetPath();
                return;
            }

            if (frameIntervalInUpdatePathFinding > 0 && findScriptFromToInUpdate && Time.frameCount % frameIntervalInUpdatePathFinding == 0)
            {
                FindAndSetPath();
            }
        }

        /// <summary>
        /// 寻路
        /// </summary>
        public void FindAndSetPath()
        {
            if (from == null || to == null)
            {
                Debug.LogError("没有指定 from to");
                return;
            }
            FindAndSetPath(from, to);
        }

        public void FindAndSetPath(Transform m_from, Transform m_to)
        {

            if (_mainThreadContext == null)
            {
                _mainThreadContext = SynchronizationContext.Current;
            }
            // 如果已有线程在运行，直接返回
            if (isThreadRunning)
            {
                Debug.Log("Pathfinding is already running in another thread.");
                return;
            }

            // 找到起点和终点对应的最近网格
            MapGridInfo startNode = GetClosestGridItem(grid, m_from.transform.position);
            MapGridInfo endNode = GetClosestGridItem(grid, m_to.transform.position);

            // 开启新的线程
            _pathfindingThread = new Thread(() =>
            {
                isThreadRunning = true;
                // 记录开始时间
                System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
                stopwatch.Start();
                // 执行路径计算
                List<MapGridInfo> path;

                if (useJPSPathFinding || useJPSPlusPathFinding)
                {
                    path = FindPathJPS(startNode, endNode);
                }
                else
                {
                    path = FindPath(startNode, endNode);
                }

                //Debug.Log(path.Count);
                // 寻路完成后，停止计时
                stopwatch.Stop();
                // 输出耗时
                if (debugConsumptionTime)
                {
                    Debug.Log($"FindPath 耗时: {stopwatch.ElapsedMilliseconds} 毫秒");
                }

                // 在主线程中执行回调，传递路径
                _mainThreadContext.Post(_ =>
                {
                    OnPathFound(path); // 执行主线程回调
                    isThreadRunning = false;
                }, null);
            });

            // 启动线程
            _pathfindingThread.Start();

        }

        /// <summary>
        /// 情况结果
        /// </summary>
        public void ClearPath()
        {
            if (_pathfindingThread != null && _pathfindingThread.IsAlive)
            {
                _pathfindingThread.Abort();
            }
            isThreadRunning = false;
            _path = new List<MapGridInfo>();
            pathLineRenderer.positionCount = 0;
        }

        private void OnPathFound(List<MapGridInfo> path)
        {
            if (path == null)
            {
                pathLineRenderer.positionCount = 0;
            }
            else
            {
                UpdateLineRenderer(path);
            }
        }


        #region A*寻路
        /// <summary>
        /// A*寻路
        /// </summary>
        List<MapGridInfo> FindPath(MapGridInfo startNode, MapGridInfo endNode)
        {
            var gridAssets = grid;
            // 如果起点或终点无效，直接返回空路径
            if (startNode.Equals(default(MapGridInfo)) || endNode.Equals(default(MapGridInfo)))
            {
                return null;
            }

            // A* 寻路
            List<MapGridInfo> openSet = new List<MapGridInfo> { startNode };
            HashSet<MapGridInfo> closedSet = new HashSet<MapGridInfo>();

            Dictionary<MapGridInfo, MapGridInfo> cameFrom = new Dictionary<MapGridInfo, MapGridInfo>();
            Dictionary<MapGridInfo, float> gScore = new Dictionary<MapGridInfo, float>();
            Dictionary<MapGridInfo, float> fScore = new Dictionary<MapGridInfo, float>();

            gScore[startNode] = 0;
            fScore[startNode] = Heuristic(startNode, endNode);

            while (openSet.Count > 0)
            {
                // 从openSet中取出fScore最低的节点
                MapGridInfo current = GetLowestFScoreNode(openSet, fScore);

                if (current.Equals(endNode))
                {
                    return ReconstructPath(cameFrom, current); // 返回到当前节点的路径
                }

                openSet.Remove(current);
                closedSet.Add(current);

                // A*算法 - 遍历当前节点的8个邻居
                foreach (MapGridInfo neighbor in GetNeighbors(gridAssets, current))
                {
                    if (neighbor.isObstacle || closedSet.Contains(neighbor))
                    {
                        continue; // 忽略障碍物或已经访问过的节点
                    }


                    float tentativeGScore = gScore[current] + Vector3.Distance(current.position, neighbor.position);

                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                    else if (tentativeGScore >= gScore[neighbor])
                    {
                        continue; // 不是更好的路径
                    }

                    // 更新最佳路径
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + Heuristic(neighbor, endNode);
                }
            }
            return null;
        }

        /// <summary>
        /// 获取给定位置最近的网格项
        /// </summary>
        private MapGridInfo GetClosestGridItem(AStarPathfindingMapGridAssets gridAssets, Vector3 position)
        {
            //----世界坐标计算索引获取
            Vector2Int Idx = GetClosestGridItemIndex(gridAssets, position);
            return gridAssets.GetGridInfo(Idx.x, Idx.y);
        }

        /// <summary>
        /// 获取给定位置最近的网格项索引
        /// </summary>
        private Vector2Int GetClosestGridItemIndex(AStarPathfindingMapGridAssets gridAssets, Vector3 position)
        {
            //----世界坐标计算索引获取
            // 将传入的世界坐标转换为基于 coverageBox 的局部坐标
            Vector3 localPos = coverageBoxTrigger.transform.InverseTransformPoint(position) - coverageBoxTrigger.center; // 转换为相对坐标                                                                               // 根据网格尺寸（boxSize 和 rows, cols）将局部坐标转换为行列索引                                                                                              // 计算列 (X 轴方向, 从左到右)
            int col = Mathf.FloorToInt((localPos.x + coverageBoxTrigger.size.x / 2) / coverageBoxTrigger.size.x * gridAssets.cols);
            // 计算行 (Z 轴方向, 从上到下)，这里要注意是从上到下，所以减去 boxSize.z / 2 再取负
            int row = Mathf.FloorToInt((coverageBoxTrigger.size.z / 2 - localPos.z) / coverageBoxTrigger.size.z * gridAssets.rows);
            // 确保行列索引在合法范围内
            row = Mathf.Clamp(row, 0, gridAssets.rows - 1);
            col = Mathf.Clamp(col, 0, gridAssets.cols - 1);
            //根据行列索引直接获取对应的 GridInfo
            return new Vector2Int(row, col);
        }

        /// <summary>
        /// 计算启发式距离 (曼哈顿距离或欧几里得距离)
        /// </summary>
        private float Heuristic(MapGridInfo a, MapGridInfo b)
        {
            return Vector3.Distance(a.position, b.position); // 使用欧几里得距离
        }

        /// <summary>
        /// 获取当前节点的邻居节点（8方向）
        /// </summary>
        private List<MapGridInfo> GetNeighbors(AStarPathfindingMapGridAssets gridAssets, MapGridInfo current)
        {
            List<MapGridInfo> neighbors = new List<MapGridInfo>();
            int currentRow = GetRow(gridAssets, current);
            int currentCol = GetCol(gridAssets, current);
            for (int rowOffset = -1; rowOffset <= 1; rowOffset++)
            {
                for (int colOffset = -1; colOffset <= 1; colOffset++)
                {
                    if (rowOffset == 0 && colOffset == 0) continue; // 跳过自己

                    int neighborRow = currentRow + rowOffset;
                    int neighborCol = currentCol + colOffset;

                    MapGridInfo neighbor = gridAssets.GetGridInfo(neighborRow, neighborCol);
                    if (neighbor != null && !neighbor.Equals(default(MapGridInfo))) // 确保邻居合法
                    {
                        neighbors.Add(neighbor);
                    }
                }
            }
            return neighbors;
        }

        /// <summary>
        /// 返回fScore最低的节点
        /// </summary>
        private MapGridInfo GetLowestFScoreNode(List<MapGridInfo> openSet, Dictionary<MapGridInfo, float> fScore)
        {
            MapGridInfo lowest = openSet[0];
            float lowestScore = fScore.ContainsKey(lowest) ? fScore[lowest] : Mathf.Infinity;

            foreach (MapGridInfo node in openSet)
            {
                float score = fScore.ContainsKey(node) ? fScore[node] : Mathf.Infinity;
                if (score < lowestScore)
                {
                    lowest = node;
                    lowestScore = score;
                }
            }

            return lowest;
        }

        /// <summary>
        /// 重建路径
        /// </summary>
        private List<MapGridInfo> ReconstructPath(Dictionary<MapGridInfo, MapGridInfo> cameFrom, MapGridInfo current)
        {
            List<MapGridInfo> totalPath = new List<MapGridInfo> { current };
            while (cameFrom.ContainsKey(current))
            {
                current = cameFrom[current];
                totalPath.Add(current);
            }
            totalPath.Reverse();
            return totalPath;
        }
        #endregion

        #region JPS寻路
        List<MapGridInfo> FindPathJPS(MapGridInfo startNode, MapGridInfo endNode)
        {
            // 如果起点或终点无效，直接返回空路径
            if (startNode.Equals(default(MapGridInfo)) || endNode.Equals(default(MapGridInfo)))
            {
                return null;
            }

            List<MapGridInfo> openSet = new List<MapGridInfo> { startNode };
            HashSet<MapGridInfo> closedSet = new HashSet<MapGridInfo>();

            Dictionary<MapGridInfo, MapGridInfo> cameFrom = new Dictionary<MapGridInfo, MapGridInfo>();
            Dictionary<MapGridInfo, float> gScore = new Dictionary<MapGridInfo, float>();
            Dictionary<MapGridInfo, float> fScore = new Dictionary<MapGridInfo, float>();

            gScore[startNode] = 0;
            fScore[startNode] = Heuristic(startNode, endNode);

            while (openSet.Count > 0)
            {
                // 从openSet中取出fScore最低的节点
                MapGridInfo current = GetLowestFScoreNode(openSet, fScore);

                //如果最低是终点重建路线返回
                if (current.Equals(endNode))
                {
                    return ReconstructPath(cameFrom, current); // 返回到当前节点的路径
                }

                //当前节点移除open加入closed
                openSet.Remove(current);
                closedSet.Add(current);

                // 获取前一个节点
                MapGridInfo previous = cameFrom.ContainsKey(current) ? cameFrom[current] : null;

                //横向寻找跳点 //斜向寻找跳点//若有跳点加入openset
                foreach (Vector2Int direction in GetDirections(current, previous))
                {
                    MapGridInfo jumpPoint;

                    //如果是JSP+寻路
                    if (useJPSPlusPathFinding)
                    {
                        jumpPoint = JumpJPSPlus(current, direction, endNode);
                    }
                    else
                    {
                        jumpPoint = Jump(current, direction, endNode);
                    }

                    if (jumpPoint == null || closedSet.Contains(jumpPoint))
                    {
                        continue;
                    }

                    float tentativeGScore = gScore[current] + Heuristic(current, jumpPoint);

                    if (!openSet.Contains(jumpPoint))
                    {
                        openSet.Add(jumpPoint);
                    }
                    else if (tentativeGScore >= gScore[jumpPoint])
                    {
                        continue; // 不是更好的路径
                    }

                    cameFrom[jumpPoint] = current;
                    gScore[jumpPoint] = tentativeGScore;
                    fScore[jumpPoint] = gScore[jumpPoint] + Heuristic(jumpPoint, endNode);
                }


            }
            return null;
        }

        /// <summary>
        /// JPS 跳跃函数
        /// </summary>
        public MapGridInfo Jump(MapGridInfo current, Vector2Int direction, MapGridInfo endNode)
        {
            MapGridInfo newNode = grid.GetGridInfo(current.rowIndex + direction.y, current.colIndex + direction.x);
            if (newNode == null || newNode.isObstacle)
            {
                return null;
            }

            // 如果到达终点，返回
            if (newNode.Equals(endNode))
            {
                return newNode;
            }

            // 检查是否有强迫邻居
            if (HasForcedNeighbor(newNode, direction))
            {
                return newNode;
            }

            // 对角方向
            if (direction.x != 0 && direction.y != 0)
            {
                if (Jump(newNode, new Vector2Int(direction.x, 0), endNode) != null || Jump(newNode, new Vector2Int(0, direction.y), endNode) != null)
                {
                    return newNode;
                }
            }

            if (newNode.colIndex == current.colIndex && newNode.rowIndex == current.rowIndex)
            {
                return null;
            }

            // 递归跳跃
            return Jump(newNode, direction, endNode);
        }

        /// <summary>
        /// 检查节点是否有强迫邻居
        /// </summary>
        public bool HasForcedNeighbor(MapGridInfo node, Vector2Int direction)
        {
            int col = node.colIndex;
            int row = node.rowIndex;

            // 对角方向检查
            if (direction.x != 0 && direction.y != 0)
            {
                // 左右方向有无强迫邻居
                if (IsValidPositionAndIsObstacle(col - direction.x, row) && IsValidPositionAndIsNotObstacle(col - direction.x, row - direction.y))
                {
                    return true;
                }

                if (IsValidPositionAndIsObstacle(col, row - direction.y) && IsValidPositionAndIsNotObstacle(col - direction.x, row - direction.y))
                {
                    return true;
                }
            }
            // 横向或纵向的强迫邻居检查
            else if (direction.x != 0)
            {
                // 上下方向有无强迫邻居
                bool topObstacle = IsValidPositionAndIsObstacle(col, row - 1);
                bool bottomObstacle = IsValidPositionAndIsObstacle(col, row + 1);

                return (topObstacle && IsValidPositionAndIsNotObstacle(col + direction.x, row - 1)) ||
                        (bottomObstacle && IsValidPositionAndIsNotObstacle(col + direction.x, row + 1));

            }
            else if (direction.y != 0)
            {
                // 左右方向有无强迫邻居
                bool leftObstacle = IsValidPositionAndIsObstacle(col - 1, row);
                bool rightObstacle = IsValidPositionAndIsObstacle(col + 1, row);

                return (leftObstacle && IsValidPositionAndIsNotObstacle(col - 1, row + direction.y)) ||
                       (rightObstacle && IsValidPositionAndIsNotObstacle(col + 1, row + direction.y));
            }

            return false;
        }

        bool IsValidPositionAndIsObstacle(int x, int y)
        {
            return grid.IsValidPosition(x, y) && grid.GetGridInfo(y, x).isObstacle;
        }

        bool IsValidPositionAndIsNotObstacle(int x, int y)
        {
            return grid.IsValidPosition(x, y) && !grid.GetGridInfo(y, x).isObstacle;
        }

        public List<Vector2Int> GetDirections()
        {
            return new List<Vector2Int>
                {
                    new Vector2Int(0, -1), // 上
                    new Vector2Int(1, 0),  // 右
                    new Vector2Int(0, 1),  // 下
                    new Vector2Int(-1, 0), // 左
                    new Vector2Int(1, -1), // 右上
                    new Vector2Int(1, 1),  // 右下
                    new Vector2Int(-1, 1), // 左下
                    new Vector2Int(-1, -1) // 左上
                };
        }

        /// <summary>
        /// 获取自然节点方向
        /// </summary>
        public List<Vector2Int> GetDirections(MapGridInfo current, MapGridInfo previous)
        {
            if (previous == null)
            {
                return GetDirections();
            }
            // 根据 previous 和 current 的坐标差异来确定方向
            int dx = current.colIndex - previous.colIndex;
            int dy = current.rowIndex - previous.rowIndex;
            List<Vector2Int> directions = new List<Vector2Int>();

            // 水平和垂直移动
            if (dx == 0 && dy < -0) // 上
            {
                directions.Add(new Vector2Int(0, -1));  // 上
                directions.Add(new Vector2Int(1, 0)); //  右
                directions.Add(new Vector2Int(-1, 0)); //  左
                directions.Add(new Vector2Int(1, -1));  // 右上
                directions.Add(new Vector2Int(-1, -1)); // 左上
            }
            else if (dx > 0 && dy == 0) // 右
            {
                directions.Add(new Vector2Int(1, 0));   // 右
                directions.Add(new Vector2Int(0, 1));   // 下
                directions.Add(new Vector2Int(0, -1));   // 上
                directions.Add(new Vector2Int(1, 1));   // 右下
                directions.Add(new Vector2Int(1, -1));  // 右上
            }
            else if (dx == 0 && dy > 0) // 下
            {
                directions.Add(new Vector2Int(0, 1));   // 下
                directions.Add(new Vector2Int(-1, 0));   // 左
                directions.Add(new Vector2Int(1, 0));   // 右
                directions.Add(new Vector2Int(1, 1));   // 右下
                directions.Add(new Vector2Int(-1, 1));  // 左下
            }
            else if (dx < 0 && dy == 0) // 左
            {
                directions.Add(new Vector2Int(-1, 0));  // 左
                directions.Add(new Vector2Int(0, -1));  // 上
                directions.Add(new Vector2Int(0, 1));  // 下
                directions.Add(new Vector2Int(-1, 1));  // 左下
                directions.Add(new Vector2Int(-1, -1)); // 左上
            }
            // 对角线移动
            else if (dx > 0 && dy < 0) // 右上
            {
                directions.Add(new Vector2Int(1, 0));   // 右
                directions.Add(new Vector2Int(0, -1));  // 上
                directions.Add(new Vector2Int(1, -1));  // 右上
            }
            else if (dx > 0 && dy > 0) // 右下
            {
                directions.Add(new Vector2Int(1, 0));   // 右
                directions.Add(new Vector2Int(0, 1));   // 下
                directions.Add(new Vector2Int(1, 1));   // 右下
            }
            else if (dx < 0 && dy > 0) // 左下
            {
                directions.Add(new Vector2Int(-1, 0));  // 左
                directions.Add(new Vector2Int(0, 1));   // 下
                directions.Add(new Vector2Int(-1, 1));  // 左下
            }
            else if (dx < 0 && dy < 0) // 左上
            {

                directions.Add(new Vector2Int(-1, 0));  // 左
                directions.Add(new Vector2Int(0, -1));  // 上
                directions.Add(new Vector2Int(-1, -1)); // 左上
            }

            return directions;
        }

        #endregion

        #region JPS+

        MapGridInfo TryGetEndPointInHVDir(MapGridInfo current, Vector2Int direction, MapGridInfo endNode)
        {
            if (direction.x != 0 && direction.y != 0) return null;
            int newCol = current.colIndex + direction.x;
            int newRow = current.rowIndex + direction.y;
            MapGridInfo newNode = grid.GetGridInfo(newRow, newCol);

            if (newNode == null || newNode.isObstacle)
            {
                return null;
            }

            // 如果到达终点，返回终点
            if (newNode.colIndex == endNode.colIndex && newNode.rowIndex == endNode.rowIndex)
            {
                return newNode;
            }

            // 检查跳跃过程中是否继续沿该方向前进
            return TryGetEndPointInHVDir(newNode, direction, endNode);
        }

        MapGridInfo GetPointByDir(MapGridInfo current, Vector2Int direction)
        {

            int idx = IndexByVector2Distance(direction);
            if (current.jumpPointsDistace[idx] > 0)
            {
                int newCol = current.colIndex + direction.x * current.jumpPointsDistace[idx];
                int newRow = current.rowIndex + direction.y * current.jumpPointsDistace[idx];
                return grid.GetGridInfo(newRow, newCol);
            }
            if (current.jumpPointsDistace[idx] == -1)
            {
                int newCol = current.colIndex + direction.x * -current.jumpPointsDistace[idx];
                int newRow = current.rowIndex + direction.y * -current.jumpPointsDistace[idx];
                return grid.GetGridInfo(newRow, newCol);
            }

            return null;
        }

        /// <summary>
        /// JPS+ 跳跃函数
        /// </summary>
        MapGridInfo JumpJPSPlus(MapGridInfo current, Vector2Int direction, MapGridInfo endNode)
        {
            MapGridInfo newNode = grid.GetGridInfo(current.rowIndex + direction.y, current.colIndex + direction.x);
            if (newNode == null || newNode.isObstacle)
            {
                return null;
            }
            // 如果到达终点，返回
            if (newNode.Equals(endNode))
            {
                return newNode;
            }

            if (direction.x == 0 || direction.y == 0)
            {
                return GetPointByDir(current, direction);
            }

            // 对角方向
            if (direction.x != 0 && direction.y != 0)
            {
                if (JumpJPSPlus(newNode, new Vector2Int(direction.x, 0), endNode) != null || JumpJPSPlus(newNode, new Vector2Int(0, direction.y), endNode) != null)
                {
                    return newNode;
                }
            }

            if (newNode.colIndex == current.colIndex && newNode.rowIndex == current.rowIndex)
            {
                return null;
            }

            // 递归跳跃
            return JumpJPSPlus(newNode, direction, endNode);
        }

        /// <summary>
        /// JPS+烘焙
        /// </summary>
        public int JumpBakeDistance(MapGridInfo current, Vector2Int direction)
        {
            //第一步 遍历所有节点找出跳点
            int count = 0;
            bool isObstacle = JumpBakeDistanceMain(current, direction, ref count);
            return isObstacle ? -count : count;
        }

        /// <summary>
        /// JPS+ 获取可达性距离（递归方法）
        /// </summary>
        bool JumpBakeDistanceMain(MapGridInfo current, Vector2Int direction, ref int count)
        {
            var gridAssets = grid;
            int col = current.colIndex;
            int row = current.rowIndex;
            int newCol = col + direction.x;
            int newRow = row + direction.y;

            if (newCol == col && newRow == row)
            {
                return false;
            }
            count++;
            MapGridInfo newNode = gridAssets.GetGridInfo(newRow, newCol);

            if (newNode == null || newNode.isObstacle)
            {
                count--;
                return true;
            }

            // 检测是否是跳点
            Vector2 rowCol = new Vector2(newNode.rowIndex, newNode.colIndex);
            if (gridAssets.jumpPoints.Contains(rowCol))
            {
                return false;
            }

            // 递归跳跃
            return JumpBakeDistanceMain(newNode, direction, ref count);
        }

        public int IndexByVector2Distance(Vector2Int direction)
        {
            if (direction == new Vector2Int(0, -1))
            {
                return 0;
            }
            else if (direction == new Vector2Int(1, 0))
            {
                return 1;
            }
            else if (direction == new Vector2Int(0, 1))
            {
                return 2;
            }
            else if (direction == new Vector2Int(-1, 0))
            {
                return 3;
            }
            else if (direction == new Vector2Int(1, -1))
            {
                return 4;
            }
            else if (direction == new Vector2Int(1, 1))
            {
                return 5;
            }
            else if (direction == new Vector2Int(-1, 1))
            {
                return 6;
            }
            else if (direction == new Vector2Int(-1, -1))
            {
                return 7;
            }
            return -1;
        }

        #endregion

        #region 索引相关
        /// <summary>
        /// 获取gridItem在gridAssets的行数
        /// </summary>
        private int GetRow(AStarPathfindingMapGridAssets gridAssets, MapGridInfo gridItem)
        {
            int index = System.Array.IndexOf(gridAssets.matrix, gridItem);
            if (index != -1)
            {
                return index / gridAssets.cols;  // 行 = 索引 / 列数
            }
            return -1;  // 未找到
        }

        /// <summary>
        /// 获取gridItem在gridAssets的列数
        /// </summary>
        private int GetCol(AStarPathfindingMapGridAssets gridAssets, MapGridInfo gridItem)
        {
            int index = System.Array.IndexOf(gridAssets.matrix, gridItem);
            if (index != -1)
            {
                return index % gridAssets.cols;  // 列 = 索引 % 列数
            }
            return -1;  // 未找到
        }
        #endregion

        /// <summary>
        /// 输出结果
        /// </summary>
        /// <param name="path"></param>
        private void UpdateLineRenderer(List<MapGridInfo> path)
        {
            // 设置 LineRenderer 的点数
            pathLineRenderer.positionCount = path.Count;

            // 将路径点赋值给 LineRenderer
            for (int i = 0; i < path.Count; i++)
            {
                Vector3 _pos = path[i].position;
                if (resultPathsHeightOffset != 0)
                {
                    _pos = new Vector3(_pos.x, _pos.y + resultPathsHeightOffset, _pos.z);
                }
                pathLineRenderer.SetPosition(i, _pos);
            }
        }

    }
}