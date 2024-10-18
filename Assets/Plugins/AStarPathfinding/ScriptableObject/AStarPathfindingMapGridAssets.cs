using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AmagiSakuya.AstarPathFinding
{
    [System.Serializable]
    public class MapGridInfo
    {
        public Vector3 position;
        public bool isObstacle;
        public int colIndex;
        public int rowIndex;

        public int[] jumpPointsDistace = new int[8];
    }

    [CreateAssetMenu(menuName = "AStarPathfinding/MapGridAssets")]
    public class AStarPathfindingMapGridAssets : ScriptableObject
    {
        public MapGridInfo[] matrix;
        public int cols; //Size
        public int rows; //Size

        public List<Vector2> jumpPoints;

        public MapGridInfo GetGridInfo(int row, int col)
        {
            if (row >= 0 && row < rows && col >= 0 && col < cols)
            {
                return matrix[row * cols + col]; // 使用行列计算索引
            }
            return null; // 返回默认值
        }

        public float GetGridHeight(int row, int col)
        {
            if (row >= 0 && row < rows && col >= 0 && col < cols)
            {
                return matrix[row * cols + col].position.y; // 使用行列计算索引
            }
            return default; // 返回默认值
        }

        public bool IsValidPosition(int x, int y)
        {
            // 检查行列索引是否在网格范围内
            if (x < 0 || x >= cols || y < 0 || y >= rows)
            {
                return false; // 索引超出范围
            }
            return true;
        }
    }
}