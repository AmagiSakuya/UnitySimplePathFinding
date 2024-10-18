using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace AmagiSakuya.AstarPathFinding.EditorClass
{
    [CustomEditor(typeof(AStarPathfindingBrain))]
    public class AStarPathfindingBrainEditor : Editor
    {
        private AStarPathfindingBrain t;
        void OnEnable()
        {
            t = (AStarPathfindingBrain)target;

            // 订阅 Scene 视图中的 GUI 事件
            SceneView.duringSceneGui += OnSceneGUI;
        }

        void OnDisable()
        {
            // 取消订阅
            SceneView.duringSceneGui -= OnSceneGUI;
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            GUILayout.Label("------------------网格生成--------------------");

            if (!HasGrid() && GUILayout.Button("创建编辑器网格"))
            {
                GenerateGrid(t);
            }

            if (!HasGrid() && t.grid != null && GUILayout.Button("读取Assets网格"))
            {
                LoadGrid(t);
            }

            if (t.grid != null && GUILayout.Button("预计算JPS+"))
            {
                BakeJumpPoints(t.grid);
            }

            if (HasGrid() && GUILayout.Button("打印编辑器网格数量"))
            {
                Debug.Log(t.coverageBoxTrigger.transform.childCount);
            }

            if (HasGrid() && GUILayout.Button("清空编辑器网格"))
            {
                ClearChildren();
            }

            if (HasGrid() && GUILayout.Button("新建Assets保存网格"))
            {
                SaveGridToScriptableObject(t);
            }

            if (t.grid != null)
            {
                GUILayout.Label("------------------寻路测试--------------------");
                if (GUILayout.Button("寻路测试"))
                {
                    t.ClearPath();
                    t.FindAndSetPath();
                }

                if (GUILayout.Button("清除寻路结果"))
                {
                    t.ClearPath();
                }
            }

            serializedObject.ApplyModifiedProperties();
        }

        #region 地图网格
        void GenerateGrid(AStarPathfindingBrain brain)
        {
            EditorCoroutines.Execute(GenerateGridAsync(brain));
        }

        IEnumerator GenerateGridAsync(AStarPathfindingBrain brain)
        {
            int maxRowCount = -1;
            int maxColCount = -1;

            // 检查是否有子对象并提示是否清空
            ClearChildren();

            // 获取 BoxCollider 的范围
            BoxCollider box = brain.coverageBoxTrigger;

            // 计算 BoxCollider 的真实世界坐标中的中心点
            Vector3 boxSize = box.size;

            // 计算 BoxCollider 的最小和最大边界
            Vector3 boxMin = box.center - boxSize / 2;
            Vector3 boxMax = box.center + boxSize / 2;

            // 计算网格步长
            float step = brain.gridSize;

            // 在 X-Z 平面上遍历
            int rowsCount = -1;
            int colCount;
            int count = 0;
            // 从左上角向右扫描，然后换行
            for (float z = boxMax.z; z >= boxMin.z; z -= step) // 从上到下
            {
                rowsCount++;
                maxRowCount = Mathf.Max(maxRowCount, rowsCount);
                colCount = -1;
                for (float x = boxMin.x; x <= boxMax.x; x += step) // 从左到右
                {
                    colCount++;
                    maxColCount = Mathf.Max(maxColCount, colCount);
                    // 从 BoxCollider 顶部向下发射射线
                    Vector3 startPoint = new Vector3(x, boxMax.y, z);
                    //startPoint  = boxRotation * startPoint + boxWorldCenter;
                    startPoint = box.transform.TransformPoint(startPoint);
                    Ray ray = new Ray(startPoint, Vector3.down);
                    RaycastHit hit;

                    if (Physics.Raycast(ray, out hit, Mathf.Infinity, brain.groundLayer))
                    {
                        // 在碰撞点生成一个 box
                        Vector3 hitPoint = hit.point;
                        var item = CreateCubeItem(hitPoint);
                        item.matrixRow = rowsCount;
                        item.matrixCol = colCount;
                        item.IsObstacle = t.createObstacleItemOnGenerate;
                        item.itemAlpha = t.itemAlpha;
                    }
                    else
                    {
                        Vector3 position = new Vector3(x, box.center.y - (boxSize.y / 2), z);
                        position = box.transform.TransformPoint(position);
                        var item = CreateCubeItem(position);
                        item.matrixRow = rowsCount;
                        item.matrixCol = colCount;
                        item.IsObstacle = true;
                        item.itemAlpha = t.itemAlpha;
                    }

                    count++;
                    //EditorUtility.DisplayProgressBar("网格地图生成", $"正在生成 ({count}/{brain.grid.rows * brain.grid.cols})", (float)count / (brain.grid.rows * brain.grid.cols));

                    if (count % 2000 == 0)
                        yield return null;

                }
            }

            maxRowCount += 1;
            maxColCount += 1;
            //EditorUtility.ClearProgressBar();

            Debug.Log($"地图网络生成完成:{maxRowCount}x{maxColCount}");
        }

        AStarPathfindingGridItem CreateCubeItem(Vector3 pos)
        {
            GameObject boxObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
            boxObject.transform.position = pos;
            boxObject.transform.localScale = new Vector3(t.gridSize, 0.2f, t.gridSize);
            AStarPathfindingGridItem item = boxObject.gameObject.AddComponent<AStarPathfindingGridItem>();
            boxObject.gameObject.layer = (int)Mathf.Log(t.itemlayer.value, 2);
            boxObject.GetComponent<Renderer>().sharedMaterial = new Material(t.gridItemMat);
            // 设置生成的 box 为 coverageBoxTrigger 的子对象
            boxObject.transform.SetParent(t.coverageBoxTrigger.transform);
            boxObject.transform.localEulerAngles = Vector3.zero;
            return item;
        }

        private void ClearChildren()
        {
            if (HasGrid())
            {
                bool confirmClear = EditorUtility.DisplayDialog("清空确认", "覆盖区域内已有子对象，是否清空？", "是", "否");

                if (!confirmClear)
                {
                    return; // 用户选择不清空，直接返回
                }

                for (int i = t.coverageBoxTrigger.transform.childCount - 1; i >= 0; i--)
                {
                    DestroyImmediate(t.coverageBoxTrigger.transform.GetChild(i).gameObject);
                }
            }
        }

        Vector2 GetMaxRowColCount()
        {
            int maxRowCount = -1;
            int maxColCount = -1;

            foreach (Transform child in t.coverageBoxTrigger.transform)
            {
                AStarPathfindingGridItem gridItem = child.GetComponent<AStarPathfindingGridItem>();
                maxRowCount = Mathf.Max(maxRowCount, gridItem.matrixRow);
                maxColCount = Mathf.Max(maxColCount, gridItem.matrixCol);
            }

            maxRowCount += 1;
            maxColCount += 1;
            return new Vector2(maxRowCount, maxColCount);
        }

        // 保存网格高度信息ScriptableObject 障碍信息成图片
        void SaveGridToScriptableObject(AStarPathfindingBrain brain)
        {
            EditorCoroutines.Execute(SaveGridToScriptableObjectAsync(brain));
        }

        IEnumerator SaveGridToScriptableObjectAsync(AStarPathfindingBrain brain)
        {
            // 计算网格的行数和列数
            Vector2 matrixSize = GetMaxRowColCount();
            int gridRows = (int)matrixSize.x;
            int gridCols = (int)matrixSize.y;

            MapGridInfo[] matrix = new MapGridInfo[gridRows * gridCols]; // 创建一维数组
            Transform[] children = brain.coverageBoxTrigger.GetComponentsInChildren<Transform>();

            int totalCount = children.Length;
            int currentIndex = -1;

            // 遍历所有子网格，按位置保存到矩阵
            foreach (Transform child in children)
            {

                AStarPathfindingGridItem gridItem = child.GetComponent<AStarPathfindingGridItem>();
                if (gridItem != null)
                {
                    currentIndex++;
                    // 填充矩阵
                    matrix[gridItem.matrixRow * gridCols + gridItem.matrixCol] = new MapGridInfo // 使用计算的索引填充
                    {
                        position = child.position,
                        isObstacle = gridItem.IsObstacle,
                        rowIndex = gridItem.matrixRow,
                        colIndex = gridItem.matrixCol
                    };
                }

                // 更新进度条
                EditorUtility.DisplayProgressBar("网格保存", $"正在计算 ({currentIndex}/{totalCount})", (float)currentIndex / totalCount);

                if (currentIndex % 2000 == 0)
                    yield return null;
            }

            // 创建 ScriptableObject 并保存
            AStarPathfindingMapGridAssets mapGridAsset = ScriptableObject.CreateInstance<AStarPathfindingMapGridAssets>();
            mapGridAsset.matrix = matrix; // 一维数组
            mapGridAsset.rows = gridRows; // 保存行数
            mapGridAsset.cols = gridCols; // 保存列数

            // 打开保存面板让用户选择路径
            string path = EditorUtility.SaveFilePanelInProject("保存网格", "MapGridAsset", "asset", "选择保存位置");
            if (!string.IsNullOrEmpty(path))
            {
                AssetDatabase.CreateAsset(mapGridAsset, path);
                AssetDatabase.SaveAssets();
                Debug.Log("保存网格到 " + path);
            }

            // 清除进度条
            EditorUtility.ClearProgressBar();
        }

        void LoadGrid(AStarPathfindingBrain brain)
        {
            EditorCoroutines.Execute(LoadGridAsync(brain));
        }

        IEnumerator LoadGridAsync(AStarPathfindingBrain brain)
        {
            MapGridInfo[] matrix = brain.grid.matrix;
            int count = 0;
            // 创建新的 AStarPathfindingGridItem
            for (int row = 0; row < brain.grid.rows; row++)
            {
                for (int col = 0; col < brain.grid.cols; col++)
                {
                    count++;
                    MapGridInfo gridInfo = matrix[row * brain.grid.cols + col]; // 使用行列计算索引
                    var item = CreateCubeItem(gridInfo.position);
                    item.itemAlpha = t.itemAlpha;
                    item.IsObstacle = gridInfo.isObstacle; // 设置障碍物状态
                    item.matrixCol = col;
                    item.matrixRow = row;
                    item.gameObject.name = $"{item.matrixCol},{item.matrixRow}";
                    item.jumpPointsDistace = gridInfo.jumpPointsDistace;
                    Vector2 rowCol = new Vector2(item.matrixRow, item.matrixCol);
                    item.isJumpPoints = t.grid.jumpPoints.Contains(rowCol);
                    EditorUtility.DisplayProgressBar("网格加载", $"正在加载 ({count}/{brain.grid.rows * brain.grid.cols})", (float)count / (brain.grid.rows * brain.grid.cols));

                    if (count % 2000 == 0)
                        yield return null;
                }
            }

            EditorUtility.ClearProgressBar();
        }

        bool HasGrid()
        {
            return t.coverageBoxTrigger.transform.childCount > 0;
        }

        #endregion

        #region JPS+
        private void BakeJumpPoints(AStarPathfindingMapGridAssets gridAssets)
        {
            EditorCoroutines.Execute(BakeJumpPointsAsync(gridAssets));
        }
        IEnumerator BakeJumpPointsAsync(AStarPathfindingMapGridAssets gridAssets)
        {
            t.grid.jumpPoints = new List<Vector2>();
            //第一步 遍历所有节点找出跳点
            for (int i = 0; i < gridAssets.matrix.Length; i++)
            {
                MapGridInfo current = gridAssets.matrix[i];

                // Skip obstacles
                if (current.isObstacle)
                    continue;

                foreach (Vector2Int direction in t.GetDirections())
                {
                    if (t.HasForcedNeighbor(current, direction))
                    {
                        Vector2 rowCol = new Vector2(current.rowIndex, current.colIndex);
                        if (!t.grid.jumpPoints.Contains(rowCol))
                        {
                            t.grid.jumpPoints.Add(rowCol);
                        }
                    }
                    EditorUtility.DisplayProgressBar("JPS+烘培", $"计算跳点 ({i}/{gridAssets.matrix.Length})", (float)i / (gridAssets.matrix.Length));

                }

                if (i % 2000 == 0)
                    yield return null;
            }

            //第二步 遍历所有节点 对跳点 障碍进行可达性记录

            for (int i = 0; i < gridAssets.matrix.Length; i++)
            {
                MapGridInfo current = gridAssets.matrix[i];

                // Skip obstacles
                if (current.isObstacle)
                    continue;

                foreach (Vector2Int direction in t.GetDirections())
                {
                    int count = t.JumpBakeDistance(current, direction);
                    int idx = t.IndexByVector2Distance(direction);
                    gridAssets.matrix[i].jumpPointsDistace[idx] = count;
                }
                EditorUtility.DisplayProgressBar("JPS+烘培", $"计算节点可达性 ({i}/{gridAssets.matrix.Length})", (float)i / (gridAssets.matrix.Length));
                if (i % 2000 == 0)
                    yield return null;
            }

            EditorUtility.ClearProgressBar();
            // Mark the asset as dirty to ensure Unity saves the changes
            EditorUtility.SetDirty(gridAssets);
        }
        #endregion

        #region 地图网络笔刷
        private bool isLeftDragging = false;  // 左键拖动标志
        private bool isRightDragging = false; // 右键拖动标志
        private float brushSize = 10f;         // 画笔大小
        private Tool previousTool = Tool.None; // 记录之前的工具

        private void OnSceneGUI(SceneView sceneView)
        {

            Event e = Event.current;
            AStarPathfindingBrain brain = (AStarPathfindingBrain)target;
            if (!HasGrid()) return;
            // 判断是否按住 Ctrl 键
            if (e.control)
            {
                // 保存当前工具并切换为 Hand Tool
                if (previousTool == Tool.None)
                {
                    previousTool = Tools.current;  // 记录当前工具
                    Tools.current = Tool.View;     // 切换为 Hand Tool
                }

                // 获取鼠标位置对应的世界坐标
                Ray worldRay = HandleUtility.GUIPointToWorldRay(e.mousePosition);
                RaycastHit hit;

                // 如果鼠标指向物体并且碰到item
                if (Physics.Raycast(worldRay, out hit, Mathf.Infinity, brain.itemlayer))
                {
                    Vector3 hitPoint = hit.point;
                    // 左键按住拖动
                    if (e.type == EventType.MouseDown && e.button == 0)
                    {
                        isLeftDragging = true;
                        SetObstacleInArea(hitPoint, true, brain);  // 设置为障碍物
                        e.Use();  // 阻止事件传递
                    }
                    else if (e.type == EventType.MouseDrag && isLeftDragging)
                    {
                        SetObstacleInArea(hitPoint, true, brain);
                        e.Use();  // 阻止事件传递
                    }
                    else if (e.type == EventType.MouseUp)
                    {
                        isLeftDragging = false;
                    }

                    // 右键按住拖动
                    if (e.type == EventType.MouseDown && e.button == 1)
                    {
                        isRightDragging = true;
                        SetObstacleInArea(hitPoint, false, brain);  // 设置为非障碍物
                        e.Use();  // 阻止事件传递
                    }
                    else if (e.type == EventType.MouseDrag && isRightDragging)
                    {

                        SetObstacleInArea(hitPoint, false, brain);
                        e.Use();  // 阻止事件传递
                    }
                    else if (e.type == EventType.MouseUp)
                    {
                        isRightDragging = false;
                    }

                    // 绘制画笔指示器
                    Handles.color = new Color(0f, 0f, 1f, 0.3f); // 蓝色画笔圈
                    Handles.DrawSolidDisc(hitPoint, Vector3.up, brushSize); // 根据画笔大小绘制圆形

                    // 使用滚轮调整画笔大小
                    if (e.type == EventType.ScrollWheel)
                    {
                        brushSize -= e.delta.y * 0.5f; // 调整步长
                        brushSize = Mathf.Clamp(brushSize, 0.5f, 50f); // 限制画笔大小
                        e.Use();  // 阻止事件传递
                    }
                }
            }
            else
            {
                isLeftDragging = false;
                isRightDragging = false;
                if (previousTool != Tool.None)
                {
                    // 当 Ctrl 键释放时，恢复之前的工具
                    Tools.current = previousTool;
                    previousTool = Tool.None;  // 重置之前的工具记录
                }
            }
        }

        // 设置指定区域内的网格项为障碍物或非障碍物
        private void SetObstacleInArea(Vector3 center, bool isObstacle, AStarPathfindingBrain brain)
        {
            // 遍历所有网格项
            foreach (Transform child in brain.coverageBoxTrigger.transform)
            {
                AStarPathfindingGridItem gridItem = child.GetComponent<AStarPathfindingGridItem>();
                if (gridItem != null)
                {
                    // 判断网格项是否在当前画笔范围内
                    if (Vector3.Distance(child.position, center) <= brushSize)
                    {
                        gridItem.IsObstacle = isObstacle;  // 设置障碍物状态
                        EditorUtility.SetDirty(gridItem);  // 标记为已修改，保证编辑器中及时更新
                    }
                }
            }
        }

        #endregion
    }
}
