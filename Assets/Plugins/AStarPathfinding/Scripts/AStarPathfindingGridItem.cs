using System.Collections;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;

namespace AmagiSakuya.AstarPathFinding
{
    [ExecuteInEditMode]
    public class AStarPathfindingGridItem : MonoBehaviour
    {
        public bool IsObstacle;
        public int matrixCol;
        public int matrixRow;
        public int[] jumpPointsDistace = new int[8];
        public bool isJumpPoints;
        public float itemAlpha = 0.1f;

        Renderer objectRenderer;  // 对象的渲染器

        void Start()
        {
            if (Application.isPlaying) return;
            objectRenderer = GetComponent<Renderer>();
            UpdateGridItemColor();
        }

        // Update is called once per frame
        void Update()
        {
            if (Application.isPlaying) return;
            UpdateGridItemColor();
        }

        void UpdateGridItemColor()
        {
            if (objectRenderer == null) return;
            // 根据 IsObstacle 值来设置材质的颜色
            Color color = IsObstacle ? new Color(1f, 0f, 0f, itemAlpha) : isJumpPoints? new Color(0f, 0f, 1f, itemAlpha) : new Color(0f, 1f, 0f, itemAlpha);
            objectRenderer.sharedMaterial.color = color;
        }

#if UNITY_EDITOR
        Vector3 center;
        string centerText;
        Vector3 size;
        void OnDrawGizmos()
        {
            if (Selection.activeGameObject != gameObject)
                return; // 如果没有被选中，则不绘制 Gizmos

            centerText = GetJumpDistacneText();
            // 中心位置
            center = transform.position;
            size = transform.localScale * 0.5f;
            // 绘制调试文本
            Gizmos.color = Color.green;
            Handles.Label(center - new Vector3(size.x, 0, -size.y), centerText);
        }
#endif
        string GetJumpDistacneText()
        {
            int max = 0;
            for (int i = 0; i < jumpPointsDistace.Length; i++)
            {
                max = Mathf.Max(max, Mathf.Abs(jumpPointsDistace[i]));
            }
            if (max == 0) return "";
            return $"{jumpPointsDistace[7]} {jumpPointsDistace[0]} {jumpPointsDistace[4]} \n {jumpPointsDistace[3]} ({matrixCol},{matrixRow}) {jumpPointsDistace[1]} \n {jumpPointsDistace[6]} {jumpPointsDistace[2]} {jumpPointsDistace[5]}";
        }

    }
}

