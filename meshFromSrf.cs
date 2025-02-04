using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(Surface Srf, int U, int V, int Iterations, double Lambda_Ref, double Lambda_Len, double Lambda_Cur, ref object A)
    {
        // 檢查輸入是否有效
        if (Srf == null || U < 2 || V < 2 || Iterations < 1)
        {
            A = "Invalid Input";
            return;
        }

        // 生成初始網格
        Mesh mesh = GenerateMesh(Srf, U, V);

        // 優化網格
        Mesh optimizedMesh = OptimizeMesh(mesh, Srf, Iterations, Lambda_Ref, Lambda_Len, Lambda_Cur);

        // 輸出結果
        A = optimizedMesh;
    }

    // 生成初始網格
    private Mesh GenerateMesh(Surface srf, int u, int v)
    {
        Mesh mesh = new Mesh();
        List<Point3d> points = new List<Point3d>();

        // 創建網格頂點
        for (int i = 0; i <= u; i++)
        {
            for (int j = 0; j <= v; j++)
            {
                double uNorm = i / (double)u;
                double vNorm = j / (double)v;
                Point3d pt = srf.PointAt(uNorm, vNorm);
                points.Add(pt);
                mesh.Vertices.Add(pt);
            }
        }

        // 創建網格面
        for (int i = 0; i < u; i++)
        {
            for (int j = 0; j < v; j++)
            {
                int index1 = i * (v + 1) + j;
                int index2 = index1 + 1;
                int index3 = index1 + (v + 1);
                int index4 = index3 + 1;
                mesh.Faces.AddFace(index1, index2, index4, index3);
            }
        }

        mesh.Normals.ComputeNormals();
        mesh.Compact();
        return mesh;
    }

    // 優化網格
  private Mesh OptimizeMesh(Mesh mesh, Surface srf, int iterations, double lambdaRef, double lambdaLen, double lambdaCur)
  {
    // 複製 Mesh，避免影響原始數據
    Mesh optimizedMesh = mesh.DuplicateMesh();

    for (int iter = 0; iter < iterations; iter++)
    {
      Mesh tempMesh = optimizedMesh.DuplicateMesh();
      double bestEnergy = ComputeEnergy(tempMesh, srf, lambdaRef, lambdaLen, lambdaCur);

      for (int i = 0; i < tempMesh.Vertices.Count; i++)
      {
        Point3d pt = tempMesh.Vertices[i];

        // 確保點在曲面上，獲取最近點
        Point3d closestPt = GetClosestPoint(srf, pt);
        if (closestPt == Point3d.Unset) continue; // 避免錯誤點

        // 計算移動向量
        Vector3d moveVec = (closestPt - pt) * 0.1;
        if (moveVec.Length < 1e-6) continue; // 避免過小移動

        // 創建新 Mesh 以防止影響現有結構
        Mesh newMesh = tempMesh.DuplicateMesh();
        newMesh.Vertices.SetVertex(i, pt + moveVec);

        // 計算能量，確保新形狀更優化
        double energy = ComputeEnergy(newMesh, srf, lambdaRef, lambdaLen, lambdaCur);
        if (energy < bestEnergy)
        {
          bestEnergy = energy;
          optimizedMesh = newMesh.DuplicateMesh(); // 更新最佳 Mesh
        }
      }
    }

    return optimizedMesh;
  }

  // 計算能量函數
  private double ComputeEnergy(Mesh mesh, Surface srf, double lambdaRef, double lambdaLen, double lambdaCur)
  {
    double E_ref = 0;
    double E_len = 0;
    double E_cur = 0;

    // 計算距離懲罰能量 E_ref
    for (int i = 0; i < mesh.Vertices.Count; i++)
    {
      Point3d pt = mesh.Vertices[i];
      Point3d closestPt = GetClosestPoint(srf, pt);

      if (closestPt != Point3d.Unset)
      {
        E_ref += pt.DistanceToSquared(closestPt);
      }
    }

    // 計算邊長均勻能量 E_len
    double avgLen = 0;
    List<double> edgeLengths = new List<double>();

    for (int i = 0; i < mesh.TopologyEdges.Count; i++)
    {
      // ✅ 使用正確的 GetTopologyVertices 語法
      IndexPair indices = mesh.TopologyEdges.GetTopologyVertices(i);
      int v1 = indices.I;
      int v2 = indices.J;

      // 確保索引有效
      if (v1 < 0 || v2 < 0 || v1 >= mesh.Vertices.Count || v2 >= mesh.Vertices.Count) continue;

      Point3d p1 = mesh.Vertices[v1];
      Point3d p2 = mesh.Vertices[v2];

      double length = p1.DistanceTo(p2);
      edgeLengths.Add(length);
      avgLen += length;
    }

    if (edgeLengths.Count > 0)
    {
      avgLen /= edgeLengths.Count;
      foreach (double length in edgeLengths)
      {
        E_len += Math.Pow(length - avgLen, 2);
      }
    }

    // 計算曲率能量 E_cur
    for (int i = 0; i < mesh.Vertices.Count - 2; i++)
    {
      Point3d v1 = mesh.Vertices[i];
      Point3d v2 = mesh.Vertices[i + 1];
      Point3d v3 = mesh.Vertices[i + 2];

      double r = ComputeCircumcircleRadius(v1, v2, v3);
      if (r > 0) E_cur += 1 / (r * r);
    }

    return lambdaRef * E_ref + lambdaLen * E_len + lambdaCur * E_cur;
  }

  // 計算三點圓弧曲率半徑
  private double ComputeCircumcircleRadius(Point3d v1, Point3d v2, Point3d v3)
  {
    double a = v1.DistanceTo(v2);
    double b = v2.DistanceTo(v3);
    double c = v3.DistanceTo(v1);

    if (a <= 1e-6 || b <= 1e-6 || c <= 1e-6) return double.PositiveInfinity;
    if ((a + b <= c) || (a + c <= b) || (b + c <= a)) return double.PositiveInfinity;

    double s = (a + b + c) / 2;
    double A = Math.Sqrt(s * (s - a) * (s - b) * (s - c));

    if (A <= 1e-6) return double.PositiveInfinity;

    return (a * b * c) / (4 * A);
  }




  // 獲取最近點
  private Point3d GetClosestPoint(Surface srf, Point3d pt)
  {
    double u, v;
    bool success = srf.ClosestPoint(pt, out u, out v);

    if (!success) return Point3d.Unset;

    return srf.PointAt(u, v);
  }
}
