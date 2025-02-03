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
        Mesh optimizedMesh = mesh.DuplicateMesh();

        for (int iter = 0; iter < iterations; iter++)
        {
            Mesh tempMesh = optimizedMesh.DuplicateMesh();
            double bestEnergy = ComputeEnergy(tempMesh, srf, lambdaRef, lambdaLen, lambdaCur);

            for (int i = 0; i < tempMesh.Vertices.Count; i++)
            {
                Point3d pt = tempMesh.Vertices[i];
                Point3d closestPt = GetClosestPoint(srf, pt);
                Vector3d moveVec = (closestPt - pt) * 0.1;

                Mesh newMesh = tempMesh.DuplicateMesh();
                newMesh.Vertices.SetVertex(i, pt + moveVec);

                double energy = ComputeEnergy(newMesh, srf, lambdaRef, lambdaLen, lambdaCur);
                if (energy < bestEnergy)
                {
                    bestEnergy = energy;
                    optimizedMesh = newMesh;
                }
            }
        }
        return optimizedMesh;
    }

    // 計算能量函數
    private double ComputeEnergy(Mesh mesh, Surface srf, double lambdaRef, double lambdaLen, double lambdaCur)
    {
        double E_ref = 0; // 距離懲罰能量
        double E_len = 0; // 邊長均勻能量
        double E_cur = 0; // 曲率能量

        for (int i = 0; i < mesh.Vertices.Count; i++)
        {
            Point3d pt = mesh.Vertices[i];
            Point3d closestPt = GetClosestPoint(srf, pt);
            E_ref += pt.DistanceToSquared(closestPt);
        }

        double avgLen = 0;
        List<double> edgeLengths = new List<double>();

        foreach (MeshEdge edge in mesh.TopologyEdges)
        {
            Point3d p1 = mesh.Vertices[edge.I];
            Point3d p2 = mesh.Vertices[edge.J];
            double length = p1.DistanceTo(p2);
            edgeLengths.Add(length);
            avgLen += length;
        }

        avgLen /= edgeLengths.Count;
        foreach (double length in edgeLengths)
        {
            E_len += Math.Pow(length - avgLen, 2);
        }

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
        double s = (a + b + c) / 2;
        double A = Math.Sqrt(s * (s - a) * (s - b) * (s - c));
        if (A == 0) return double.PositiveInfinity;
        return (a * b * c) / (4 * A);
    }

    // 獲取最近點
    private Point3d GetClosestPoint(Surface srf, Point3d pt)
    {
        double u, v;
        srf.ClosestPoint(pt, out u, out v);
        return srf.PointAt(u, v);
    }
}
