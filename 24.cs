using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using System.Linq;

public class CurvatureTracing : GH_ScriptInstance
{
    private RhinoDoc RhinoDocument;
    private GH_Document GrasshopperDocument;
    private IGH_Component Component;
    private int Iteration;

    public override void RunScript(Surface srf, Point3d uv, double accuracy, double angle, bool max, int alg, ref object pts)
    {
        if (srf == null || !uv.IsValid) return;

        // 確保精度不低於模型公差
        accuracy = Math.Max(accuracy, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

        // 計算兩個方向的曲率線
        List<Point3d> dir0 = SampleCurvature(srf, uv, accuracy, max, angle, alg);
        List<Point3d> dir1 = SampleCurvature(srf, uv, accuracy, max, angle + Math.PI, alg);

        // 移除重複點並反轉方向
        if (dir1.Count > 0) dir1.RemoveAt(0);
        dir1.Reverse();

        // 合併兩個方向的結果
        dir1.AddRange(dir0);
        pts = dir1;
    }

    /// <summary>
    /// 追蹤曲面的主曲率方向。
    /// </summary>
    private List<Point3d> SampleCurvature(Surface srf, Point3d uv, double accuracy, bool max, double angle, int alg)
    {
        Point3d p = uv;
        Interval U = srf.Domain(0);
        Interval V = srf.Domain(1);
        List<Point3d> samples = new List<Point3d>();

        while (true)
        {
            samples.Add(srf.PointAt(p.X, p.Y));
            Vector3d dir = Vector3d.Unset;

            // 選擇數值積分方法
            switch (alg)
            {
                case 1:
                    dir = Euler(srf, p, max, angle, accuracy, samples);
                    break;
                case 2:
                    dir = ModEuler(srf, p, max, angle, accuracy, samples);
                    break;
                case 3:
                    dir = RK4(srf, p, max, angle, accuracy, samples);
                    break;
                default:
                    RhinoApp.WriteLine("請選擇 1 到 3 之間的算法");
                    break;
            }

            if (dir == Vector3d.Unset) break;

            double s = 0, t = 0;
            Point3d pt = samples[samples.Count - 1] + dir;
            if (!srf.ClosestPoint(pt, out s, out t)) break;

            // 停止條件
            if (samples.Count > 9999 || !U.IncludesParameter(s, true) || !V.IncludesParameter(t, true)) break;
            if (Math.Abs(p.X - s) < 1e-12 && Math.Abs(p.Y - t) < 1e-12) break;

            p.X = s;
            p.Y = t;
        }

        return samples;
    }

    /// <summary>
    /// 計算指定點的曲率方向。
    /// </summary>
    private Vector3d GetDir(Surface srf, Point3d p, bool max, double angle, double h, Vector3d prevDir)
    {
        SurfaceCurvature crv = srf.CurvatureAt(p.X, p.Y);
        if (crv == null) return Vector3d.Unset;

        Vector3d dir = (crv.Kappa(0) > crv.Kappa(1)) == max ? crv.Direction(0) : crv.Direction(1);
        dir.Rotate(angle, crv.Normal);

        if (!dir.IsValid || !dir.Unitize()) return Vector3d.Unset;
        dir *= h;

        // 如果方向與前一方向相反，則翻轉方向
        if (prevDir != Vector3d.Unset && dir.IsParallelTo(prevDir, 0.5 * Math.PI) < 0)
            dir.Reverse();

        return dir;
    }

    /// <summary>
    /// 使用 Euler 方法進行數值積分。
    /// </summary>
    private Vector3d Euler(Surface srf, Point3d p, bool max, double angle, double h, List<Point3d> samples)
    {
        Vector3d prevDir = samples.Count > 1 ? samples[samples.Count - 1] - samples[samples.Count - 2] : Vector3d.Unset;
        return GetDir(srf, p, max, angle, h, prevDir);
    }

    /// <summary>
    /// 使用修正 Euler 方法進行數值積分。
    /// </summary>
    private Vector3d ModEuler(Surface srf, Point3d p, bool max, double angle, double h, List<Point3d> samples)
    {
        Vector3d prevDir = samples.Count > 1 ? samples[samples.Count - 1] - samples[samples.Count - 2] : Vector3d.Unset;
        Vector3d dir1 = GetDir(srf, p, max, angle, h, prevDir);
        if (dir1 == Vector3d.Unset) return Vector3d.Unset;

        Point3d pt = samples[samples.Count - 1] + dir1;
        double s = 0, t = 0;
        if (!srf.ClosestPoint(pt, out s, out t)) return Vector3d.Unset;

        pt.X = s;
        pt.Y = t;
        Vector3d dir2 = GetDir(srf, pt, max, angle, h, dir1);
        return (dir1 + dir2) * 0.5;
    }

    /// <summary>
    /// 使用四階 Runge-Kutta 方法進行數值積分。
    /// </summary>
    private Vector3d RK4(Surface srf, Point3d p, bool max, double angle, double h, List<Point3d> samples)
    {
        Vector3d prevDir = samples.Count > 1 ? samples[samples.Count - 1] - samples[samples.Count - 2] : Vector3d.Unset;

        Vector3d K1 = GetDir(srf, p, max, angle, h, prevDir);
        if (K1 == Vector3d.Unset) return Vector3d.Unset;

        Point3d pt1 = samples[samples.Count - 1] + K1 * 0.5;
        double s = 0, t = 0;
        if (!srf.ClosestPoint(pt1, out s, out t)) return Vector3d.Unset;
        pt1.X = s;
        pt1.Y = t;

        Vector3d K2 = GetDir(srf, pt1, max, angle, h, K1);
        if (K2 == Vector3d.Unset) return Vector3d.Unset;

        return (K1 + 2 * K2) / 3.0;
    }
}
