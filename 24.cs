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

        accuracy = Math.Max(accuracy, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

        List<Point3d> dir0 = SampleCurvature(srf, uv, accuracy, max, angle, alg);
        List<Point3d> dir1 = SampleCurvature(srf, uv, accuracy, max, angle + Math.PI, alg);

        if (dir1.Count > 0) dir1.RemoveAt(0);
        dir1.Reverse();
        dir1.AddRange(dir0);
        pts = dir1;
    }

    private List<Point3d> SampleCurvature(Surface srf, Point3d uv, double accuracy, bool max, double angle, int alg)
    {
        Point3d p = uv;
        Interval U = srf.Domain(0);
        Interval V = srf.Domain(1);
        List<Point3d> samples = new List<Point3d>();

        while (true)
        {
            samples.Add(srf.PointAt(p.X, p.Y));
            Vector3d dir = new Vector3d();

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
                    RhinoApp.WriteLine("Choose an algorithm between 1 and 3");
                    break;
            }

            if (dir == Vector3d.Unset) break;

            double s, t;
            Point3d pt = samples[samples.Count - 1] + dir;
            if (!srf.ClosestPoint(pt, out s, out t)) break;

            if (samples.Count > 9999 || !U.IncludesParameter(s, true) || !V.IncludesParameter(t, true)) break;
            if (Math.Abs(p.X - s) < 1e-12 && Math.Abs(p.Y - t) < 1e-12) break;

            p.X = s;
            p.Y = t;
        }

        return samples;
    }

    private Vector3d Euler(Surface srf, Point3d p, bool max, double angle, double h, List<Point3d> samples)
    {
        int N = samples.Count;
        Vector3d prevDir = N > 1 ? samples[N - 1] - samples[N - 2] : Vector3d.Unset;
        return GetDir(srf, p, max, angle, h, prevDir);
    }

    private Vector3d ModEuler(Surface srf, Point3d p, bool max, double angle, double h, List<Point3d> samples)
    {
        int N = samples.Count;
        Vector3d prevDir = N > 1 ? samples[N - 1] - samples[N - 2] : Vector3d.Unset;
        Vector3d dir1 = GetDir(srf, p, max, angle, h, prevDir);
        if (dir1 == Vector3d.Unset) return Vector3d.Unset;

        Point3d pt = samples[samples.Count - 1] + dir1;
        double s, t;
        if (!srf.ClosestPoint(pt, out s, out t)) return Vector3d.Unset;

        pt.X = s;
        pt.Y = t;
        Vector3d dir2 = GetDir(srf, pt, max, angle, h, dir1);
        return (dir1 + dir2) * 0.5;
    }

    private Vector3d RK4(Surface srf, Point3d p, bool max, double angle, double h, List<Point3d> samples)
    {
        int N = samples.Count;
        Vector3d prevDir = N > 1 ? samples[N - 1] - samples[N - 2] : Vector3d.Unset;

        Vector3d K1 = GetDir(srf, p, max, angle, h, prevDir);
        if (K1 == Vector3d.Unset) return Vector3d.Unset;

        Point3d pt1 = samples[samples.Count - 1] + K1 * 0.5;
        double s, t;
        if (!srf.ClosestPoint(pt1, out s, out t)) return Vector3d.Unset;
        pt1.X = s;
        pt1.Y = t;

        Vector3d K2 = GetDir(srf, pt1, max, angle, h, K1);
        if (K2 == Vector3d.Unset) return Vector3d.Unset;

        return (K1 + 2 * K2) / 3.0;
    }
}
