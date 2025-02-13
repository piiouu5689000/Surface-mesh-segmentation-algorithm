private void RunScript(Surface srf, Point3d pt, double angle, double h, ref object A, ref object B, ref object C, ref object D, ref object E)
  {


    List<Point3d> dir0 = MeanCrvPt(srf, pt, angle, h);
    List<Point3d> dir1 = MeanCrvPt(srf, pt, angle + 180, h);

    if(dir1.Count > 0) dir1.RemoveAt(0);
    dir1.Reverse();
    dir1.AddRange(dir0);
    A = dir1;

    var curves0 = AsymptoticCrvPt(srf, pt, angle, h);
    var curves1 = AsymptoticCrvPt(srf, pt, angle + 180, h);

    B = curves0.Curve1;
    C = curves0.Curve2;
    D = curves1.Curve1;
    E = curves1.Curve2;
  }

  // <Custom additional code> 

  private List<Point3d> MeanCrvPt(Surface srf, Point3d pt, double angle, double h)
  {
    List < Point3d > plist = new List<Point3d>();

    double u = 0, v = 0;
    srf.ClosestPoint(pt, out u, out v);
    Interval U = srf.Domain(0);
    Interval V = srf.Domain(1);

    while(true)
    {
      plist.Add(srf.PointAt(u, v));

      SurfaceCurvature crv = srf.CurvatureAt(u, v);
      if(crv == null) break;

      Vector3d dir;
      double k1, k2;

      if(crv.Kappa(0) > crv.Kappa(1))
      {
        dir = crv.Direction(0);
        k1 = crv.Kappa(0);
        k2 = crv.Kappa(1);
      }
      else
      {
        dir = crv.Direction(1);
        k1 = crv.Kappa(1);
        k2 = crv.Kappa(0);
      }

      if(!dir.IsValid || !dir.Unitize()) break;
      if(k1 * k2 > 0) break;

      double ang = angle * Math.PI / 180.0;
      dir.Rotate(ang, crv.Normal);

      dir *= h;

      if(plist != null && plist.Count > 1)
      {
        Vector3d pred = plist[plist.Count - 1] - plist[plist.Count - 2];
        if (pred.IsValid && pred.IsParallelTo(dir, 0.5 * Math.PI) < 0 )
        {
          dir.Reverse();
        }
      }

      Point3d newp = plist[plist.Count - 1] + dir;

      double s = 0, t = 0;
      if(!srf.ClosestPoint(newp, out s, out t)) break;

      if(!U.IncludesParameter(s, true) || !V.IncludesParameter(t, true)) break;

      if(plist.Count > 9999) break;

      if(Math.Abs(s - u) < 1e-12 && Math.Abs(t - v) < 1e-12) break;

      u = s;
      v = t;

    }

    return plist;
  }

  ////////////////////////////////////////////////////////
  ////////////// /// 用於存儲兩組點列表的類 //////////////
  ////////////////////////////////////////////////////////
  public class AsymptoticCurves
  {
    public List<Point3d> Curve1 { get; set; }
    public List<Point3d> Curve2 { get; set; }

    public AsymptoticCurves()
    {
      Curve1 = new List<Point3d>();
      Curve2 = new List<Point3d>();
    }
  }

  ////////////////////////////////////////////////////////
  ////////////// 輸出兩個漸近線 //////////////
  ////////////////////////////////////////////////////////
  private AsymptoticCurves AsymptoticCrvPt(Surface srf, Point3d pt, double angle, double h)
  {
    var result = new AsymptoticCurves();

    // 獲取起始點的UV參數
    double u = 0, v = 0;
    if (!srf.ClosestPoint(pt, out u, out v))
      return result;

    // 獲取曲面參數域
    Interval U = srf.Domain(0);
    Interval V = srf.Domain(1);

    // 計算兩個方向的漸近線
    result.Curve1 = ComputeAsymptoticCurve(srf, u, v, angle, h, U, V, true);
    result.Curve2 = ComputeAsymptoticCurve(srf, u, v, angle, h, U, V, false);

    return result;
  }

  ////////////////////////////////////////////////////////
  ////////////// 計算漸近線 //////////////
  ////////////////////////////////////////////////////////
  private List<Point3d> ComputeAsymptoticCurve(Surface srf, double u, double v,
    double angle, double h, Interval U, Interval V, bool useAlpha1)
  {
    List<Point3d> points = new List<Point3d>();

    while(true)
    {
      // 添加當前點
      points.Add(srf.PointAt(u, v));

      // 獲取曲率信息
      SurfaceCurvature crv = srf.CurvatureAt(u, v);
      if(crv == null) break;

      // 計算主曲率方向和值
      Vector3d dir;
      double k1, k2;
      if(crv.Kappa(0) > crv.Kappa(1))
      {
        dir = crv.Direction(0);
        k1 = crv.Kappa(0);
        k2 = crv.Kappa(1);
      }
      else
      {
        dir = crv.Direction(1);
        k1 = crv.Kappa(1);
        k2 = crv.Kappa(0);
      }

      // 檢查向量有效性
      if(!dir.IsValid || !dir.Unitize()) break;

      // 應用角度旋轉
      double ang = angle * Math.PI / 180.0;
      dir.Rotate(ang, crv.Normal);

      // 檢查是否為反雙曲面（主曲率異號）
      if(k1 * k2 > 0) break;

      // 計算漸近方向角度
      double alpha;
      double theta1;
      if(useAlpha1)
      {
        alpha = Math.Atan(Math.Sqrt((2 * Math.Sqrt(k2 * (-(k1 - k2))) + k1 - (2 * k2)) / k1));
        theta1 = Math.Atan(Math.Sqrt(-k1 / k2));
      }
      else
      {
        alpha = Math.Atan(Math.Sqrt((-2 * Math.Sqrt(k2 * (-(k1 - k2))) + k1 - (2 * k2)) / k1));
        theta1 = -Math.Atan(Math.Sqrt(-k1 / k2));
      }

      // 旋轉到漸近方向
      dir.Rotate(theta1, crv.Normal);

      // 應用步長
      dir *= h;

      // 調整方向以保持連續性
      if(points.Count > 1)
      {
        Vector3d pred = points[points.Count - 1] - points[points.Count - 2];
        if (pred.IsValid && pred.IsParallelTo(dir, 0.5 * Math.PI) < 0)
        {
          dir.Reverse();
        }
      }

      // 計算下一個點
      Point3d newp = points[points.Count - 1] + dir;

      // 投影到曲面上
      double s = 0, t = 0;
      if(!newp.IsValid || !srf.ClosestPoint(newp, out s, out t)) break;

      // 檢查邊界條件
      if(!U.IncludesParameter(s, true) || !V.IncludesParameter(t, true)) break;
      if(points.Count > 9999) break;
      if(Math.Abs(s - u) < 1e-12 && Math.Abs(t - v) < 1e-12) break;

      // 更新參數值
      u = s;
      v = t;
    }

    return points;
  }
