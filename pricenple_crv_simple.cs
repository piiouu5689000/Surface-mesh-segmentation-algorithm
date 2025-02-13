private void RunScript(Surface srf, Point3d uv, double angle, double h, ref object A)
  {


    Point3d p = uv;
    Interval U = srf.Domain(0);
    Interval V = srf.Domain(1);
    List<Point3d> sample = new List<Point3d>();
    double s = 0, t = 0;
    srf.ClosestPoint(p, out s, out t);


    while(true)
    {
      // 先存入第一個點
      Point3d cp = srf.PointAt(s, t);
      sample.Add(cp);

      // 找出該點曲率
      SurfaceCurvature crv = srf.CurvatureAt(s, t);
      if(crv == null)
        break;

      // 選出方向
      Vector3d dir;
      if (crv.Kappa(0) > crv.Kappa(1))
      {
        dir = crv.Direction(0);
      }
      else
      {
        dir = crv.Direction(1);
      }

      // 向量單位化
      if(!dir.IsValid || !dir.Unitize())
        break;

      // 旋轉向量
      double ang = angle * Math.PI / 180.0;
      dir.Rotate(ang, crv.Normal);

      // 向量X步長
      dir *= h;

      Vector3d predir;
      if(sample != null && sample.Count > 1)
      {
        predir = sample[sample.Count - 1] - sample[sample.Count - 2];

        // 若predir存在，且predir與dir之間夾角大於90度
        if(predir.IsValid && dir.IsParallelTo(predir, 0.5 * Math.PI) < 0)
        {
          dir.Reverse();
        }
      }

      Point3d pt = sample[sample.Count - 1] + dir;

      // 判斷pt是否在曲面上，並找到uv
      double u = 0, v = 0;
      if(!srf.ClosestPoint(pt, out u, out v))
        break;

      // 判斷點數量，u與v範圍
      if(sample.Count > 9999 || !U.IncludesParameter(u, true) || !V.IncludesParameter(v, true))
        break;

      // 當前點是否與上一點重合
      if(Math.Abs(u - s) < 1e-12 && Math.Abs(v - t) < 1e-12)
        break;

      s = u;
      t = v;

    }

    A = sample;

  }
