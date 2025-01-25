private IntersectionResult ComputeIntersections(Brep bsrf, Point3d pt0, Vector3d dir, double radi, int it)
  {
    // 創建結果對象
    IntersectionResult result = new IntersectionResult();

    // 單位化方向向量
    dir.Unitize();

    // 設置初始點
    Point3d cp = pt0;

    // 法向量
    Vector3d zAxis = Vector3d.ZAxis;
    Vector3d dir2 = Vector3d.CrossProduct(dir, zAxis);

    // 檢查 dir2 是否有效
    if (dir2.IsZero)
    {
      RhinoApp.WriteLine("Direction vector is parallel to Z-axis.");
      return result;
    }

    dir2.Unitize(); // 確保 dir2 為單位向量

    // 判斷是否為第一次迴圈
    bool first = true;

    // 迴圈進行多次交點計算
    for (int i = 0; i < it; i++)
    {
      // 創建平面和圓
      Plane plane = new Plane(cp, dir);
      Circle circle = new Circle(plane, radi);
      Curve circleCurve = circle.ToNurbsCurve();

      // 設定容差，用於交點計算
      double tolerance = 0.9;

      // 計算交點
      Curve[] cirs;
      Point3d[] intP;
      bool isIntersecting = Intersection.CurveBrep(circleCurve, bsrf, tolerance, out cirs, out intP);
      // 如果沒有交點，停止迴圈
      if (!isIntersecting )
      {
        break;
      }

      // 選擇正確方向的交點
      Point3d tp;
      // 第一圈的狀況
      if (first)
      {
          first = false;
          if (intP.Length > 1)
          {
            // 過濾出沿 dir2 方向的交點，並作為當前tp
            var validPoints = intP.Where(p => Vector3d.Multiply(p - cp, dir2) < 0).ToList();
            tp = validPoints.FirstOrDefault();
          }
          else
          {
              // 如果只有一個交點，檢查方向是否正確
              if (Vector3d.Multiply(intP[0] - cp, dir2) > 0)
              {
                tp = intP[0];
              }
              else
              {
                break;
              }
          }
      }
      else
      {
         // 其他圈的狀況
          Point3d slp;
          if (result.intpList.Count >= 2)
          {
             slp = result.intpList.ElementAt(result.intpList.Count.Count - 2);
          }
          else
          {
             break;
          }
          if (intP.Length > 1)
          {
            Point3d farthestPoint = intP.OrderByDescending(p => p.DistanceTo(slp)).First();
            tp = farthestPoint;
          }
           else
          {
              // 如果只有一個交點，檢查方向是否正確
              if (Vector3d.Multiply(intP[0] - cp, dir2) > 0)
              {
                tp = intP[0];
              }
              else
              {
                break;
              }
          }
      }
         
      // 更新當前點
      cp = tp;

      // 將結果存入
      result.intpList.Add(tp);
      result.Circles.Add(circleCurve);
      result.dd = dir2;
      //result.ttp = testP;
    }

    return result;
  }
