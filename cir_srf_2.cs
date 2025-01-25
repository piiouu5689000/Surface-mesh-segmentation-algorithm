private IntersectionResult ComputeIntersections(Brep bsrf, Point3d pt0, Vector3d dir, double radi, int it)
  {
    // 創建結果對象
    IntersectionResult result = new IntersectionResult();

    // 單位化方向向量
    dir.Unitize();

    // 設置初始點
    Point3d cp = pt0;

    // 計算法向量
    Vector3d zAxis = Vector3d.ZAxis;
    Vector3d dir2 = Vector3d.CrossProduct(dir, zAxis);

    // 檢查 dir2 是否有效
    if (dir2.IsZero)
    {
      RhinoApp.WriteLine("Direction vector is parallel to Z-axis.");
      return result;
    }

    dir2.Unitize(); // 確保 dir2 為單位向量
    RhinoApp.WriteLine("Initialized direction vector and cross-product direction.");

    // 判斷是否為第一次迴圈
    bool first = true;

    // 迴圈進行多次交點計算
    for (int i = 0; i < it; i++)
    {
      RhinoApp.WriteLine(string.Format("Iteration {0}: Starting computation.", i + 1));

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
      if (!isIntersecting || intP == null || intP.Length == 0)
      {
        RhinoApp.WriteLine(string.Format("Iteration {0}: No intersection points found.", i + 1));
        break;
      }

      RhinoApp.WriteLine(string.Format("Iteration {0}: Found {1} intersection points.", i + 1, intP.Length));

      // 第一次迭代
      // 選擇正確方向的交點
      Point3d tp;
      if (first)
      {
        first = false;
        RhinoApp.WriteLine("Processing first iteration.");
        if (intP.Length > 1)
        {
          // 將以原點生成圓的兩個交點放入列表
          foreach(var p0 in intP){
            result.intpList.Add(p0);
          }
          // 過濾出沿 dir2 方向的交點，並作為當前 tp
          var validPoints = intP.Where(p => Vector3d.Multiply(p - cp, dir2) < 0).ToList();
          tp = validPoints.FirstOrDefault();
          if (tp == Point3d.Unset)
          {
            RhinoApp.WriteLine("No valid points found in the correct direction.");
            break;
          }
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
            RhinoApp.WriteLine(string.Format("Iteration {0}: Single intersection point is in the wrong direction.", i + 1));
            break;
          }
        }
      }
      else
      {
        // 非第一次迭代
        RhinoApp.WriteLine("Processing subsequent iterations.");
        Point3d slp;
        if (result.intpList.Count >= 2)
        {
          // 取出料表中倒數第二個點，為slp
          slp = result.intpList.ElementAt(result.intpList.Count - 2);
        }
        else
        {
          RhinoApp.WriteLine(string.Format("Iteration {0}: Not enough points in result list to proceed.", i + 1));
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
            RhinoApp.WriteLine(string.Format("Iteration {0}: Single intersection point is in the wrong direction.", i + 1));
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

      RhinoApp.WriteLine(string.Format("Iteration {0}: Successfully added point {1} and circle.", i + 1, tp));
    }

    RhinoApp.WriteLine("Computation completed.");
    return result;
  }

  // IntersectionResult 類
  public class IntersectionResult
  {
    public List<Point3d> intpList { get; set; }
    public List<Curve> Circles { get; set; }
    public Vector3d dd { get; set; }

    // 構造函式
    public IntersectionResult()
    {
      intpList = new List<Point3d>();
      Circles = new List<Curve>();
      dd = new Vector3d();
    }
  }
