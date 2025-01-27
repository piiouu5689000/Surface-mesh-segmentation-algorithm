private void RunScript(Brep bsrf, Point3d pt0, List<Vector3d> dir, double radi, int it, ref object IntP, ref object Cir, ref object C, ref object A, ref object B)
  {

    // 創建樹狀結構來存儲每次迴圈的結果
    DataTree<Point3d> intpTree = new DataTree<Point3d>();
    DataTree<Curve> circlesTree = new DataTree<Curve>();
    List<Vector3d> allDirs = new List<Vector3d>(); // 每組的方向向量
    
    // 用於記錄每個路徑的點數量
    Dictionary<string, int> pointCounts = new Dictionary<string, int>(); 
    
    // 進行分組的雙重迴圈
    for (int group = 0; group < 2; group++) // 外層代表分組 [dir[0], dir[1]] 和 [dir[2], dir[3]]
    {
        for (int repeat = 0; repeat < 2; repeat++) // 每組重複兩次
        {
            for (int i = 0; i < 2; i++) // 遍歷組內的方向
            {
                int dirIndex = group * 2 + i; // 計算方向索引，group=0 對應 dir[0], dir[1]，group=1 對應 dir[2], dir[3]
                
                // 調用 ComputeIntersections，存儲每次的結果
                IntersectionResult singleResult = ComputeIntersections(bsrf, pt0, dir[dirIndex], radi, it);
    
                // 保存交點到樹狀結構
                if (singleResult.intpList != null)
                {
                    // 設置路徑，外層用 group 表示分組，內層用 repeat 和 i 表示重複次數和方向
                    GH_Path path = new GH_Path(group, repeat * 2 + i);
                    foreach (var p in singleResult.intpList)
                    {
                        intpTree.Add(p, path);
                    }
    
                    // 記錄該路徑下的點數量
                    pointCounts[path.ToString()] = singleResult.intpList.Count;
                }
                else
                {
                    // 如果沒有交點，記錄點數量為 0
                    GH_Path path = new GH_Path(group, repeat * 2 + i);
                    pointCounts[path.ToString()] = 0;
                }
            }
        }
    }

  
    // 輸出每個路徑的點數量
    List<string> pathPointCounts = new List<string>();
    foreach (var kvp in pointCounts)
    {
        pathPointCounts.Add($"{kvp.Key}: {kvp.Value} points");
    }
    
    // 輸出點數量的統計結果
    C = pathPointCounts;


    
  
  ////// 進行圓的多次交點計算
    for (int i = 0; i < 4; i++)
    {
      // 調用 ComputeIntersections，存儲每次的結果
      IntersectionResult singleResult = ComputeIntersections(bsrf, pt0, dir[i], radi, it);


      // 保存圓曲線到樹狀結構
      if (singleResult.Circles != null)
      {
        GH_Path path = new GH_Path(i); // 與交點相同的路徑
        foreach (var circle in singleResult.Circles)
        {
          circlesTree.Add(circle, path);
        }
      }

      // 保存方向向量
      allDirs.Add(singleResult.dd);
    }


    IntP = intpTree;       // 每組交點輸出為一棵樹
    Cir = circlesTree;     // 每組圓輸出為一棵樹
    C = allDirs;           // 所有方向向量輸出



  }

  ////////////////////////////////////// <Custom additional code> /////////////////////////////////////

  private IntersectionResult ComputeIntersections(Brep bsrf, Point3d pt0, Vector3d dir, double radi, int it)
  {
    // 創建結果對象
    IntersectionResult result = new IntersectionResult();

    result.intpList.Add(pt0);

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
      RhinoApp.WriteLine(string.Format("迭代 {0}: 開始計算.", i + 1));

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
        RhinoApp.WriteLine(string.Format("迭代 {0}: 無交點.", i + 1));
        break;
      }

      RhinoApp.WriteLine(string.Format("迭代 {0}: 找到 {1} 交點", i + 1, intP.Length));

      // 選擇正確方向的交點
      Point3d tp;
      if (first)
      {
        first = false;
        RhinoApp.WriteLine("Processing first iteration.");
        if (intP.Length > 1)
        {
          // 過濾出沿 dir2 方向的交點，並作為當前 tp
          var validPoints = intP.Where(p => Vector3d.Multiply(p - cp, dir2) < 0).ToList();

          // 設置 tp 為篩選出的第一個有效點
          tp = validPoints.FirstOrDefault();
          if (tp == Point3d.Unset)
          {
            RhinoApp.WriteLine("該方向無交點");
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
            RhinoApp.WriteLine(string.Format("迭代 {0}: 只有一個交點在反方向上", i + 1));
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
          slp = result.intpList.ElementAt(result.intpList.Count - 2);
        }
        else
        {
          RhinoApp.WriteLine(string.Format("迭代 {0}: 列表中的點不夠繼續", i + 1));
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
