private void RunScript(Brep bsrf, Point3d pt0, Vector3d dir, double radi, int it, ref object IntP, ref object Cir, ref object dd, ref object firstP)
  {

    // 創建樹狀結構來存儲每次迴圈的結果

    DataTree<Curve> circlesTree = new DataTree<Curve>();
    List<Vector3d> allDirs = new List<Vector3d>(); // 每組的方向向量



    List<Point3d> intpL = new List<Point3d>();
    List<Curve> cir = new List<Curve>();


    IntersectionResult result = ComputeIntersections(bsrf, pt0, dir, radi, it);


    intpL = result.intpList;
    cir = result.Circles;


    Cir = cir;     // 每組圓輸出為一棵樹
    IntP = intpL;
    dd = result.dd;
    firstP = result.fp;



  }

  // <Custom additional code> 



  // IntersectionResult 類

  private IntersectionResult ComputeIntersections(Brep bsrf, Point3d pt0, Vector3d dir, double radi, int it)
  {
    // 創建結果對象
    IntersectionResult result = new IntersectionResult();

    // 單位化方向向量
    dir.Unitize();

    // 設置初始點
    Point3d cp = pt0;

    // 在列表中存入起始點 pt0
    result.intpList.Add(pt0);

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

    ///////////////////////////// 開始迴圈 ////////////////////////////////
    for (int i = 0; i < it; i++)
    {
      RhinoApp.WriteLine(string.Format("迭代 {0}: 開始計算.", i + 1));

      // 每次迴圈增加 5 度偏轉
      double angle = 5.0 * Math.PI / 180.0; // 將 5 度轉換為弧度
      dir2.Rotate(angle, dir); // 圍繞主方向向量 dir 旋轉 dir2

      // 創建平面和圓
      Plane plane = new Plane(cp, dir2);
      Circle circle = new Circle(plane, radi);
      Curve circleCurve = circle.ToNurbsCurve();

      // 設定容差，用於交點計算
      double tolerance = 0.01;

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
          var validPoints = intP.Where(p => Vector3d.Multiply(p - cp, dir) > 0).ToList();

          // 設置 tp 為篩選出的第一個有效點
          tp = validPoints.FirstOrDefault();
          result.fp = tp; // 將第一點存入
          if (tp == Point3d.Unset)
          {
            RhinoApp.WriteLine("該方向無交點");
            break;
          }
        }
        else
        {
          // 如果只有一個交點，檢查方向是否正確
          if (Vector3d.Multiply(intP[0] - cp, dir) > 0)
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
          // 設置 slp 為倒數第二個點
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
          if (Vector3d.Multiply(intP[0] - cp, dir) > 0)
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


  public class IntersectionResult
  {
    public List<Point3d> intpList { get; set; }
    public List<Curve> Circles { get; set; }
    public Vector3d dd { get; set; }
    public Point3d fp {get; set;}

    // 構造函式
    public IntersectionResult()
    {
      intpList = new List<Point3d>();
      Circles = new List<Curve>();
      dd = new Vector3d();
      fp = new Point3d();
    }
  }


  // </Custom additional code> 
}
