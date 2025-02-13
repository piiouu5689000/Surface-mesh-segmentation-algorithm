 private void RunScript(Surface bSrf, List<Point3d> pU, List<Point3d> pV, double L, double wPD, ref object A, ref object B, ref object C, ref object D)
  {

    // 初始化數據樹
    var coordinatesTree = new DataTree<Point3d>();
    var orthoLinesTree = new DataTree<Line>();
    var weftLinesTree = new DataTree<Line>();
    var warpLinesTree = new DataTree<Line>();


    // 生成網格 ID 矩陣
    int nU = pU.Count;
    int nV = pV.Count;
    List<List<int>> gridIds = new List<List<int>>();
    List<List<int>> binIds = new List<List<int>>();

    // 創建對角遞增矩陣
    int m = 0;
    for (int i = 0; i < nU; i++)
    {
      gridIds.Add(new List<int>());
      binIds.Add(new List<int>());
      for (int j = 0; j < nV; j++)
      {
        gridIds[i].Add(0);
        binIds[i].Add(0);
      }
    }
    // 作了兩個空矩陣

    // 填充對角遞增矩陣
    for (int p = 0; p < nU + nV - 1; p++)
    {
      for (int q = Math.Max(0, p - nU + 1); q < Math.Min(p + 1, nV); q++)
      {
        gridIds[p - q][q] = m++;
      }
    }

    // 生成二元矩陣 (1,-1 pattern)
    int sign = -1;
    for (int i = 0; i < nU; i++)
    {
      if (i > 0)
      {
        if (nV % 2 == 0)
        {
          sign = -binIds[i - 1][0];
        }
      }
      for (int j = 0; j < nV; j++)
      {
        binIds[i][j] = sign;
        sign = -sign;
      }
    }

    // 儲存初始座標點
    Dictionary<int, Point3d> coordinates = new Dictionary<int, Point3d>();
    for (int i = 0; i < nU; i++)
    {
      coordinates[gridIds[i][0]] = pU[i];
    }
    for (int j = 0; j < nV; j++)
    {
      coordinates[gridIds[0][j]] = pV[j];
    }

    // 遍歷內部點生成網格
    for (int i = 1; i < nU; i++)
    {
      for (int j = 1; j < nV; j++)
      {
        // 獲取相關點的ID
        int currentId = gridIds[i][j];
        int id0 = gridIds[i - 1][j - 1];
        int id1 = gridIds[i][j - 1];
        int id2 = gridIds[i - 1][j];

        if (!coordinates.ContainsKey(id0) ||
          !coordinates.ContainsKey(id1) ||
          !coordinates.ContainsKey(id2))
          continue;

        // 獲取三個已知點
        Point3d p0 = coordinates[id0];
        Point3d p1 = coordinates[id1];
        Point3d p2 = coordinates[id2];

        // 計算對角線中點和向量
        Point3d midPoint = (p1 + p2) * 0.5;
        Vector3d diagonalVector = p2 - p1;

        // 創建圓平面
        Plane midPlane = new Plane(midPoint, diagonalVector);
        double radius = p0.DistanceTo(midPoint);
        Circle midCircle = new Circle(midPlane, radius);
        Curve circleCurve = midCircle.ToNurbsCurve();

        // 計算與曲面的交點
        Brep bsrf = Brep.CreateFromSurface(bSrf);
        Point3d[] intersectionPoints;
        Curve[] curves;
        var isIntersecting = Rhino.Geometry.Intersect.Intersection.CurveBrep(
          circleCurve,
          bsrf,
          RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,
          out curves,
          out intersectionPoints
          );

        if (isIntersecting && intersectionPoints != null && intersectionPoints.Length > 0)
        {
          // 選擇合適的交點
          Point3d newPoint = intersectionPoints[0];
          if (intersectionPoints.Length > 1)
          {
            newPoint = (p0.DistanceTo(intersectionPoints[0]) >
              p0.DistanceTo(intersectionPoints[1]))
              ? intersectionPoints[0]
              : intersectionPoints[1];
          }

          // 驗證邊長
          if (Math.Abs(newPoint.DistanceTo(p1) - L) < 0.001 &&
            Math.Abs(newPoint.DistanceTo(p2) - L) < 0.001)
          {
            // 儲存新點
            coordinates[currentId] = newPoint;

            // 創建正交線
            var quad = new Line[]
              {
              new Line(p0, p1),
              new Line(p1, newPoint),
              new Line(newPoint, p2),
              new Line(p2, p0)
              };

            // 獲取法向量
            var normals = new Vector3d[4];
            double u, v;
            bSrf.ClosestPoint(p0, out u, out v);
            normals[0] = bSrf.NormalAt(u, v);
            bSrf.ClosestPoint(p1, out u, out v);
            normals[1] = bSrf.NormalAt(u, v);
            bSrf.ClosestPoint(p2, out u, out v);
            normals[2] = bSrf.NormalAt(u, v);
            bSrf.ClosestPoint(newPoint, out u, out v);
            normals[3] = bSrf.NormalAt(u, v);

            // 添加正交線
            foreach (var line in quad)
            {
              orthoLinesTree.Add(line, new GH_Path(0));
            }

            // 添加經緯線
            int[] signs = new[]
              {
                binIds[i - 1][j - 1],
                binIds[i][j - 1],
                binIds[i - 1][j],
                binIds[i][j]
                };

            Point3d[] points = new[] { p0, p1, p2, newPoint };

            // 生成經線
            for (int k = 0; k < 4; k++)
            {
              Point3d warpPoint = points[k] + normals[k] * signs[k] * wPD * L;
              warpLinesTree.Add(
                new Line(points[k], warpPoint),
                new GH_Path(0)
                );
            }

            // 生成緯線
            for (int k = 0; k < 4; k++)
            {
              Point3d weftPoint = points[k] + normals[k] * -signs[k] * wPD * L;
              weftLinesTree.Add(
                new Line(points[k], weftPoint),
                new GH_Path(0)
                );
            }

            // 添加座標點
            coordinatesTree.Add(newPoint, new GH_Path(0));
          }
        }
      }
    }

    // 輸出結果
    A = coordinatesTree;
    B = orthoLinesTree;
    C = weftLinesTree;
    D = warpLinesTree;

  }
