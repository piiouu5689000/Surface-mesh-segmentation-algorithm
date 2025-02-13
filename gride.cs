using Rhino.Geometry.Intersect;
private void RunScript(Brep bSrf, List<Point3d> pU, List<Point3d> pV, double L, double wPD, ref object A, ref object B)
  {

    List<Point3d> test = new List<Point3d>();

    // 做一個對角矩陣
    List<List<int>> diagride = new List<List<int>>();
    int U = pU.Count;
    int V = pV.Count;

    // 作一個 U 行 V 列的矩陣
    for(int i = 0; i < V; i++)
    {
      diagride.Add(new List<int>()); // 先加入 V 次列表
      for(int j = 0; j < U; j++)
      {
        diagride[i].Add(0);  // 再第 i 個列表加入0，共U次
      }
    }

    // 按對角方式填入值，作為字典的標籤
    int m = 0;
    for(int i = 0; i < U + V - 1; i++)
    {
      for(int j = Math.Max(0, i - V + 1); j < Math.Min(i + 1, U); j++)
      {
        diagride[i - j][j] = m++;
      }
    }

  // 作一個字典，填入第一行與第一列的點，與第一行與第一列的標籤  
  Dictionary<int, Point3d> coodi = new Dictionary<int, Point3d>();
    for(int i = 0; i < V; i++)
    {
      coodi[diagride[i][0]] = pV[i];
    }
    for(int i = 0; i < U; i++)
    {
      coodi[diagride[0][i]] = pU[i];
    }


    for(int i = 1; i < V; i++)
    {
      for(int j = 1; j < U; j++)
      {
        // 以方形的關係，在對角矩陣取值，作為字典的標籤值
        int cpid = diagride[i][j];
        int id0 = diagride[i - 1][j - 1];
        int id1 = diagride[i][j - 1];
        int id2 = diagride[i - 1][j];

        // 若字典中該標籤沒有點，就跳過
        if(!coodi.ContainsKey(id0) ||
          !coodi.ContainsKey(id1) ||
          !coodi.ContainsKey(id2))
          continue;

        // 根據標籤選出三個點
        Point3d pt0 = coodi[id0];
        Point3d pt1 = coodi[id1];
        Point3d pt2 = coodi[id2];

        Point3d midp = (pt1 + pt2) * 0.5;
        Vector3d cdir = pt2 - pt1;
        double radius = pt0.DistanceTo(midp);

        Plane plan = new Plane(midp, cdir);
        Circle circle = new Circle(plan, radius);
        Curve cir = circle.ToNurbsCurve();

        Curve[] curv;
        Point3d[] intp;

        if(!Intersection.CurveBrep(cir, bSrf, 0.01, out curv, out intp)) break;

        Point3d newpt = new Point3d();// 局部變數在「所有可能的程式路徑」中都必須被賦值後才能使用！
        if(intp.Length > 1)
        {
          if( pt0.DistanceTo(intp[0]) > pt0.DistanceTo(intp[1]))
          {
            newpt = intp[0];
          }
          else
          {
            newpt = intp[1];
          }
        }

        coodi[cpid] = newpt;
        test.Add(newpt);

      }
    }

    A = test;

  }
