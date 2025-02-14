 private void RunScript(List<Point3d> u_Pt_list, List<Point3d> v_Pt_list, List<double> vList, List<double> uList, ref object A, ref object B, ref object C, ref object D, ref object E)
  {

    // 獲取 u_Pt_list 和 v_Pt_list 的長度
    int nv = v_Pt_list.Count;
    int nu = u_Pt_list.Count;
    // 初始化對角矩陣
    List<List<int>> diagride = new List<List<int>>();
    List<Point3d> test = new List<Point3d>(); // 用於存儲計算出的新點
    List<Line> vlin = new List<Line>(); // 用於存儲 V 方向的線條
    List<Line> ulin = new List<Line>(); // 用於存儲 U 方向的線條

    for(int i = 0 ; i < nv; i++)
    {
      diagride.Add(new List<int>());
      for(int j = 0; j < nu; j++)
      {
        diagride[i].Add(0);
      }
    }

    // 填充矩陣索引值
    int m = 0;
    for (int i = 0; i < nv; i++)
    {
      for (int j = 0; j < nu; j++)
      {
        diagride[i][j] = m++;
      }
    }

    // 建立點座標字典，存儲對應關係
    Dictionary<int, Point3d> coodi = new Dictionary<int, Point3d>();
    for (int i = 0; i < nv; i++)
    {
      coodi[diagride[i][0]] = v_Pt_list[i]; // 存儲 V 方向的點
    }
    for (int i = 0; i < nu; i++)
    {
      coodi[diagride[0][i]] = u_Pt_list[i]; // 存儲 U 方向的點
    }

    int c = 0; // 計數器
    for (int i = 1; i < nv; i++)
    {
      for (int j = 1; j < nu; j++)
      {
        // 確保 vList 和 uList 的索引不越界
        if (c >= vList.Count || c >= uList.Count)
        {
          Print("Warning: vList or uList index out of range.");
          continue;
        }

        // 獲取當前點的索引
        int cid = diagride[i][j];
        int id0 = diagride[i - 1][j - 1];
        int id1 = diagride[i - 1][j];
        int id2 = diagride[i][j - 1];

        // 檢查是否所有所需的點都已經存入字典
        if (!coodi.ContainsKey(id0) || !coodi.ContainsKey(id1) || !coodi.ContainsKey(id2))
          continue;

        // 獲取對應點的座標
        Point3d pt0 = coodi[id0];
        Point3d pt1 = coodi[id1];
        Point3d pt2 = coodi[id2];

        // 建立兩個圓，用來尋找交點
        Vector3d zaxis = Vector3d.ZAxis;
        Plane plan1 = new Plane(pt1, zaxis);
        Circle circle1 = new Circle(plan1, vList[c]);
        Plane plan2 = new Plane(pt2, zaxis);
        Circle circle2 = new Circle(plan2, uList[c]);

        // 計算圓的交點
        Point3d intp0;
        Point3d intp1;
        CircleCircleIntersection result = Intersection.CircleCircle(circle1, circle2, out intp0, out intp1);

        // 如果沒有交點，則跳過該點
        if (result == CircleCircleIntersection.None)
          continue;

        // 選擇合適的交點
        Point3d newp = Point3d.Unset;
        if (intp0.IsValid && intp1.IsValid)
        {
          newp = (pt0.DistanceTo(intp0) > pt0.DistanceTo(intp1)) ? intp0 : intp1;
        }

        // 存儲新點座標
        coodi[cid] = newp;
        test.Add(newp);

        // 建立 U 和 V 方向的線條
        vlin.Add(new Line(pt0, pt1));
        vlin.Add(new Line(pt2, newp));
        ulin.Add(new Line(pt0, pt2));
        ulin.Add(new Line(pt1, newp));

        c++; // 計數器遞增
      }
    }

    // 輸出結果
    C = test; // 交點列表
    D = vlin; // V 方向的線條
    E = ulin; // U 方向的線條
  }
