 private void RunScript(List<Point3d> pt_list, Surface srf, int nu, int nv, double L, int it, double Lmax, double Lmin, ref object A, ref object B, ref object C, ref object D)
  {

    // 初始化變數
    List<List<int>> keygride = new List<List<int>>(); // 存儲點的索引矩陣
    List<Point3d> optimizedPts = new List<Point3d>(); // 優化後的點
    List<Point3d> Pt_onSrf = new List<Point3d>(); // 投影到曲面的點
    List<Line> vlin = new List<Line>(); // 垂直方向的連接線
    List<Line> ulin = new List<Line>(); // 水平方向的連接線
    const double tolerance = 0.001; // 迭代收斂容差
    double damping = 0.02; // 阻尼係數
    double relaxation = 0.8; // 放鬆因子

    // 初始化索引矩陣，所有值預設為0
    for (int i = 0; i < nv; i++) {
      keygride.Add(new List<int>());
      for (int j = 0; j < nu; j++) {
        keygride[i].Add(0);
      }
    }

    // 給每個點分配唯一索引值
    int m = 0;
    for (int i = 0; i < nv; i++) {
      for (int j = 0; j < nu; j++) {
        keygride[i][j] = m;
        m++;
      }
    }

    // 投影點到曲面
    int p = 0;
    for (int i = 0; i < nv; i++) {
      for (int j = 0; j < nu; j++) {
        double s = 0, t = 0;
        if (!srf.ClosestPoint(pt_list[p], out s, out t)) continue;
        Pt_onSrf.Add(srf.PointAt(s, t));
        p++;
      }
    }

    // 建立點索引到座標的對應關係
    Dictionary<int, Point3d> coodi = new Dictionary<int, Point3d>();
    int k = 0;
    if (pt_list.Count != nu * nv) return;

    for (int i = 0; i < nv; i++) {
      for (int j = 0; j < nu; j++) {
        coodi[keygride[i][j]] = Pt_onSrf[k];
        k++;
      }
    }

    double prevMaxAdjustment = double.MaxValue;
    int stabilityCount = 0;

    // 迭代優化
    for (int iter = 0; iter < it; iter++) {
      vlin.Clear();
      ulin.Clear();
      double maxAdjustment = 0.0;
      Dictionary<int, Vector3d> adjustments = new Dictionary<int, Vector3d>();

      for (int i = 0; i < nv; i++) {
        for (int j = 0; j < nu; j++) {
          int cid = keygride[i][j];
          if (!coodi.ContainsKey(cid)) continue;

          Point3d cpt = coodi[cid];
          List<int> neighbors = new List<int>(); // 存儲相鄰索引
          if (j < nu - 1) neighbors.Add(keygride[i][j + 1]);
          if (i < nv - 1) neighbors.Add(keygride[i + 1][j]);
          if (j > 0) neighbors.Add(keygride[i][j - 1]);
          if (i > 0) neighbors.Add(keygride[i - 1][j]);

          Vector3d totalAdjustment = Vector3d.Zero;
          double adjustedDamping = damping * (1.0 + (double) j / nu);  // 內圈點阻尼較高
          foreach (int nid in neighbors) {
            if (!coodi.ContainsKey(nid)) continue;

            Point3d npt = coodi[nid];
            double r = cpt.DistanceTo(npt);
            Vector3d dir = (npt - cpt) / r;
            double factor = Math.Min(0.5, Math.Abs(r - L) / L);

            if (r < Lmin) totalAdjustment += dir * (Lmin - r) * factor * 0.5; // 內縮速度降低
            else if (r > Lmax) totalAdjustment += dir * (Lmax - r) * factor * 0.8; // 外擴速度稍微提高
          }

          Vector3d dampedAdjustment = totalAdjustment * adjustedDamping * relaxation;

          if (j == 0 || j == nu - 1)  // 邊界點的額外限制
            dampedAdjustment *= 0.5;

          adjustments[cid] = dampedAdjustment;
          maxAdjustment = Math.Max(maxAdjustment, dampedAdjustment.Length);
        }
      }

      foreach (var kvp in adjustments) {
        int cid = kvp.Key;
        Vector3d adjustment = kvp.Value;
        Point3d currentPt = coodi[cid];
        Point3d newp = currentPt + adjustment;

        double s = 0, t = 0;
        if (!srf.ClosestPoint(newp, out s, out t)) continue;

        Point3d finalPt = srf.PointAt(s, t);
        coodi[cid] = 0.9 * finalPt + 0.1 * currentPt; // 平滑移動
      }

      if (maxAdjustment < tolerance) {
        Print(string.Format("在第 {0} 次迭代後收斂，最大調整量: {1}", iter + 1, maxAdjustment));
        break;
      }

      if (Math.Abs(maxAdjustment - prevMaxAdjustment) < tolerance * 0.02) {
        stabilityCount++;
        if (stabilityCount > 5) {
          Print(string.Format("在第 {0} 次迭代後達到穩定狀態，最大調整量: {1}", iter + 1, maxAdjustment));
          break;
        }
      } else {
        stabilityCount = 0;
      }

      prevMaxAdjustment = maxAdjustment;
    }

    // 儲存最終結果
    foreach (var kvp in coodi) optimizedPts.Add(kvp.Value);
    A = Pt_onSrf;
    B = optimizedPts;
    C = vlin;
    D = ulin;

  }

  // <Custom additional code> 

  private bool IsOuterPoint(int i, int j, int nu, int nv)
  {
    return i == 0 || i == nv - 1 || j == 0 || j == nu - 1;
  }

  // </Custom additional code> 
}
