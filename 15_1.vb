Option Strict Off
Option Explicit On

' 匯入必要的命名空間
Imports System
Imports System.Collections
Imports System.Collections.Generic
Imports Rhino
Imports Rhino.Geometry
Imports Grasshopper
Imports Grasshopper.Kernel
Imports Grasshopper.Kernel.Data
Imports Grasshopper.Kernel.Types
Imports System.IO
Imports System.Linq
Imports System.Data
Imports System.Drawing
Imports System.Reflection
Imports System.Windows.Forms
Imports System.Xml
Imports System.Xml.Linq
Imports Microsoft.VisualBasic
Imports System.Runtime.InteropServices
Imports Rhino.DocObjects
Imports Rhino.Collections
Imports GH_IO
Imports GH_IO.Serialization

''' <summary>
''' 這個類別是 Grasshopper Script 組件的實例，會根據需求被 Grasshopper 呼叫。
''' </summary>
Public Class Script_Instance
  Inherits GH_ScriptInstance

  #Region "Utility functions"
  ''' <summary>將字串輸出到 [Out] 參數。</summary>
  ''' <param name="text">要輸出的字串。</param>
  Private Sub Print(ByVal text As String)
    ' 具體實作在 Grasshopper 編輯模式中隱藏。
  End Sub
  ''' <summary>以格式化方式輸出字串到 [Out] 參數。</summary>
  Private Sub Print(ByVal format As String, ByVal ParamArray args As Object())
  End Sub
  ''' <summary>輸出對象的詳細資訊到 [Out] 參數。</summary>
  Private Sub Reflect(ByVal obj As Object)
  End Sub
  ''' <summary>輸出特定方法的所有重載資訊到 [Out] 參數。</summary>
  Private Sub Reflect(ByVal obj As Object, ByVal method_name As String)
  End Sub
#End Region

#Region "Members"
  ' 取得目前的 Rhino 文件
  Private Readonly RhinoDocument As RhinoDoc
  ' 取得 Grasshopper 文件
  Private Readonly GrasshopperDocument as GH_Document
  ' 取得擁有此腳本的 Grasshopper 元件
  Private Readonly Component As IGH_Component
  ' 追蹤目前的執行次數，初始為 0
  Private Readonly Iteration As Integer
#End Region

  ''' <summary>
  ''' 主要執行的函式，負責計算主曲率線。
  ''' </summary>
  Private Sub RunScript(ByVal srf As Surface, ByVal uv As Point3d, ByVal accuracy As Double, ByVal angle As Double, ByVal max As Boolean, ByVal alg As Integer, ByRef pts As Object) 
    If (srf Is Nothing) Then Return
    If (Not uv.IsValid) Then Return

    ' 限制精度以符合 Rhino 文件的公差
    accuracy = Math.Max(accuracy, doc.ModelAbsoluteTolerance)

    ' 取樣兩個方向的曲率線
    Dim dir0 As List(Of Point3d) = SampleCurvature(srf, uv, accuracy, max, angle, alg)
    Dim dir1 As List(Of Point3d) = SampleCurvature(srf, uv, accuracy, max, angle + Math.PI, alg)

    ' 移除 dir1 的第一個點以避免重複，然後反轉順序
    dir1.RemoveAt(0)
    dir1.Reverse()

    ' 合併兩個方向的結果
    dir1.AddRange(dir0)
    pts = dir1
  End Sub 

  ''' <summary>
  ''' 根據指定的參數計算主曲率線。
  ''' </summary>
  Private Function SampleCurvature(ByVal srf As Surface, ByVal uv As Point3d, ByVal accuracy As Double, ByVal max As Boolean, ByVal angle As Double, ByVal alg As Integer) As List(Of Point3d)
    Dim p As Point3d = uv
    Dim U As Interval = srf.Domain(0)
    Dim V As Interval = srf.Domain(1)
    Dim samples As New List(Of Point3d)
    
    Do
      ' 新增當前點
      samples.Add(srf.PointAt(p.X, p.Y))
      Dim dir As Vector3D

      ' 根據指定的方法計算方向
      Select Case alg
        Case 1
          dir = Euler(srf, p, max, angle, accuracy, samples)
        Case 2
          dir = ModEuler(srf, p, max, angle, accuracy, samples)
        Case 3
          dir = RK4(srf, p, max, angle, accuracy, samples)
        Case Else
          Print("選擇 1 到 3 之間的演算法編號")
      End Select

      If (IsNothing(dir)) Then Exit Do

      ' 計算下一個點的位置
      Dim s, t As Double
      Dim pt As Point3d = samples(samples.Count - 1) + dir
      If (Not srf.ClosestPoint(pt, s, t)) Then Exit Do

      ' 停止條件：
      If (samples.Count > 9999) Then Exit Do ' 避免過多樣本
      If (Not U.IncludesParameter(s, True) OrElse Not V.IncludesParameter(t, True)) Then Exit Do ' 避免超出曲面範圍
      If (Math.Abs(p.X - s) < 1e-12 AndAlso Math.Abs(p.Y - t) < 1e-12) Then Exit Do ' 避免重複點

      p.X = s
      p.Y = t
    Loop

    Return samples
  End Function

  ' 其他數值方法（Euler, ModEuler, RK4）略，同樣包含方向計算與步進策略。
  
End Class
