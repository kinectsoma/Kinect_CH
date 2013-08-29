using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
//
using Microsoft.Kinect; // 레퍼런스에 Microsoft.Kinect 추가 
using Microsoft.Kinect.Toolkit;
using Microsoft.Kinect.Toolkit.FaceTracking;
using Point = System.Windows.Point;
using System.Windows.Threading;
namespace KD01
{
    /// <summary>
    /// MainWindow.xaml에 대한 상호 작용 논리
    /// </summary>
    public partial class MainWindow : Window
    {
        private readonly KinectSensorChooser sensorChooser = new KinectSensorChooser();
        System.Windows.Point point = new System.Windows.Point();

        private byte[] colorImage;

        private ColorImageFormat colorImageFormat = ColorImageFormat.Undefined;

        private short[] depthImage;

        private DepthImageFormat depthImageFormat = DepthImageFormat.Undefined;

        private Skeleton[] skeletonData;
        private FaceTracker faceTracker;

        private EnumIndexableCollection<FeaturePoint, PointF> facePoints;

        KinectSensor frontSensor = null;
        KinectSensor obstaSensor = null;
        private static FaceTriangle[] faceTriangles;

        //Image image1 = new Image();

        private Point nosePoint = new Point();
        //장애물 센서 변수
        rectPoint[,] _rectPoint;
        Rectangle[,] rect;
        Image[] imgAlert = new Image[9];

        int[,] obsCell = new int[3, 3];
        int m_row = 12, m_col = 12;

        int[] savePoint = new int[2];
        //
        bool[] frameProccessed = new bool[3];
        struct rectPoint
        {
            public int min_X, min_Y, max_X, max_Y;
        }

       // DispatcherTimer alertTimer = new DispatcherTimer();

        public MainWindow()
        {
            //image1.Width = 640;
            //image1.Height = 480;
            //alertTimer.Interval = TimeSpan.FromSeconds(2d);
            //alertTimer.Tick += delegate
            //{
            //    new System.Threading.Thread(() => Console.Beep()).Start();
            //    alertTimer.Stop();
            //};
            InitializeComponent();

            for (int i = 0; i < frameProccessed.Length; i++)
            {
                frameProccessed[i] = false;
            }

            InitalizeNui();
        }
        public void InitalizeNui()
        {
            frontSensor = KinectSensor.KinectSensors[0];

            frontSensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            frontSensor.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);
            try
            {
                // This will throw on non Kinect For Windows devices.
                frontSensor.DepthStream.Range = DepthRange.Near;
                frontSensor.SkeletonStream.EnableTrackingInNearRange = true;
            }
            catch (InvalidOperationException)
            {
                frontSensor.DepthStream.Range = DepthRange.Default;
                frontSensor.SkeletonStream.EnableTrackingInNearRange = false;
            }

            frontSensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
            frontSensor.SkeletonStream.Enable();
            frontSensor.AllFramesReady += KinectSensorOnAllFramesReady;
            frontSensor.ColorFrameReady += nui_ColorFrameReady;

            frontSensor.Start();

            //장애물 센터
           
            rect = new Rectangle[m_col, m_col];
            _rectPoint = new rectPoint[m_col, m_row];
            InitRectangle(rect, m_col, m_row, Colors.Blue);

            for (int i = 0; i < imgAlert.Length; i++)
            {
                imgAlert[i] = new Image();
                imgAlert[i].Source = new BitmapImage(new Uri("pack://application:,,,/KD01;component/AlertImage.png"));
                imgAlert[i].Stretch = Stretch.Fill;
                imgAlert[i].Height = 160;
                imgAlert[i].Width = 213;
                rootCanvas.Children.Add(imgAlert[i]);
            }

            obstaSensor = KinectSensor.KinectSensors[1];
            obstaSensor.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);
            obstaSensor.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(obstaSensor_DepthFrameReady);
            obstaSensor.DepthStream.Range = DepthRange.Near;
            obstaSensor.Start();

    
        }
        private void KinectSensorOnAllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            if (frameProccessed[1] == false)
            {
                frameProccessed[1] = true;
            }
            else
            {
                frameProccessed[1] = false;
                return;
            }
            ColorImageFrame colorImageFrame = null;
            DepthImageFrame depthImageFrame = null;
            SkeletonFrame skeletonFrame = null;

            try
            {

                colorImageFrame = e.OpenColorImageFrame();
                depthImageFrame = e.OpenDepthImageFrame();
                skeletonFrame = e.OpenSkeletonFrame();

                if (colorImageFrame == null || depthImageFrame == null || skeletonFrame == null)
                {
                    return;
                }

                if (this.depthImageFormat != depthImageFrame.Format)
                {
                    this.depthImage = null;
                    this.depthImageFormat = depthImageFrame.Format;
                }

                if (this.colorImageFormat != colorImageFrame.Format)
                {
                    this.colorImage = null;
                    this.colorImageFormat = colorImageFrame.Format;
                }

                if (this.depthImage == null)
                {
                    this.depthImage = new short[depthImageFrame.PixelDataLength];
                }

                if (this.colorImage == null)
                {
                    this.colorImage = new byte[colorImageFrame.PixelDataLength];
                }

                if (this.skeletonData == null || this.skeletonData.Length != skeletonFrame.SkeletonArrayLength)
                {
                    this.skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                }

                colorImageFrame.CopyPixelDataTo(this.colorImage);
                depthImageFrame.CopyPixelDataTo(this.depthImage);
                skeletonFrame.CopySkeletonDataTo(this.skeletonData);
            }
            finally
            {
                if (colorImageFrame != null)
                {
                    colorImageFrame.Dispose();
                }

                if (depthImageFrame != null)
                {
                    depthImageFrame.Dispose();
                }

                if (skeletonFrame != null)
                {
                    skeletonFrame.Dispose();
                }

                using (depthImageFrame)
                {
                    if (depthImageFrame != null && skeletonData != null)
                    {
                        foreach (Skeleton sd in skeletonData)
                        {
                            if (sd.TrackingState == SkeletonTrackingState.Tracked || sd.TrackingState == SkeletonTrackingState.PositionOnly)
                            {
                                Joint joint = sd.Joints[JointType.Head];

                                DepthImagePoint depthPoint;
                                CoordinateMapper coordinateMapper = new CoordinateMapper(frontSensor);
                                depthPoint = coordinateMapper.MapSkeletonPointToDepthPoint(joint.Position, DepthImageFormat.Resolution320x240Fps30);

                                point = new System.Windows.Point((int)(frontSensor.ColorStream.FrameWidth * depthPoint.X
                                                                   / depthImageFrame.Width),
                                                        (int)(frontSensor.ColorStream.FrameHeight * depthPoint.Y
                                                                   / depthImageFrame.Height));

                                /* textBlock1.Text = string.Format("X:{0:0.00} Y:{1:0.00} Z:{2:0.00}",
                                                                point.X,
                                                                point.Y,
                                                                joint.Position.Z); */

                                Canvas.SetLeft(headEllipse, point.X - headEllipse.Width / 2);
                                Canvas.SetTop(headEllipse, point.Y - headEllipse.Height / 2);

                                if (this.faceTracker == null)
                                {
                                    try
                                    {
                                        this.faceTracker = new FaceTracker(frontSensor);
                                    }
                                    catch (InvalidOperationException)
                                    {
                                        // During some shutdown scenarios the FaceTrack
                                        // is unable to be instantiated.  Catch that exception
                                        // and don't track a face.
                                        this.faceTracker = null;
                                    }
                                }
                                if (this.faceTracker != null)
                                {
                                    FaceTrackFrame frame = this.faceTracker.Track(
                                        colorImageFormat, colorImage, depthImageFormat, depthImage, sd);

                                    if (frame.TrackSuccessful)
                                    {
                                        faceTriangles = frame.GetTriangles();
                                        this.facePoints = frame.GetProjected3DShape();

                                        var faceModelPts = new List<Point>();
                                        var faceModel = new List<FaceModelTriangle>();


                                        for (int i = 0; i < this.facePoints.Count; i++)
                                        {
                                            faceModelPts.Add(new Point(this.facePoints[i].X + 0.5f, this.facePoints[i].Y + 0.5f));
                                        }

                                        foreach (var t in faceTriangles)
                                        {
                                            var triangle = new FaceModelTriangle();
                                            triangle.P1 = faceModelPts[t.First];
                                            //triangle.P2 = faceModelPts[t.Second];
                                            //triangle.P3 = faceModelPts[t.Third];
                                            faceModel.Add(triangle);
                                        }

                                        Canvas.SetLeft(noseEllipse, faceModel[108].P1.X - noseEllipse.Width / 2);
                                        Canvas.SetTop(noseEllipse, faceModel[108].P1.Y - noseEllipse.Height / 2);
                                        nosePoint = new Point(faceModel[108].P1.X, faceModel[108].P1.Y);
                                    }
                                }
                            }
                        }
                    }
                }

                getAttentionAngle(nosePoint);
            }
        }

        void nui_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            if (frameProccessed[0] == false)
            {
                frameProccessed[0] = true;
            }
            else
            {
                frameProccessed[0] = false;
                return;
            }
            ColorImageFrame ImageParam = e.OpenColorImageFrame();

            if (ImageParam == null) return;

            byte[] ImageBits = new byte[ImageParam.PixelDataLength];
            ImageParam.CopyPixelDataTo(ImageBits);

            BitmapSource src = null;
            src = BitmapSource.Create(ImageParam.Width, ImageParam.Height,
                                        96, 96, PixelFormats.Bgr32, null,
                                        ImageBits,
                                        ImageParam.Width * ImageParam.BytesPerPixel);
            image1.Source = src;
        }

        public void getAttentionAngle(System.Windows.Point nosePoint)
        {
            double xDiff = nosePoint.X - point.X;
            double yDiff = nosePoint.Y - point.Y;
            StringBuilder sb = new StringBuilder();

            if (xDiff >= 38) { sb.Append("오른쪽"); }
            else if (xDiff <= 0) { sb.Append("왼쪽"); }
            else { sb.Append("가운데"); }

            if (yDiff <= 35) { sb.Append(" 위"); }
            else if (yDiff >= 68) { sb.Append(" 아래"); }

            textBlock2.Text = sb.ToString();
            //Console.WriteLine(frontSensor.UniqueKinectId);
            //USB\VID_045E&PID_02BF\3 ??????
        }

        #region 장애물 센서
        void obstaSensor_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            if (frameProccessed[2] == false)
            {
                frameProccessed[2] = true;
            }
            else
            {
                frameProccessed[2] = false;
                return; 
            }
            DepthImageFrame ImageParam = e.OpenDepthImageFrame();

            if (ImageParam == null) return;

            short[] ImageBits = new short[ImageParam.PixelDataLength];
            ImageParam.CopyPixelDataTo(ImageBits);

            WriteableBitmap wb = new WriteableBitmap(ImageParam.Width,
                                                     ImageParam.Height,
                                                     96, 96,
                                                     PixelFormats.Bgr32, null);

            wb.WritePixels(new Int32Rect(0, 0,
                                            ImageParam.Width,
                                            ImageParam.Height),
                                            GetRGB(ImageParam, ImageBits, obstaSensor.DepthStream),
                            ImageParam.Width * 4,
                            0);
            changeRectSize();
            image2.Source = wb;
            int[,] maxValue = new int[9, 3];
            int obsCount = 0;


            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (obsCell[i, j] > 0)
                    {
                        maxValue[obsCount, 0] = obsCell[i, j];
                        maxValue[obsCount, 1] = i;
                        maxValue[obsCount, 2] = j;
                        obsCount++;
                    }
                }
            }
            for (int i = 0; i < imgAlert.Length; i++)
            {
                if (maxValue[i, 0] > 0)
                {
                    Canvas.SetLeft(imgAlert[i], (image2.Width / 3) * maxValue[i, 1] + Canvas.GetLeft(image2));
                    Canvas.SetTop(imgAlert[i], (image2.Height / 3) * maxValue[i, 2]);
                    imgAlert[i].Visibility = System.Windows.Visibility.Visible;
                }
                else
                {
                    imgAlert[i].Visibility = Visibility.Collapsed;
                }
            }
        }

        void SetRGB(byte[] nPlayers, int nPos, byte r, byte g, byte b)
        {
            nPlayers[nPos + 2] = r;
            nPlayers[nPos + 1] = g;
            nPlayers[nPos + 0] = b;
        }

        public void InitRectangle(Rectangle[,] rectangle, int Col, int Row, Color StrokeColor)
        {
            for (int i = 0; i < Col; i++)
            {
                for (int j = 0; j < Row; j++)
                {
                    rectangle[i, j] = new Rectangle();
                    rectangle[i, j].Width = 0;
                    rectangle[i, j].Height = 0;
                    rectangle[i, j].StrokeThickness = 5;
                    rectangle[i, j].Stroke = new SolidColorBrush(StrokeColor);
                    rectangle[i, j].Opacity = 0.7;
                    rootCanvas.Children.Add(rectangle[i, j]);
                }
            }
        }

        byte[] GetRGB(DepthImageFrame PImage, short[] depthFrame, DepthImageStream depthStream)
        {
            obsCell = new int[3, 3];
            byte[] rgbs = new byte[PImage.Width * PImage.Height * 4];

            int nSavei32 = 0;

            for (int i = 0; i < m_col; i++)
            {
                for (int j = 0; j < m_row; j++)
                {
                    _rectPoint[i, j].min_X = int.MaxValue;
                    _rectPoint[i, j].min_Y = int.MaxValue;
                    _rectPoint[i, j].max_X = int.MinValue;
                    _rectPoint[i, j].max_Y = int.MinValue;
                }
            }

            for (int i16 = 0, i32 = 0; i16 < depthFrame.Length && i32 < rgbs.Length; i16++, i32 += 4)
            {
                int nDistance = depthFrame[i16] >>
                                DepthImageFrame.PlayerIndexBitmaskWidth;

                int nX = i16 % PImage.Width;
                int nY = i16 / PImage.Width;


                int i = (nY - 1) / (PImage.Height / m_col);
                int j = (nX - 1) / (PImage.Width / m_row);



                int nColor = 0xFF * nDistance / 4000;
                if (nDistance >= 0 && nDistance <= 1000)
                {
                    if (!((i == m_col) || (j == m_row)))
                    {
                        _rectPoint[i, j].min_X = Math.Min(_rectPoint[i, j].min_X, nX);
                        _rectPoint[i, j].min_Y = Math.Min(_rectPoint[i, j].min_Y, nY);
                        _rectPoint[i, j].max_X = Math.Max(_rectPoint[i, j].max_X, nX);
                        _rectPoint[i, j].max_Y = Math.Max(_rectPoint[i, j].max_Y, nY);
                        obsCell[j / (m_row / 3), i / (m_col / 3)] += 1;
                    }
                    nSavei32 = i32;
                    SetRGB(rgbs, i32, 0xff, 0, 0);
                }
                else SetRGB(rgbs, i32, (byte)nColor, (byte)nColor, (byte)nColor);
            }

            SetRGB(rgbs, nSavei32, 0xFF, 0, 0);
            return rgbs;
        }

        void changeRectSize()
        {
            for (int i = 0; i < m_col; i++)
            {
                for (int j = 0; j < m_row; j++)
                {
                    Canvas.SetLeft(rect[i, j], _rectPoint[i, j].min_X * 2 + Canvas.GetLeft(image2));
                    Canvas.SetTop(rect[i, j], _rectPoint[i, j].min_Y * 2);

                    int nW = (_rectPoint[i, j].max_X - _rectPoint[i, j].min_X) * 2;
                    int nH = (_rectPoint[i, j].max_Y - _rectPoint[i, j].min_Y) * 2;
                    rect[i, j].Width = nW < 0 ? rect[i, j].Width : nW;
                    rect[i, j].Height = nH < 0 ? rect[i, j].Height : nH;
                }
            }
        }
        #endregion
    }
   
    public struct FaceModelTriangle
    {
        public Point P1;
        public Point P2;
        public Point P3;
    }
}
