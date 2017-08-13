//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Windows.Controls;
    using System.Windows.Shapes;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;
        
        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup,drawingGroup2;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource,imageSource3;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth,displayWidthDepth;

        private ColorFrameReader colorFrameReader = null;
        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight, displayHeightDepth;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;
        //private string testing = null;
      
       // DrawingContext dc2 ;
        private List<Point> trackingPencil = new List<Point>();
        DrawingContext dc,dc2;
        private Point topLeft, bottomRight;
        int framecount=0,touchCount=0;
        ushort[] depthData;
        DepthFrame frame;
        double PrimaryScreenWidth = SystemParameters.PrimaryScreenWidth;
        double PrimaryScreenHeight = SystemParameters.PrimaryScreenHeight;
        private DepthSpacePoint[] colorMappedToDepthPoints = null;
        int framerate = 0;
        Point prevPosi, currPosi;
        int test = 0;
        String flagStat= "";
        Double Zup = 3;
        Double Zdown = 1.6;
        Boolean gesture = false ;
        Double currdistance, prevdistance;
        int count = 1;
       // Canvas canvasPanel;
        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;
        /// <summary>nce of the MainWindow class.
        /// </summary>
        /// Initializes a new insta
       // public MainWindow(Point tL, Point bR)
//        {
            
  //      }
        public MainWindow(Point tL, Point bR)
        {
            this.topLeft = tL;
            this.bottomRight = bR;
            Debug.WriteLine("Main Window "+tL + bR );
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

          
            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            FrameDescription frameDescriptioncolor = this.kinectSensor.ColorFrameSource.FrameDescription;
            // get size of joint space
            this.displayWidth = frameDescriptioncolor.Width;
            this.displayHeight = frameDescriptioncolor.Height;

            this.displayWidthDepth = frameDescription.Width;
            this.displayHeightDepth = frameDescription.Height;

            depthData = new ushort[frameDescription.LengthInPixels];

            this.colorMappedToDepthPoints = new DepthSpacePoint[displayWidth * displayHeight];
            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
      //      this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

          
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = (this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText 
                                                          : Properties.Resources.NoSensorStatusText);

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();
            this.drawingGroup2 = new DrawingGroup();
            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);
          

            // use the window object as the view model in this simple example
            this.DataContext = this;
            // initialize the components (controls) of the window
            this.InitializeComponent();

                     
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }
        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource2
        {
            get
            {
                return this.colorBitmap;
            }
        }
      

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        public object RootWindow { get; private set; }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        private void Gesture_Click(object sender, RoutedEventArgs e)
        {
            if (gesture)
            {
                Gesture.Content = "Gesture Off";
                gesture = false;
            }
            else
            {
                Gesture.Content = "Gesture On";
                gesture = true; }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                        }

                        this.colorBitmap.Unlock();
                    }
                }
            }
        }
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
          //  if (framerate /2 == 0)
            {
            //    if (framerate == 30)
                {
              //      framerate = 0;
                }
       /*         framerate++;
                using (DepthImageFrame df = e.FrameReference.AcquireFrame())
                {
                    short[] pixelData = new short[df.PixelDataLength];
                    int stride = df.Width * 2;
                    df.CopyPixelDataTo(pixelData);

                   
                }*/
                using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
                {
                    if (bodyFrame != null)
                    {
                        if (this.bodies == null)
                        {
                            this.bodies = new Body[bodyFrame.BodyCount];
                        }

                        // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                        // As long as those body objects are not disposed and not set to null in the array,
                        // those body objects will be re-used.
                        bodyFrame.GetAndRefreshBodyData(this.bodies);
                        dataReceived = true;
                    }
                }

                if (dataReceived)
                {
                    using (DrawingContext dc = this.drawingGroup.Open())
                    {
                        // Draw a transparent background to set the render size
                        Pen p12 = new Pen(Brushes.Blue, 5);
                  //      dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, 1366, 768));
                  //      dc.DrawRectangle(null, p12, new Rect(0.0, 0.0,1366, 768));
                        
                        int penIndex = 0;
                        foreach (Body body in this.bodies)
                        {
                            Pen drawPen = this.bodyColors[penIndex++];

                            if (body.IsTracked)
                            {
                                // this.DrawClippedEdges(body, dc);

                                IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                                Pen p13 = new Pen(Brushes.Red, 5);
                                if(topLeft.X != 0 && topLeft.Y!=0 && bottomRight.X !=0 && bottomRight.Y!=0)
                                {
                    //                dc.DrawRectangle(null, p13, new Rect(topLeft.X, topLeft.Y, (-1) * topLeft.X + bottomRight.X, bottomRight.Y - topLeft.Y));
                                }// convert the joint points to depth (display) space
                                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                            //    Dictionary<JointType, Point> jointPointsDepth = new Dictionary<JointType, Point>();
                                foreach (JointType jointType in joints.Keys)
                                {
                                    // sometimes the depth(Z) of an inferred joint may show as negative
                                    // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                   

                                    CameraSpacePoint position = joints[jointType].Position;
                                    if (position.Z < 0)
                                    {
                                        position.Z = InferredZPositionClamp;
                                    }

                                    ColorSpacePoint colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);
                                    DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);

                                    jointPoints[jointType] = new Point((int)colorSpacePoint.X, (int)colorSpacePoint.Y);
                            //        jointPointsDepth[jointType] = new Point((int)depthSpacePoint.X, (int)depthSpacePoint.Y);
                                    //GetDepthOF Hands
                                   
                                    if (jointType == JointType.HandTipLeft || jointType == JointType.HandTipRight)
                                    {
                                       
                                        // int temp = depthData[(int)jointPointsDepth[jointType].X + (int)jointPointsDepth[jointType].Y * (int)this.displayWidthDepth] >> 3;
                                        String depthOfHand = (position.Z).ToString() + jointType;// + " " + temp.ToString()+" " + framecount.ToString();// depthData[(int)jointPoints[jointType].X + (int)jointPoints[jointType].Y * (int)this.displayWidth].ToString();
                                        
                                        FormattedText ft = new FormattedText(depthOfHand, CultureInfo.CurrentCulture, FlowDirection.LeftToRight, new Typeface(new FontFamily("Century"), FontStyles.Normal, FontWeights.Bold, FontStretches.Normal), 36, Brushes.White);
                                        textBox1.Text = depthOfHand;
                                        //            dc.DrawText(ft, new Point(jointPoints[jointType].X + 15, jointPoints[jointType].Y + 15));

                                        if (position.Z > Zdown && position.Z < Zup)
                                        {



                                            if (jointPoints[jointType].X >= topLeft.X && jointPoints[jointType].Y >= topLeft.Y && jointPoints[jointType].X <= bottomRight.X && jointPoints[jointType].Y <= bottomRight.Y)
                                            {
                                                double conversion_factorForX = this.PrimaryScreenWidth / (bottomRight.X - topLeft.X);
                                                double conversion_factorForY = this.PrimaryScreenHeight / (bottomRight.Y - topLeft.Y);
                                                double newX = (int)(jointPoints[jointType].X - topLeft.X) * conversion_factorForX;
                                                double newY = (int)(jointPoints[jointType].Y - topLeft.Y) * conversion_factorForY;
                                                //Debug.WriteLine("Joint" + jointPoints[jointType]);
                                                prevPosi = currPosi;
                                                Point coordi = new Point(1920 - newX, newY);
                                                jointPoints[jointType] = coordi;
                                                currPosi = coordi;
                                                //textBox1.Text=coordi.ToString();
                                                //  Debug.WriteLine(SystemParameters.PrimaryScreenWidth.ToString());
                                                //  Debug.WriteLine(SystemParameters.PrimaryScreenHeight.ToString());

                                                /*  Debug.WriteLine("top left"+topLeft.ToString());
                                                  Debug.WriteLine(bottomRight.ToString());
                                                 */
                                                //Debug.WriteLine("coordi" + coordi.ToString());
                                                // FormattedText ft1 = new FormattedText("Touched", CultureInfo.CurrentCulture, FlowDirection.LeftToRight, new Typeface(new FontFamily("Century"), FontStyles.Normal, FontWeights.Bold, FontStretches.Normal), 12, Brushes.White);
                                                // dc.DrawText(ft1, new Point(jointPoints[jointType].X, jointPoints[jointType].Y));
                                                Pen p1 = new Pen(Brushes.Blue, 5);
                                                Pen p2 = new Pen(Brushes.Red, 5);
                                                Pen p3 = new Pen(Brushes.Yellow, 5);

                                                //                      dc.DrawEllipse(null, p1, coordi, 20, 20);
                                                //                    dc.DrawEllipse(null, p2, coordi, 35, 35);
                                                //                  dc.DrawEllipse(null, p3, coordi, 55, 55
                                                if (gesture) {
                                                    //     prevdistance = currdistance;
                                                    //        currdistance = Math.Sqrt(Math.Pow(currPosi.X - prevPosi.X, 2) + Math.Pow(currPosi.Y - prevPosi.Y, 2));
                                                    Image slider = new Image();
                                                    
                                                    Canvas.SetLeft(slider, 300);
                                                    Canvas.SetTop(slider, 350);
                                                    slider.Width = 750;
                                                    slider.Height = 300;
                                                    canvasPanel.Children.Add(slider);
                                                    if (currPosi.X > prevPosi.X && currPosi.Y - prevPosi.Y<200)
                                                    {
                                                        String cts = (count%6).ToString();
                                                        slider.Source = new BitmapImage(new Uri(@"Images" + "/pic" + cts + ".jpg", UriKind.RelativeOrAbsolute));
                                                        TouchStat.Text="Sliding";
                                                        count++;

                                                    }
                                                  else if (currPosi.Y > prevPosi.Y && currPosi.X - prevPosi.X <200  ) { TouchStat.Text = "Scrolling"; }


                                                }

                                                else
                                                {
                                                    if (position.Z - Zdown <= 0.3)
                                                    {

                                                          Ellipse brushEllipse = new Ellipse();
                                                           brushEllipse.Height = 15;
                                                           brushEllipse.Width = 15;
                                                           brushEllipse.Fill = new SolidColorBrush(Colors.White);
                                                           Canvas.SetLeft(brushEllipse, coordi.X);
                                                           Canvas.SetTop(brushEllipse, coordi.Y);
                                                     /*   Canvas.SetLeft(pointer, coordi.X);
                                                        Canvas.SetTop(pointer, coordi.Y);*/
                                                        canvasPanel.Background = Brushes.Blue;
                                                        canvasPanel.Children.Clear();
                                                        canvasPanel.Children.Add(TouchStat);
                                                        TouchStat.Text = "Hovering";
                                                        // canvasPanel.Children.Add(pointer);
                                                        canvasPanel.Children.Add(brushEllipse);

                                                    }
                                                    else
                                                    {

                                                        // Add Child Elements to Canvas
                                                        currPosi = coordi;

                                                        canvasPanel.Background = Brushes.Green;
                                                        /*          if (jointType == JointType.HandTipRight )
                                                                  {
                                                                  */
                                                          Ellipse brushEllipse = new Ellipse();
                                                          brushEllipse.Height = 15;
                                                          brushEllipse.Width = 15;
                                                          brushEllipse.Fill = new SolidColorBrush(Colors.Red);
                                                          Canvas.SetLeft(brushEllipse, coordi.X);
                                                          Canvas.SetTop(brushEllipse, coordi.Y);
                                                     /*   Canvas.SetLeft(pointer, coordi.X);
                                                        Canvas.SetTop(pointer, coordi.Y);*/
                                                        canvasPanel.Children.Clear();
                                                        canvasPanel.Children.Add(TouchStat);
                                                        TouchStat.Text = "Touched";
                                                        canvasPanel.Background = Brushes.Green;
                                                        //       canvasPanel.Children.Add(pointer);
                                                        canvasPanel.Children.Add(brushEllipse);
                                                        /*  Rectangle redRectangle = new Rectangle();
                                                           redRectangle.Width = 20;
                                                           redRectangle.Height = 20;
                                                           redRectangle.Stroke = new SolidColorBrush(Colors.Blue);
                                                           redRectangle.StrokeThickness = 1;
                                                           redRectangle.Fill = new SolidColorBrush(Colors.Red);
                                                           // Set Canvas position
                                                           Canvas.SetLeft(redRectangle, coordi.X);
                                                           Canvas.SetTop(redRectangle, coordi.Y);

                                                          // Add Rectangle to Canvas

                                                         Canvas.SetLeft(brushEllipse, coordi.X);
                                                          Canvas.SetTop(brushEllipse, coordi.Y);
                                                          canvasPanel.Children.Add(brushEllipse);
                                                          //Debug.WriteLine("Rectangle");
                                                      }*/

                                                        /*     if (jointPoints[jointType].X >= 100 && jointPoints[jointType].X <= 200 && jointPoints[jointType].Y >= 380 && jointPoints[jointType].Y <= 480)
                                                             {

                                                                 if (!flagStat.Equals("BoundA"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("A");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundA";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 250 && jointPoints[jointType].X <= 350 && jointPoints[jointType].Y >= 380 && jointPoints[jointType].Y <= 480)
                                                             {

                                                                 if (!flagStat.Equals("BoundB"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("B");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundB";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 400 && jointPoints[jointType].X <= 500 && jointPoints[jointType].Y >= 380 && jointPoints[jointType].Y <= 480)
                                                             {

                                                                 if (!flagStat.Equals("BoundC"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("C");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundC";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 550 && jointPoints[jointType].X <= 650 && jointPoints[jointType].Y >= 380 && jointPoints[jointType].Y <= 480)
                                                             {

                                                                 if (!flagStat.Equals("BoundD"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("D");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundD";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 700 && jointPoints[jointType].X <= 800 && jointPoints[jointType].Y >= 380 && jointPoints[jointType].Y <= 480)
                                                             {

                                                                 if (!flagStat.Equals("BoundE"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("E");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundE";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 850 && jointPoints[jointType].X <= 950 && jointPoints[jointType].Y >= 380 && jointPoints[jointType].Y <= 480)
                                                             {

                                                                 if (!flagStat.Equals("BoundF"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("F");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundF";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 1000 && jointPoints[jointType].X <= 1100 && jointPoints[jointType].Y >= 380 && jointPoints[jointType].Y <= 480)
                                                             {

                                                                 if (!flagStat.Equals("BoundG"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("G");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundG";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 1150 && jointPoints[jointType].X <= 1250 && jointPoints[jointType].Y >= 380 && jointPoints[jointType].Y <= 480)
                                                             {

                                                                 if (!flagStat.Equals("BoundH"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("H");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundH";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 100 && jointPoints[jointType].X <= 200 && jointPoints[jointType].Y >= 518 && jointPoints[jointType].Y <= 618)
                                                             {

                                                                 if (!flagStat.Equals("BoundI"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("I");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundI";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 250 && jointPoints[jointType].X <= 350 && jointPoints[jointType].Y >= 518 && jointPoints[jointType].Y <= 618)
                                                             {

                                                                 if (!flagStat.Equals("BoundJ"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("J");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundJ";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 400 && jointPoints[jointType].X <= 500 && jointPoints[jointType].Y >= 518 && jointPoints[jointType].Y <= 618)
                                                             {

                                                                 if (!flagStat.Equals("BoundK"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("K");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundK";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 550 && jointPoints[jointType].X <= 650 && jointPoints[jointType].Y >= 518 && jointPoints[jointType].Y <= 618)
                                                             {

                                                                 if (!flagStat.Equals("BoundL"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("L");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundL";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 700 && jointPoints[jointType].X <= 800 && jointPoints[jointType].Y >= 518 && jointPoints[jointType].Y <= 618)
                                                             {

                                                                 if (!flagStat.Equals("BoundM"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("M");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundM";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 850 && jointPoints[jointType].X <= 950 && jointPoints[jointType].Y >= 518 && jointPoints[jointType].Y <= 618)
                                                             {

                                                                 if (!flagStat.Equals("BoundN"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("N");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundN";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 1000 && jointPoints[jointType].X <= 1100 && jointPoints[jointType].Y >= 518 && jointPoints[jointType].Y <= 618)
                                                             {

                                                                 if (!flagStat.Equals("BoundO"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("O");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundO";
                                                                 }


                                                             }
                                                             else if (jointPoints[jointType].X >= 1150 && jointPoints[jointType].X <= 1250 && jointPoints[jointType].Y >= 518 && jointPoints[jointType].Y <= 618)
                                                             {

                                                                 if (!flagStat.Equals("BoundP"))
                                                                 {
                                                                     Debug.WriteLine(test + "loop");
                                                                     textBox2.AppendText("P");
                                                                     test = 1;
                                                                     prevPosi = jointPoints[jointType];
                                                                     flagStat = "BoundP";
                                                                 }


                                                             }

                                                             else { flagStat = "NoBound"; }*/
                                                        //  textBox1.Text = "Not Clicked";

                                                        //   trackingPencil.Add(jointPoints[jointType]);
                                                        //   pencilDraw(jointPoints[jointType].X, jointPoints[jointType].Y,dc);
                                                    }
                                                }
                                            }

                                        }
                                        else
                                        {
                                            TouchStat.Text = "Not Touched";
                                            canvasPanel.Background = Brushes.Red;
                                            flagStat = "NoBound";
                                        }
                                        //     test = 0;
                                        //      flagStat = "NoBound";

                                    }
                                }
                                this.DrawBody(joints, jointPoints, dc, drawPen);

                                //this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                                // this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                            }
                        }

                        // prevent drawing outside of our render area
                        // this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                }
            }
        }
        //depthframe arrived
 /*      private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            framecount += 1;
            if (framecount == 5)
            {
                
                frame = e.FrameReference.AcquireFrame();
                frame.CopyFrameDataToArray(depthData);

            }


        }*/
        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
           // foreach (var bone in this.bones)
            {
        //        this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null )
                {
                    if ( jointType == JointType.HandTipLeft || jointType == JointType.HandTipRight)
                    {
                   //    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                       
                                                  String positionOfHand = jointPoints[jointType].ToString();

                        FormattedText ft = new FormattedText(
                     positionOfHand,
                     CultureInfo.CurrentCulture,
                     FlowDirection.LeftToRight,
                     new Typeface(new FontFamily("Century"), FontStyles.Normal, FontWeights.Bold, FontStretches.Normal),
                     36,    // 36 pt type
                     Brushes.White);
                     //  drawingContext.DrawText(ft, new Point(jointPoints[jointType].X + 5, jointPoints[jointType].Y + 5));

                    }
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }
        
        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

       
        public void pencilDraw(double x, double y,DrawingContext dc)
        {
        
            foreach (Point pt in trackingPencil)
                dc.DrawEllipse(this.handClosedBrush, null, pt, 15, 15);
        }
        
      //  private void button_Click(object sender, RoutedEventArgs e)
       
    }
}
