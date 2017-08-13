using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;


using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;

using Microsoft.Kinect;


namespace Microsoft.Samples.Kinect.BodyBasics
{
    /// <summary>
    /// Interaction logic for Window1.xaml
    /// </summary>
    public partial class Window1 : Window
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;
        Point MousepL,MousepR;
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public Window1()
        {
           
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            // set IsAvailableChanged event notifier
           // this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
        //    this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
          //                                                  : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
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
        
        public object RootWindow { get; private set; }
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
        private void canvas_MouseDown(object sender, MouseButtonEventArgs e) {
            Point Mousep = e.GetPosition(canvas); 
            if (MousepL.X ==0 && MousepL.Y == 0)
            { MousepL = Mousep; }
            else if (MousepL.X != 0 && MousepL.Y != 0 && MousepR.X == 0 && MousepR.Y==0)
            { MousepR = Mousep; }
          
            System.Text.StringBuilder messageBoxCS = new System.Text.StringBuilder();
            messageBoxCS.AppendFormat("{0} = {1}", "X", Mousep.X);
            messageBoxCS.AppendLine();
            messageBoxCS.AppendFormat("{0} = {1}", "Y", Mousep.Y);
            messageBoxCS.AppendLine();
            MessageBox.Show(messageBoxCS.ToString(), "MouseClick Event");

        }
       
        private void button_Click(object sender, RoutedEventArgs e)
        {
            Point topLeft, bottomRight;
            topLeft = MousepL;
            bottomRight = MousepR;
            Debug.WriteLine(""+topLeft + bottomRight);
            MainWindow mw = new MainWindow(topLeft, bottomRight);
            mw.Show();
            this.Close();
        }
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;
            }
        }
    }
}
