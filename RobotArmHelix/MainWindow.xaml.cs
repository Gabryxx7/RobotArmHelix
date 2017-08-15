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
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using HelixToolkit.Wpf;
using System.IO;

/**
 * Author: Gabriele Marini (Gabryxx7)
 * This class load the 3d models of all the parts of the robotic arms and add them to the viewport
 * It also defines the relations among the joints of the robotic arms in order to reflect the movement of the robot in the real world
 * **/
namespace RobotArmHelix
{
    /// <summary>
    /// Interaction logic for UserControl1.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        //provides functionality to 3d models
        Model3DGroup RA = new Model3DGroup(); //RoboticArm 3d group
        Model3D geom = null; //Debug sphere to check in which point the joint is rotatin
        List<Model3D> links = null; //list of all the loaded 3d models of the joints
        List<Tuple<double, double>> linksAngles = null; //list of all the loaded 3d models of the joints
        Color oldColor = Colors.White;
        GeometryModel3D oldSelectedModel = null;
        string basePath = "";
        ModelVisual3D visual;
        double LearningRate = 0.01;
        double SamplingDistance = 0.15;
        double DistanceThreshold = 20;
        //provides render to model3d objects
        ModelVisual3D RoboticArm = new ModelVisual3D();
        Transform3DGroup F1;
        Transform3DGroup F2;
        Transform3DGroup F3;
        Transform3DGroup F4;
        Transform3DGroup F5;
        Transform3DGroup F6;
        Vector3D reachingPoint;
        int movements = 10;
        System.Windows.Forms.Timer timer1;

        //directroy of all stl files
        private const string MODEL_PATH1 = "IRB4600_20kg-250_LINK1_CAD_rev04.stl";
        private const string MODEL_PATH2 = "IRB4600_20kg-250_LINK2_CAD_rev04.stl";
        private const string MODEL_PATH3 = "IRB4600_20kg-250_LINK3_CAD_rev04.stl";
        private const string MODEL_PATH4 = "IRB4600_20kg-250_LINK3_CAD_rev005.stl";
        private const string MODEL_PATH5 = "IRB4600_20kg-250_LINK4_CAD_rev04.stl";
        private const string MODEL_PATH6 = "IRB4600_20kg-250_LINK5_CAD_rev04.stl";
        private const string MODEL_PATH7 = "IRB4600_20kg-250_LINK6_CAD_rev04.stl";
        private const string MODEL_PATH8 = "IRB4600_20kg-250_CABLES_LINK1_rev03.stl";
        private const string MODEL_PATH9 = "IRB4600_20kg-250_CABLES_LINK2_rev03.stl";
        private const string MODEL_PATH10 = "IRB4600_20kg-250_CABLES_LINK3_rev03.stl";
        private const string MODEL_PATH11 = "IRB4600_20kg-250_BASE_CAD_rev04.stl";

        RotateTransform3D R = new RotateTransform3D();
        TranslateTransform3D T = new TranslateTransform3D();

        public MainWindow()
        {
            InitializeComponent();
            basePath = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + "\\3D_Models\\";
            List<string> modelsNames = new List<string>();
            modelsNames.Add(MODEL_PATH1);
            modelsNames.Add(MODEL_PATH2);
            modelsNames.Add(MODEL_PATH3);
            modelsNames.Add(MODEL_PATH4);
            modelsNames.Add(MODEL_PATH5);
            modelsNames.Add(MODEL_PATH6);
            modelsNames.Add(MODEL_PATH7);
            modelsNames.Add(MODEL_PATH8);
            modelsNames.Add(MODEL_PATH9);
            modelsNames.Add(MODEL_PATH10);
            modelsNames.Add(MODEL_PATH11);
            RoboticArm.Content = Initialize_Environment(modelsNames);

            /** Debug sphere to check in which point the joint is rotating**/
            var builder = new MeshBuilder(true, true);
            var position = new Point3D(0, 0, 0);
            builder.AddSphere(position, 50, 15, 15);
            geom = new GeometryModel3D(builder.ToMesh(), Materials.Brown);
            visual = new ModelVisual3D();
            visual.Content = geom;
            viewPort3d.Children.Add(visual);
            viewPort3d.Children.Add(RoboticArm);
            viewPort3d.Camera.LookDirection = new Vector3D(2038, -5200, -2930);
            viewPort3d.Camera.UpDirection = new Vector3D(-0.145, 0.372, 0.917);
            viewPort3d.Camera.Position = new Point3D(-1571, 4801, 3774);
            double[] angles = { joint1.Value, joint2.Value, joint3.Value, joint4.Value, joint5.Value, joint6.Value };
            ForwardKinematics(angles);

            timer1 = new System.Windows.Forms.Timer();
            timer1.Interval = 5;
            timer1.Tick += new System.EventHandler(timer1_Tick);
        }

        private Model3DGroup Initialize_Environment(List<string> modelsNames)
        {
            try
            {
                viewPort3d.RotateGesture = new MouseGesture(MouseAction.RightClick);
                viewPort3d.PanGesture = new MouseGesture(MouseAction.LeftClick);
                ModelImporter import = new ModelImporter();
                links = new List<Model3D>();

                foreach (string modelName in modelsNames)
                {
                    var materialGroup = new MaterialGroup();
                    Color mainColor = Colors.White;
                    EmissiveMaterial emissMat = new EmissiveMaterial(new SolidColorBrush(mainColor));
                    DiffuseMaterial diffMat = new DiffuseMaterial(new SolidColorBrush(mainColor));
                    SpecularMaterial specMat = new SpecularMaterial(new SolidColorBrush(mainColor), 200);
                    materialGroup.Children.Add(emissMat);
                    materialGroup.Children.Add(diffMat);
                    materialGroup.Children.Add(specMat);

                    var link = import.Load(basePath + modelName);
                    link.SetName(modelName);
                    GeometryModel3D model = link.Children[0] as GeometryModel3D;
                    model.Material = materialGroup;
                    model.BackMaterial = materialGroup;
                    links.Add(link);
                }

                changeModelColor(links[5], Colors.LightGray);
                changeModelColor(links[6], Colors.LightGray);
                changeModelColor(links[7], Colors.Black);
                changeModelColor(links[8], Colors.Black);
                changeModelColor(links[9], Colors.Black);
                changeModelColor(links[10], Colors.Gray);
                changeModelColor(links[2], Colors.Red);

                RA.Children.Add(links[0]);
                RA.Children.Add(links[1]);
                RA.Children.Add(links[2]);
                RA.Children.Add(links[3]);
                RA.Children.Add(links[4]);
                RA.Children.Add(links[5]);
                RA.Children.Add(links[6]);
                RA.Children.Add(links[7]);
                RA.Children.Add(links[8]);
                RA.Children.Add(links[9]);
                RA.Children.Add(links[10]);

                linksAngles = new List<Tuple<double, double>>();
                linksAngles.Add(new Tuple<double, double>(-180, 180));
                linksAngles.Add(new Tuple<double, double>(-100, 60));
                linksAngles.Add(new Tuple<double, double>(-90, 90));
                linksAngles.Add(new Tuple<double, double>(-180, 180));
                linksAngles.Add(new Tuple<double, double>(-115, 125));
                linksAngles.Add(new Tuple<double, double>(-180, 180));
            }
            catch (Exception e)
            {
                MessageBox.Show("Exception Error:" + e.StackTrace);
            }
            return RA;
        }

        public static T Clamp<T>(T value, T min, T max)
            where T : System.IComparable<T>
        {
            T result = value;
            if (value.CompareTo(max) > 0)
                result = max;
            if (value.CompareTo(min) < 0)
                result = min;
            return result;
        }



        private void joint1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            execute_fk();
        }

        private void joint2_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            execute_fk();
        }

        private void joint3_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            execute_fk();
        }

        private void joint4_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            execute_fk();
        }

        private void joint5_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            execute_fk();
        }

        private void joint6_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            execute_fk();
        }

        /**
         * This methodes execute the FK (Forward Kinematics). It starts from the first joint, the base.
         * */
        private void execute_fk()
        {
            /** Debug sphere, it takes the x,y,z of the textBoxes and update its position
             * This is useful when using x,y,z in the "new Point3D(x,y,z)* when defining a new RotateTransform3D() to check where the joints is actually 
               rotating */
            //double x = Double.Parse(this.TbX.Text);
            //double y = Double.Parse(this.TbY.Text);
            //double z = Double.Parse(this.TbZ.Text);
            //geom.Transform = new TranslateTransform3D(x, y, z);
            double[] angles = { joint1.Value, joint2.Value, joint3.Value, joint4.Value, joint5.Value, joint6.Value };
            ForwardKinematics(angles);
        }

        private void changeModelColor(Model3D mModel, Color newColor)
        {
            Model3DGroup models = ((Model3DGroup)mModel);
            GeometryModel3D model = models.Children[0] as GeometryModel3D;
            MaterialGroup mg = (MaterialGroup)model.Material;
            ((EmissiveMaterial)mg.Children[0]).Color = newColor;
            ((DiffuseMaterial)mg.Children[1]).Color = newColor;
        }

        private void TbX_TextChanged(object sender, TextChangedEventArgs e)
        {
            try
            {
                geom.Transform = new TranslateTransform3D(new Vector3D(Double.Parse(TbX.Text), Double.Parse(TbY.Text), Double.Parse(TbZ.Text)));
            }
            catch (Exception exc)
            {

            }
        }

        private void ViewPort3D_OnMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {

            Point mousePos = e.GetPosition(viewPort3d);
            PointHitTestParameters hitParams = new PointHitTestParameters(mousePos);
            VisualTreeHelper.HitTest(viewPort3d, null, ResultCallback, hitParams);

        }

        public HitTestResultBehavior ResultCallback(HitTestResult result)
        {
            // Did we hit 3D?
            RayHitTestResult rayResult = result as RayHitTestResult;
            if (rayResult != null)
            {
                // Did we hit a MeshGeometry3D?
                RayMeshGeometry3DHitTestResult rayMeshResult = rayResult as RayMeshGeometry3DHitTestResult;
                geom.Transform = new TranslateTransform3D(new Vector3D(rayResult.PointHit.X, rayResult.PointHit.Y, rayResult.PointHit.Z));

                if (rayMeshResult != null)
                {
                    // Yes we did!
                }
            }

            return HitTestResultBehavior.Continue;
        }

        private void ViewPort3D_OnMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            // Get the mouse's position relative to the viewport.
            Point mouse_pos = e.GetPosition(viewPort3d);

            // Perform the hit test.
            HitTestResult result =
            VisualTreeHelper.HitTest(viewPort3d, mouse_pos);    // Display information about the hit.
            RayMeshGeometry3DHitTestResult mesh_result = result as RayMeshGeometry3DHitTestResult;

            MaterialGroup mg = null;
            if (oldSelectedModel != null)
            {
                mg = (MaterialGroup)oldSelectedModel.Material;
                try
                {
                    ((EmissiveMaterial)mg.Children[0]).Color = oldColor;
                    ((DiffuseMaterial)mg.Children[1]).Color = oldColor;
                }
                catch (Exception exc)
                {

                }
            }

            if (mesh_result != null)
            {
                // Display the name of the model.
                GeometryModel3D model = mesh_result.ModelHit as GeometryModel3D;
                oldSelectedModel = model;
                mg = (MaterialGroup)oldSelectedModel.Material;
                if (mg.Children.Count > 0)
                {
                    try
                    {
                        oldColor = ((EmissiveMaterial)mg.Children[0]).Color;
                        ((EmissiveMaterial)mg.Children[0]).Color = ColorHelper.HexToColor("#ff3333");
                        ((DiffuseMaterial)mg.Children[1]).Color = ColorHelper.HexToColor("#ff3333");
                    }
                    catch (Exception exc)
                    {

                    }
                }
            }
        }


        public void StartInverseKinematics(object sender, RoutedEventArgs e)
        {
            if (timer1.Enabled)
            {
                button.Content = "Go to position";
                timer1.Stop();
                movements = 0;
            }
            else
            {
                reachingPoint = new Vector3D(Double.Parse(TbX.Text), Double.Parse(TbY.Text), Double.Parse(TbZ.Text));
                geom.Transform = new TranslateTransform3D(reachingPoint);

                //for (int i = 0; i < 10; i++)
                //{
                //angles = InverseKinematics(reachingPoint, angles);
                //}

                movements = 5000;
                button.Content = "STOP";
                timer1.Start();
            }
        }

        public void timer1_Tick(object sender, EventArgs e)
        {
            double[] angles = { joint1.Value, joint2.Value, joint3.Value, joint4.Value, joint5.Value, joint6.Value };
            angles = InverseKinematics(reachingPoint, angles);
            joint1.Value = angles[0];
            joint2.Value = angles[1];
            joint3.Value = angles[2];
            joint4.Value = angles[3];
            joint5.Value = angles[4];
            joint6.Value = angles[5];
            if ((--movements) <= 0)
            {
                button.Content = "Go to position";
                timer1.Stop();
            }
        }

        public double[] InverseKinematics(Vector3D target, double[] angles)
        {
            if (DistanceFromTarget(target, angles) < DistanceThreshold)
            {
                movements = 0;
                return angles;
            }

            double[] oldAngles = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            angles.CopyTo(oldAngles, 0);
            for (int i = 5; i >= 0; i--)
            {
                // Gradient descent
                // Update : Solution -= LearningRate * Gradient
                double gradient = PartialGradient(target, angles, i);
                angles[i] -= LearningRate * gradient;

                // Clamp
                angles[i] = Clamp(angles[i], linksAngles[i].Item1, linksAngles[i].Item2);

                // Early termination
                if (DistanceFromTarget(target, angles) < DistanceThreshold || checkAngles(oldAngles, angles))
                {
                    movements = 0;
                    return angles;
                }
            }

            return angles;
        }

        public bool checkAngles(double[] oldAngles, double[] angles)
        {
            for (int i = 0; i <= 5; i++)
            {
                if (oldAngles[i] != angles[i])
                    return false;
            }

            return true;

        }

        public double PartialGradient(Vector3D target, double[] angles, int i)
        {
            // Saves the angle,
            // it will be restored later
            double angle = angles[i];

            // Gradient : [F(x+SamplingDistance) - F(x)] / h
            double f_x = DistanceFromTarget(target, angles);

            angles[i] += SamplingDistance;
            double f_x_plus_d = DistanceFromTarget(target, angles);

            double gradient = (f_x_plus_d - f_x) / SamplingDistance;

            // Restores
            angles[i] = angle;

            return gradient;
        }


        public double DistanceFromTarget(Vector3D target, double[] angles)
        {
            Vector3D point = ForwardKinematics(angles);

            return Math.Sqrt(Math.Pow((point.X - target.X), 2.0) + Math.Pow((point.Y - target.Y), 2.0) + Math.Pow((point.Z - target.Z), 2.0));
        }


        public Vector3D ForwardKinematics(double[] angles)
        {
            //The base only has rotation and is always at the origin, so the only transform in the transformGroup is the rotation R
            F1 = new Transform3DGroup();
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), angles[0]), new Point3D(0, 0, 0));
            F1.Children.Add(R);

            //This moves the first joint attached to the base, it may translate and rotate. Since the joint are already in the right position (the .stl model also store the joints position
            //in the virtual world when they were first created, so if you load all the .stl models of the joint they will be automatically positioned in the right locations)
            //so in all of these cases the first translation is always 0, I just left it for future purposes if something need to be moved
            //After that, the joint needs to rotate of a certain amount (given by the value in the slider), and the rotation must be executed on a specific point
            //After some testing it looks like the point 175, -200, 500 is the sweet spot to achieve the rotation intended for the joint
            //finally we also need to apply the transformation applied to the base 
            F2 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), angles[1]), new Point3D(175, -200, 500));
            F2.Children.Add(T);
            F2.Children.Add(R);
            F2.Children.Add(F1);

            //The second joint is attached to the first one. As before I found the sweet spot after testing, and looks like is rotating just fine. No pre-translation as before
            //and again the previous transformation needs to be applied
            F3 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), angles[2]), new Point3D(190, -170, 1595));
            F3.Children.Add(T);
            F3.Children.Add(R);
            F3.Children.Add(F2);

            //as before
            F4 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), angles[3]), new Point3D(400, 0, 1765));
            F4.Children.Add(T);
            F4.Children.Add(R);
            F4.Children.Add(F3);

            //as before
            F5 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), angles[4]), new Point3D(1405, 50, 1765));
            F5.Children.Add(T);
            F5.Children.Add(R);
            F5.Children.Add(F4);

            //NB: I was having a nightmare trying to understand why it was always rotating in a weird way... SO I realized that the order in which
            //you add the Children is actually VERY IMPORTANT in fact before I was applyting F and then T and R, but the previous transformation
            //Should always be applied as last (FORWARD Kinematics)
            F6 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), angles[5]), new Point3D(1405, 0, 1765));
            F6.Children.Add(T);
            F6.Children.Add(R);
            F6.Children.Add(F5);

            links[0].Transform = F1; //First joint
            links[7].Transform = F1; //Cables

            links[1].Transform = F2; //Second joint (the "biceps")
            links[8].Transform = F2; //Cables

            links[2].Transform = F3; //third joint (the "knee" or "elbow")
            links[3].Transform = F3; //The ABB writings on the third joint (it should rotate according to the joint itself)
            links[9].Transform = F3; //Cables

            links[4].Transform = F4; //the "forearm"


            links[5].Transform = F5; //the tool plate
            links[6].Transform = F6; //the tool

            Tx.Content = links[6].Bounds.Location.X;
            Ty.Content = links[6].Bounds.Location.Y;
            Tz.Content = links[6].Bounds.Location.Z;
            Tx_Copy.Content = geom.Bounds.Location.X;
            Ty_Copy.Content = geom.Bounds.Location.Y;
            Tz_Copy.Content = geom.Bounds.Location.Z;

            return new Vector3D(links[6].Bounds.Location.X, links[6].Bounds.Location.Y, links[6].Bounds.Location.Z);
        }

    }
}
