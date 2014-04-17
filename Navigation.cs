using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double x_des, y_des, t_des;
        public double desiredX, desiredY, desiredT;
        public double maxweight;
        public double gyro_timestep;
        public double gyroAngle;
        public double partAngle;
        public bool newOdom = false;
        public double lasersum, prevLaser;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double wheelDistRandR, wheelDistRandL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double lastGyro_x, lastGyro_y, lastGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;

        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        
        public bool loggingOn;
        StreamWriter logFile;
        String streamPath_;
        public bool logParticles, logLaserData, logParticleEst;

        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 2;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public Particle[] tempParticles;
        public int numParticles = 5000;
        public double K_wheelRandomness = 0.15;//0.25
        public static Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 6.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        public double[] Weights;
        private int laserCounter;
        private int laserStepSize = 25;

        public List<Tuple<double, double>> trajPoints = new List<Tuple<double, double>>(); //trajectory to track
        bool pointReached = false;
        bool pathDefined = false;
        public Node randExpansionNode;
        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }

            public Particle(Particle p)
            {
                x = p.x;
                y = p.y;
                t = p.t;
                w = p.w;
            }
        }

        // Motion Planner Variables
        const int numXCells = 20;
        const int numYCells = 20;
        const int maxNumNodes = 5000;
        const float minWorkspaceX = -10.0f;
        const float maxWorkspaceX = 10.0f;
        const float minWorkspaceY = -10.0f;
        const float maxWorkspaceY = 10.0f;

        // Motion Planner Variables 
        public double samplingCellSizeX, samplingCellSizeY;
        public int numOccupiedCells;
        public int[] occupiedCellsList;
        public int[] numNodesInCell;
        public Node[,] NodesInCells;
        public Node[] trajList, nodeList;
        public int trajSize, trajCurrentNode, numNodes;
        public bool trackTrajPD = false;
        public bool allGoalsRreached = false;
        public double destX = 0;
        public double destY = 0;//destinations to achieve for now

        public class Node
        {
            public double x, y;
            public int lastNode;
            public int nodeIndex;

            public Node()
            {
                x = 0;
                y = 0;
                lastNode = 0;
                nodeIndex = 0;
            }

            public Node(double _x, double _y, int _nodeIndex, int _lastNode)
            {
                x = _x;
                y = _y;
                nodeIndex = _nodeIndex;
                lastNode = _lastNode;
            }
        }

        public Boolean resetEverything = false;
        #endregion


        #region Navigation Setup

        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }


            this.Initialize();
            loggingOn = false;
            logLaserData = false;
            logParticles = true;
            logParticleEst = true;

            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
            
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = initialX;
            y = initialY;
            t = initialT;

            // Initialize state estimates
            if (jaguarControl.startMode == jaguarControl.KNOWN)
            {
                x_est = initialX;
                y_est = initialY;
                t_est = initialT;
            }
            else
            {
                x_est = 0;
                y_est = 0;
                t_est = 0;
            }
            // Set desired state
            desiredX =  initialX;
            desiredY =  initialY;
            desiredT =  initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            
            // Set random start for particles
            InitializeParticles();
            gyroAngle = initialT;

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

            // MP variable setup
            occupiedCellsList = new int[numXCells * numYCells];
            numNodesInCell = new int[numXCells * numYCells];
            NodesInCells = new Node[numXCells * numYCells, 500];
            trajList = new Node[maxNumNodes];
            nodeList = new Node[maxNumNodes];
            numNodes = 0;
            trajList[0] = new Node(0, 0, 0, 0);
            trajSize = 0;
            trajPoints.Clear();
            trackTrajPD = false;
            allGoalsRreached = false;
            destX = 0;
            destY = 0;//destinations to achieve for now
            pathDefined = false;
        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {

            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();

        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).

                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();

                if (resetEverything)
                {
                    Reset();
                    resetEverything = false;
                }
                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5) if not defined yet
                        if (!pathDefined && trackTrajPD) 
                        {
                           definePath();
                        }
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to a desired Point (lab 3)
                    //FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                 //   if (trackTrajPD)
                  //  {
                        TrackTrajectory();
                 //   }
                  //  else 
                   // {
                      //  FlyToSetPoint();
                   // }
                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }

                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }



        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y / numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                        currentGyro_x = jaguarControl.getGyro_x();
                        currentGyro_y = jaguarControl.getGyro_y();
                        currentGyro_z = jaguarControl.getGyro_z();
                        lastGyro_x = currentGyro_x;
                        lastGyro_y = currentGyro_y;
                        lastGyro_z = currentGyro_z;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;
                lastGyro_x = 0;
                lastGyro_y = 0;
                lastGyro_z = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                newLaserData = false;
                if (laserCounter >= 200)
                {
                    for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t - 1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                    currentGyro_x = jaguarControl.getGyro_x(); //check conversion of gyro signal
                    currentGyro_y = jaguarControl.getGyro_y();
                    currentGyro_z = jaguarControl.getGyro_z();

                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;



            motorSignalL = (short)(zeroOutput);
            motorSignalR = (short)(zeroOutput);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            //changed here
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            streamPath_ = "../../../../Lab4LoggingData/E190Q_" + jaguarControl.fileName + ".csv";
            ToString();
            logFile = File.CreateText(streamPath_);
            logFile.Close();
            writeHeader();
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {

            if (logFile != null)
                try
                {
                    logFile.Close();
                }
                catch { }
            loggingOn = false;
        }

        //This function is called when logging first starts to write the headers on the file being recorded
        public void writeHeader()
        {
            String newData = "Time" + ", " + "x" + " ," + "y" + " , " + "theta" + ",";
            
            if (logParticleEst)
                newData += "x_est" + "," + "y_est" + "," + "t_est" + ",";
            
            if (logParticles)
            {
                for (int i = 0; i < numParticles; i++)
                    newData += "particles["+i+"x" + "," + "particles["+i+"].y" + "," + "particles["+i+"].t" + "," + "particles["+i+"].w" + ",";
            }

            if (logLaserData)
            {
                for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
                    newData += "LaserData["+i+"]" + ",";
            }

            
            newData += "";
            
            //newData += "," + "counter";
            logFile = File.AppendText(streamPath_);
            logFile.WriteLine(newData);
            logFile.Close();
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                //changed here
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;

                String newData = time.ToString() + ", " + x.ToString() + " ," + y.ToString() + " , " + t.ToString() + ",";

                if (logParticleEst)
                    newData += x_est + "," + y_est + "," + t_est + ",";

                if (logParticles)
                {
                    for (int i = 0; i < numParticles; i++)
                        newData += particles[i].x + "," + particles[i].y + "," + particles[i].t + "," + particles[i].w + ",";
                }

                if (logLaserData)
                {
                    for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
                        newData += LaserData[i] + ",";
                }

                newData += "";
                
                // newData += "," + countL + "," + countR; ;
                logFile = File.AppendText(streamPath_);
                logFile.WriteLine(newData);
                logFile.Close();
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control
            double error = centralLaserRange - 1;
            int K = 20;
            if (Math.Abs(error) < 0.05)
            {
                motorSignalR = 0; //desiredWheelSpeedR
                motorSignalL = 0; //desiredWheelSpeedL
            }
            else
            {
                motorSignalL = (short)Math.Max(-100, Math.Min(100, K * error));
                motorSignalR = (short)Math.Max(-100, Math.Min(100, K * error));
            }



            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate desiredRotRateR and 
            // desoredRotRateL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            desiredRotRateR = 0;
            desiredRotRateL = 0;
            double distThres = 0.05;
            double thetaThres = Math.PI / 36;
            maxVelocity = 0.3 * (2 * Math.PI) / wheelRadius;

            //variables to be used in the function
            double deltaX, deltaY, deltat;
            double alpha, beta, rho;
            double v, w;
            double SR, SL, desiredVR, desiredVL;

            
            //find difference from the desired point
                deltaX = x_des - x_est;
                deltaY = y_des - y_est;
                deltat = t_des - t_est;
                deltat = NormalizeAngle(deltat);

            //calculate rho and alpha
            rho = Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
            alpha = -t_est + Math.Atan2(deltaY, deltaX);

            alpha = NormalizeAngle(alpha);
            v = Kpho * rho;

            //change control law if the desired point is behind the robot
            if (Math.Abs(alpha) > Math.PI / 2)
            {
                alpha = -t + Math.Atan2(-deltaY, -deltaX);
                alpha = NormalizeAngle(alpha);
                v = -Kpho * rho;
            }

            beta = deltat - alpha;
            beta = NormalizeAngle(beta);
            w = Kalpha * alpha + Kbeta * beta;

            // stop if destination reached
            if (rho < distThres)
            {
                v = 0;
                w = (-Kbeta * deltat) + (Math.Sign(deltat) * 2.0);

                if (Math.Abs(deltat) < thetaThres)
                {
                    w = 0;
                }
               
            }

            //calculate desired velocities from v and w
            SR = w / 2 + v / (2 * robotRadius);
            SL = -w / 2 + v / (2 * robotRadius);
            desiredVR = SR * 2.0 * robotRadius / wheelRadius;
            desiredVL = SL * 2.0 * robotRadius / wheelRadius;

            //don't exceed the maximum velocity           
            if (Math.Abs(desiredVR) > maxVelocity)
            {
                desiredVL *= maxVelocity / Math.Abs(desiredVR);
                desiredVR = Math.Sign(desiredVR) * maxVelocity;
            }
            if (Math.Abs(desiredVL) > maxVelocity)
            {
                desiredVR *= maxVelocity / Math.Abs(desiredVL);
                desiredVL = Math.Sign(desiredVL) * maxVelocity;
            }

            //set the rotation rate
            desiredRotRateR = (short)(desiredVR * pulsesPerRotation / (Math.PI * 2));
            desiredRotRateL = (short)(desiredVL * pulsesPerRotation / (Math.PI * 2));



            // ****************** Additional Student Code: End   ************
        }


        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        /*
        private void TrackTrajectory()
        {
            if (pointReached)
            {
                trajPoints.Remove(trajPoints[0]);

                if (trajPoints.Count == 0)
                {
                    if (jaguarControl.Simulating())
                        simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, 0, 0, 0);
                    else
                        realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, 0, 0, 0);

                    jaguarControl.controlMode = jaguarControl.MANUAL;

                }
                else
                {
                    desiredX = trajPoints[0].First;
                    desiredY = trajPoints[0].Second;
                    try
                    {
                        double nextX = trajPoints[1].First;
                        double nextY = trajPoints[1].Second;
                        desiredT = Math.Atan2(nextY - desiredY, nextX - desiredX);
                    }
                    catch
                    {
                        ;
                    }
                    //desiredT = Math.Atan2(desiredY - y_est, desiredX - x_est);
                    NormalizeAngle(desiredT, Math.PI / 2);
                    pointReached = false;
                }

            //   FlyToSetPoint();
                return;
            }
            else
            {
                desiredX = trajPoints[0].First;
                desiredY = trajPoints[0].Second;
                desiredT = Math.Atan2(desiredY - y, desiredX - x);
                FlyToSetPoint();
            }
        }
        */
        #endregion


        #region Localization Functions
        /************************ LOCALIZATION *********************** github*/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.
            diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;
            diffEncoderPulseR = currentEncoderPulseR - lastEncoderPulseR;

            if (diffEncoderPulseL < 0 && Math.Abs(diffEncoderPulseL) > 30000) // robot moving forwards and encoder rollover (LW CCW)
            {
                diffEncoderPulseL = (encoderMax % lastEncoderPulseL) + (currentEncoderPulseL + 1); //+1 bc pulse includes 0
            }

            if (diffEncoderPulseL > 30000) //robot moving backwards and encoder rollover (LW CW)
            {
                diffEncoderPulseL = (encoderMax % currentEncoderPulseL) + (lastEncoderPulseL + 1);
            }

            if (diffEncoderPulseR < 0 && Math.Abs(diffEncoderPulseR) > 30000) //robot moving backwards and encoder rollover (RW CCW)
            {
                diffEncoderPulseR = (encoderMax % lastEncoderPulseR) + (currentEncoderPulseR + 1);
            }

            if (diffEncoderPulseR > 30000) //robot moving forwards and encoder rollover (RW CW)
            {
                diffEncoderPulseR = (encoderMax % currentEncoderPulseR) + (lastEncoderPulseR + 1);
            }

            lastEncoderPulseL = currentEncoderPulseL;
            lastEncoderPulseR = currentEncoderPulseR;
            wheelDistanceL = (diffEncoderPulseL / pulsesPerRotation) * 2 * Math.PI * wheelRadius;
            wheelDistanceR = -(diffEncoderPulseR / pulsesPerRotation) * 2 * Math.PI * wheelRadius;

            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2; //deltaS from class      
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius); //deltaTheta formula from class
            if ((Math.Abs(distanceTravelled) == 0) && (Math.Abs(angleTravelled) == 0))
            {
                newOdom = false;
            } //check for new odometry measurement i.e. robot has moved
            else
            {
                newOdom = true;
            }

            // ****************** Additional Student Code: End   ************
        }

        public void NormalizeAngles(double angle)
        //normalizes given angle to be between pi and -pi
        {
            while (angle < -Math.PI)
            {
                angle = angle + 2 * Math.PI; //e.g. -7pi/4 = pi/4 
            }
            while (angle > Math.PI)
            {
                angle = angle - 2 * Math.PI; //e.g. 7pi/4 = -pi/4
            }
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            double deltaX = distanceTravelled * Math.Cos(t + angleTravelled / 2);
            double deltaY = distanceTravelled * Math.Sin(t + angleTravelled / 2);

            x = x + deltaX;
            y = y + deltaY;
            t = t + angleTravelled;

            //theta is represented b/w pi and -pi            
            NormalizeAngles(t);
            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }

        #endregion

        #region ParticleFilter
        public void PropagateParticles()
        {
            //Console.WriteLine("propagated particles");
            double wheelstdR = 0.4 * wheelDistanceR; //40% of distance travelled by wheel
            double wheelstdL = 0.4 * wheelDistanceL; //m

            double gyrostd = 0.1;
            //gyroAngle = integrate(currentGyro_z, gyro_timestep): trapezoidal method
            gyroAngle = deltaT * (lastGyro_z + currentGyro_z) / 2; //gyroAngle traveled 

            for (int p = 0; p < numParticles; p++)
            {

                wheelDistRandR = wheelDistanceR + wheelstdR * RandomGaussian(); //add random error to odometry, with wheel std deviation
                wheelDistRandL = wheelDistanceL + wheelstdL * RandomGaussian();
                double distTravelledRand = (wheelDistRandR + wheelDistRandL) / 2;
                double angleTravelledRand = (wheelDistRandR - wheelDistRandL) / (2 * robotRadius);
                double deltaXRand = distTravelledRand * Math.Cos(particles[p].t + angleTravelledRand / 2);
                double deltaYRand = distTravelledRand * Math.Sin(particles[p].t + angleTravelledRand / 2);
                
                //propagate particles with odometry
                propagatedParticles[p].x = particles[p].x + deltaXRand;
                propagatedParticles[p].y = particles[p].y + deltaYRand;
                //propagatedParticles[p].t = particles[p].t + angleTravelledRand;
                partAngle = gyroAngle + gyrostd * RandomGaussian(); //add random error to gyroAngle traveled 
                propagatedParticles[p].t = particles[p].t + partAngle;
                NormalizeAngles(propagatedParticles[p].t);

            }
            //update last Gyro_z
            lastGyro_z = currentGyro_z;
        }

        public void CalculateAllWeights()
        {
            //Console.WriteLine("calculating all weights");
            for (int p = 0; p < numParticles; p++)
            {
                CalculateWeight(p);
            }
        }

        public double FindMaxWeight()
        {
            //Console.WriteLine("finding max weight");
            maxweight = 0;
            double currweight;
            for (int p = 0; p < numParticles; p++)
            {
                currweight = propagatedParticles[p].w;
                if (currweight > maxweight)
                {
                    maxweight = currweight;
                }
            }
            return maxweight;
        }

        public void NormalizeWeights()
        {
            //Console.WriteLine("normalizing weights");
            maxweight = FindMaxWeight();
            for (int p = 0; p < numParticles; p++)
            {
                propagatedParticles[p].w = propagatedParticles[p].w / maxweight;

            }
        }

        public void addToTempParticles(int p, int n, int count)
        {
            for (int i = 0; i < n; i++)
            {
                tempParticles[count + i] = propagatedParticles[p];
            }
        }

        public void Resample()
        {
            //Console.WriteLine("resampling");
            int count = 0;
            for (int p = 0; p < numParticles; p++)
            {
                if (propagatedParticles[p].w < 0.25)
                { //may need to tune these weight bounds
                    addToTempParticles(p, 1, count);
                    count += 1;
                }

                else if (propagatedParticles[p].w < 0.4)
                {
                    addToTempParticles(p, 2, count);
                    count += 2;
                }

                else if (propagatedParticles[p].w < 0.7)
                {
                    addToTempParticles(p, 3, count);
                    count += 3;
                }

                else if (propagatedParticles[p].w <= 1.0)
                {
                    addToTempParticles(p, 4, count);
                    count += 4;
                }

            }

            for (int j = 0; j < numParticles; j++)
            {
                int r = random.Next(count); //nonnegative value greater than or equal to 0 and less than maxValue
                propagatedParticles[j].x = tempParticles[r].x;
                propagatedParticles[j].y = tempParticles[r].y;
                propagatedParticles[j].t = tempParticles[r].t;
                propagatedParticles[j].w = tempParticles[r].w;
                /*particles[j].x = tempParticles[r].x;
                particles[j].y = tempParticles[r].y;
                particles[j].t = tempParticles[r].t;
                particles[j].w = tempParticles[r].w;*/


            }
        }

        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab

            // ****************** Additional Student Code: Start ************
            PropagateParticles();
            //check for new laser data by checking the sum of the laser data
            for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
            {
                lasersum += LaserData[i];
            }
            if (lasersum != prevLaser)
            {
                newLaserData = true;
            }
            else
            {
                newLaserData = false;
            }
            prevLaser = lasersum;

            if ((newOdom) && (newLaserData))
            {
                CalculateAllWeights();
                NormalizeWeights();
                Resample();
            }

            for (int p = 0; p < numParticles; p++)
            {
                particles[p].x = propagatedParticles[p].x;
                particles[p].y = propagatedParticles[p].y;
                particles[p].t = propagatedParticles[p].t;
                particles[p].w = propagatedParticles[p].w;
            }

            // Put code here to calculate x_est, y_est, t_est using a PF
            x_est = 0; y_est = 0; t_est = 0;
            for (int p = 0; p < numParticles; p++)
            {
                x_est += (particles[p].x);
                y_est += (particles[p].y);
            }
            x_est = x_est / numParticles; //x_est and y_est are averages of the x and y states
            y_est = y_est / numParticles;

            //taking average of angles is tricky... 
            double re_part = 0, im_part = 0;
            for (int p = 0; p < numParticles; p++)
            {
                //first convert angles to complex numbers on the unit circle (r = 1)
                re_part += Math.Cos(particles[p].t);
                im_part += Math.Sin(particles[p].t);
            }
            //then calculate mean of the complex numbers and find the angle of the mean complex number
            t_est = Math.Atan2(im_part / numParticles, re_part / numParticles);
            // ****************** Additional Student Code: End   ************

        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        void CalculateWeight(int p)
        {
            double laserstd = 100; //mm
            long z;
            double particlerange;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated weight. Feel free to use the
            // function map.GetClosestWallDistance from Map.cs.
            Weights = new double[LaserData.Length];
            double weight = 1;
            for (int i = 0; i < LaserData.Length; i = i + laserStepSize)
            {
                z = LaserData[i]; //mm
                particlerange = map.GetClosestWallDistance(particles[p].x, particles[p].y, particles[p].t - initialT + laserAngles[i]) * 1000; //mm
                Weights[i] = Math.Exp(-Math.Pow((z - particlerange), 2) / (2 * Math.Pow(laserstd, 2))); //normal distribution
                weight *= Weights[i]; //multiply all the laser weights to represent particle's weight
            }
            propagatedParticles[p].w = weight;
        }


        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles()
        {

            // Set particles in random locations and orientations within environment
            for (int i = 0; i < numParticles; i++)
            {

                // Either set the particles at known start position [0 0 0],  
                // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
                    SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
                    SetStartPos(i);
            }

        }

        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environment. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(int p)
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
            // It might be helpful to use boundaries defined in the
            // Map.cs file (e.g. map.minX)

            particles[p].x = random.NextDouble() * (map.maxX - map.minX) + map.minX; //random double between minX and maxX
            particles[p].y = random.NextDouble() * (map.maxY - map.minY) + map.minY; //random double between minY and maxY
            particles[p].t = random.NextDouble() * (Math.PI + Math.PI) - Math.PI; //random double between pi and -pi

            // ****************** Additional Student Code: End   ************
        }


        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p)
        {
            particles[p].x = initialX;
            particles[p].y = initialY;
            particles[p].t = initialT;
        }

        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
            double U1, U2, V1 = 0, V2;
            double S = 2.0;
            while (S >= 1.0)
            {
                U1 = random.NextDouble();
                U2 = random.NextDouble();
                V1 = 2.0 * U1 - 1.0;
                V2 = 2.0 * U2 - 1.0;
                S = Math.Pow(V1, 2) + Math.Pow(V2, 2);
            }
            double gauss = V1 * Math.Sqrt((-2.0 * Math.Log(S)) / S);
            return gauss;
        }



        

        #endregion


        #region Motion Control 
        private void PRMMotionPlanner()
        {
            // Initialize sampling grid cell variables for weighted
            // random selection of nodes to expand.
            samplingCellSizeX = (maxWorkspaceX - minWorkspaceX) / numXCells;
            samplingCellSizeY = (maxWorkspaceY - minWorkspaceY) / numYCells;
            numOccupiedCells = 0;
            for (int i = 0; i < numXCells * numYCells; i++)
                numNodesInCell[i] = 0;
            numNodes = 0;


            // ****************** Additional Student Code: Start ************

            // Put code here to expand the PRM until the goal node is reached,
            // or until a max number of iterations is reached.


            // Create and add the start Node
            Node startNode = new Node(x_est, y_est, 0, 0);
            AddNode(startNode);

            // Create the goal node
            if (trackTrajPD)
            {
                Tuple<double, double>tempGoal = findNewGoal();
                destX = tempGoal.First;
                destY = tempGoal.Second;
            }
            else {
                destX = desiredX;
                destY = desiredY;
            }
            Node goalNode = new Node(destX, destY, 0, 0); 
                                    //i think we don't care about nodeindex and lastnode for goal at this point
           

            // Loop until path created
            bool pathFound = false;
            int maxIterations = maxNumNodes;
            int iterations = 0;
            Random randGenerator = new Random();




            while (iterations < maxIterations && !pathFound)
            {
                int randCellNumber = randGenerator.Next(numOccupiedCells);
                int noNodesCell = numNodesInCell[occupiedCellsList[randCellNumber]];
                int randNodeNumber = randGenerator.Next(noNodesCell);

                //Define randomly selected node
                randExpansionNode = NodesInCells[occupiedCellsList[randCellNumber], randNodeNumber];
                
                //Randomly select distance and orientation
                double randDist = randGenerator.NextDouble();
                double randOrientation = (2 * randGenerator.NextDouble() - 1) * Math.PI;

                //Define new node and add it to node list
                double newX = randExpansionNode.x + randDist * Math.Cos(randOrientation);
                double newY = randExpansionNode.y + randDist * Math.Sin(randOrientation);
                int newNodeindex = numNodes;
                int newLastNode = randExpansionNode.nodeIndex;

                Node newNode = new Node(newX, newY, newNodeindex, newLastNode);
                
                //check if the newNode is collision free
                bool colFound = map.CollisionFound(randExpansionNode, newNode, robotRadius); //tol = robotRadius, can change it later
                if (!colFound)
                {
                    AddNode(newNode);

                    //check for connection to goal
                    bool colToGoal = map.CollisionFound(newNode, goalNode, robotRadius);
                    if (!colToGoal)
                    {
                        goalNode.nodeIndex = numNodes;
                        goalNode.lastNode = newNode.nodeIndex;
                        AddNode(goalNode);
                        pathFound = true;
                    }
                }

                // Increment number of iterations
                iterations++;
            }

            
            // Create the trajectory to follow
            BuildTraj(goalNode);


            // ****************** Additional Student Code: End   ***********

        }

        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            double distToCurrentNode = 0;
            distToCurrentNode = Math.Sqrt(Math.Pow(x_est - trajList[trajCurrentNode].x, 2) + Math.Pow(y_est - trajList[trajCurrentNode].y, 2));

            if (distToCurrentNode < 0.1 && trajCurrentNode + 1 < trajSize)
            {
                trajCurrentNode++;
                x_des = trajList[trajCurrentNode].x;
                y_des = trajList[trajCurrentNode].y;
                t_des = 0;
            }
            FlyToSetPoint();
            distToCurrentNode = Math.Sqrt(Math.Pow(x_est - trajList[trajCurrentNode].x, 2) + Math.Pow(y_est - trajList[trajCurrentNode].y, 2));
            if (distToCurrentNode < 0.1 && trajCurrentNode + 1 >= trajSize && trackTrajPD && !allGoalsRreached)
            {
                pointReached = true;
                motionPlanRequired = true;
            }
        }

        private Tuple<double, double> findNewGoal()
        {
            if (pointReached)
            {
               // motionPlanRequired = true;
                if (trajPoints.Count != 1)
                    trajPoints.Remove(trajPoints[0]);
                else 
                    allGoalsRreached = true;
            }
            return trajPoints[0];
        }

        public void definePath()
        {
            trajPoints.Clear();
            trajPoints.Add(new Tuple<double, double>(0, 1));
            trajPoints.Add(new Tuple<double, double>(1, 2));
            trajPoints.Add(new Tuple<double, double>(-1, -1));
            //trajPoints.Add(new Tuple<double, double>(1, -1));
            //trajPoints.Add(new Tuple<double, double>(1, 0));
            //trajPoints.Add(new Tuple<double, double>(1, 1));
            //trajPoints.Add(new Tuple<double, double>(1, 2));
            pathDefined = true;
            //trajPoints.Add(new Tuple<double, double>(0, 0));
        }

        // This function is used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // The work environment is divided into a grid of cells.
        // This function returns the cell number.
        int GetCellNumber(double x, double y)
        {
            int cell = (int)Math.Floor((x - minWorkspaceX) / samplingCellSizeX) + (int)(Math.Floor((y - minWorkspaceY) / samplingCellSizeY) * numXCells);
            return cell;
        }

        // This function is also used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // When new nodes for the PRM are generated, they must be added
        // to a variety of memory locations.
        // First, the node is stored in a list of nodes specific to a grid
        // cell. If this is the first node in that grid cell, the list of 
        // occupied cells is updated. Then, the node is stored in a general
        // list that keeps track of all nodes for building the final
        // trajectory.

        void AddNode(Node n)
        {
            int cellNumber = GetCellNumber(n.x, n.y);
            if (numNodesInCell[cellNumber] < 1)
            {
                occupiedCellsList[numOccupiedCells] = cellNumber;
                numOccupiedCells++;
            }

            if (numNodesInCell[cellNumber] < 400)
            {
                NodesInCells[cellNumber, numNodesInCell[cellNumber]] = n;
                numNodesInCell[cellNumber]++;

                // Add to nodelist
                nodeList[numNodes] = n;
                numNodes++;
            }
            return;
        }


        // Given the goal node, this function will recursively add the
        // parent node to a trajectory until the start node is reached.
        // The result is a list of nodes that connect the start node to
        // the goal node with collision free edges.

        void BuildTraj(Node goalNode)
        {
            Node[] tempList = new Node[maxNumNodes];
            for (int j = 0; j < maxNumNodes; j++)
                trajList[j] = new Node(0, 0, 0, 0);

            tempList[0] = goalNode;
            int i = 1;

            // Make backwards traj by looking at parent of every child node
            while (tempList[i - 1].nodeIndex != 0)
            {
                tempList[i] = nodeList[tempList[i - 1].lastNode];
                i++;
            }

            // Reverse trajectory order
            for (int j = 0; j < i; j++)
            {
                trajList[j] = tempList[i - j - 1];
            }

            // Set size of trajectory and initialize node counter
            trajSize = i;
            trajCurrentNode = 0;

            return;
        }

        #endregion

        #region helper functions

        // Get the sign of a number
        double Sgn(double a)
        {
            if (a > 0)
                return 1.0;
            else if (a < 0)
                return -1.0;
            else
                return 0.0;
        }

        private double NormalizeAngle(double alpha)
        {
            while (alpha > Math.PI)
            {
                alpha = alpha - (2 * Math.PI);
            }
            while (alpha < -Math.PI)
            {
                alpha = alpha + (2 * Math.PI);
            }
            return alpha;
        }


        private void setOrientation()
        {
            //this function sets orientation during point tracking once the robot has reached within
            //the distance threshold
            //In this case forward velocity would be zero
            // Use a proportional control for rotational velocity 

            double K_rot = 1;
            double thetaThresh = Math.PI / 18;
            double deltaTheta;
            double w;
            double SR, SL;
            double desiredVR, desiredVL;

            deltaTheta = desiredT - t;
            w = K_rot * deltaTheta;

            if (Math.Abs(deltaTheta) < thetaThresh)
                w = 0;

            //calculate desired velocities from v and w
            SR = w / 2;
            SL = -w / 2;
            desiredVR = SR * 2.0 * robotRadius / wheelRadius;
            desiredVL = SL * 2.0 * robotRadius / wheelRadius;

            //don't exceed the maximum velocity           
            if (Math.Abs(desiredVR) > maxVelocity)
            {
                desiredVL *= maxVelocity / Math.Abs(desiredVR);
                desiredVR = Math.Sign(desiredVR) * maxVelocity;
            }
            if (Math.Abs(desiredVL) > maxVelocity)
            {
                desiredVR *= maxVelocity / Math.Abs(desiredVL);
                desiredVL = Math.Sign(desiredVL) * maxVelocity;
            }

            //set the rotation rate
            desiredRotRateR = (short)(desiredVR * pulsesPerRotation / (Math.PI * 2));
            desiredRotRateL = (short)(desiredVL * pulsesPerRotation / (Math.PI * 2));
        }

        #endregion

        
    }
}