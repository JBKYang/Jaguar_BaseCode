using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class Map
    {
        public int numMapSegments = 0;
        public double[, ,] mapSegmentCorners;
        public double minX, maxX, minY, maxY;
        private double[] slopes;
        private double[] segmentSizes;
        private double[] intercepts;

        private double minWorkspaceX = -40;//CHANGE
        private double maxWorkspaceX = 40;
        private double minWorkspaceY = -40;
        private double maxWorkspaceY = 40;
        // New Class
        public class Zone
        {
            public double xmin, ymin, xmax, ymax;

            public Zone()
            {
            }

            public Zone (double x1,double x2,double y1,double y2)
            {
                xmin = x1;
                ymin = y1;
                xmax = x2;
                ymax = y2;
            }

            public double minx(){return xmin;}
            public double maxx() { return xmax; }
            public double miny() { return ymin; }
            public double maxy() { return ymax; }
        }
        public int numThrowZ = 0;
        public List<Zone> mapthrowZ;
        

        

        public Map()
        {

            // This is hard coding at its worst. Just edit the file to put in
            // segments of the environment your robot is working in. This is
            // used both for visual display and for localization.

            // ****************** Additional Student Code: Start ************
            #region drawMap
            // Change hard code here to change map:
            numMapSegments = 30;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];
            //assuming things are more or less symettric, and also that the distance between the bottom of the wall to the top has a horizontal distance of 0.07m
            mapSegmentCorners[0, 0, 0] = 0;
            mapSegmentCorners[0, 0, 1] = -1.13;
            mapSegmentCorners[0, 1, 0] = (8.477 - 6.41);
            mapSegmentCorners[0, 1, 1] = -1.13;

            mapSegmentCorners[1, 0, 0] = 0;
            mapSegmentCorners[1, 0, 1] = -1.13;
            mapSegmentCorners[1, 1, 0] = -1.204;
            mapSegmentCorners[1, 1, 1] = -1.13;//first wall

            mapSegmentCorners[2, 0, 0] = -1.204;
            mapSegmentCorners[2, 0, 1] = -1.13;
            mapSegmentCorners[2, 1, 0] = -1.204;
            mapSegmentCorners[2, 1, 1] = 8.452;//long wall

            mapSegmentCorners[3, 0, 0] = (8.477 - 7.469);
            mapSegmentCorners[3, 0, 1] = 1.13;
            mapSegmentCorners[3, 1, 0] = (8.477 - 2.416);
            mapSegmentCorners[3, 1, 1] = 1.13;

            mapSegmentCorners[4, 0, 0] = (8.477 - 2.416);
            mapSegmentCorners[4, 0, 1] = 1.13;
            mapSegmentCorners[4, 1, 0] = (8.477 - 2.416);
            mapSegmentCorners[4, 1, 1] = 1.13 + (8.435 - 2.284);

            mapSegmentCorners[5, 0, 0] = (8.477 - 7.469);
            mapSegmentCorners[5, 0, 1] = -(-1.13 - (8.435 - 2.284));
            mapSegmentCorners[5, 1, 0] = (8.477 - 2.416);
            mapSegmentCorners[5, 1, 1] = 1.13 + (8.435 - 2.284);

            mapSegmentCorners[6, 0, 0] = (8.477 - 7.469);
            mapSegmentCorners[6, 0, 1] = 1.13;
            mapSegmentCorners[6, 1, 0] = (8.477 - 7.469);
            mapSegmentCorners[6, 1, 1] = 1.13 + (8.435 - 2.284);
            //8.435 - 2.284//center wall

            mapSegmentCorners[7, 0, 0] = (8.477 - 6.41);
            mapSegmentCorners[7, 0, 1] = -1.13;
            mapSegmentCorners[7, 1, 0] = (8.477 - 6.41);
            mapSegmentCorners[7, 1, 1] = -(1.865 - 0.07 + 1.13);

            mapSegmentCorners[8, 0, 0] = (8.477 - 6.41);
            mapSegmentCorners[8, 0, 1] = -(1.865 - 0.07 + 1.13);
            mapSegmentCorners[8, 1, 0] = (8.477 - 6.41 - (9.421 - 3.575));
            mapSegmentCorners[8, 1, 1] = -(1.865 - 0.07 + 1.13);//opening to sprague;


            mapSegmentCorners[9, 0, 0] = (8.477 - 6.41 + 3.506);
            mapSegmentCorners[9, 0, 1] = -1.13;
            mapSegmentCorners[9, 1, 0] = (8.477 - 6.41 + 3.506);
            mapSegmentCorners[9, 1, 1] = -(1.865 - 0.07 + 1.13);

            mapSegmentCorners[11, 0, 0] = (8.477 - 6.41 + 3.506);
            mapSegmentCorners[11, 0, 1] = -(1.865 - 0.07 + 1.13);
            mapSegmentCorners[11, 1, 0] = (8.477 - 6.41 + 3.506 + (9.421 - 3.575));
            mapSegmentCorners[11, 1, 1] = -(1.865 - 0.07 + 1.13);

            mapSegmentCorners[10, 0, 0] = (8.477 - 6.41 + 3.506);
            mapSegmentCorners[10, 0, 1] = -1.13;
            mapSegmentCorners[10, 1, 0] = (8.477 - 0.07);
            mapSegmentCorners[10, 1, 1] = -1.13;

            mapSegmentCorners[12, 0, 0] = (8.477 - 0.07);
            mapSegmentCorners[12, 0, 1] = -1.13;
            mapSegmentCorners[12, 1, 0] = (8.477 - 0.07);
            mapSegmentCorners[12, 1, 1] = 8.452;
            //other side

            mapSegmentCorners[13, 0, 0] = (8.477 - 6.41 - (9.399 - 3.671));
            mapSegmentCorners[13, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984);
            mapSegmentCorners[13, 1, 0] = (8.477 - 6.41 - (9.399 - 3.984));
            mapSegmentCorners[13, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984);

            mapSegmentCorners[14, 0, 0] = (8.477 - 6.41 - (9.399 - 7.308));
            mapSegmentCorners[14, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984);
            mapSegmentCorners[14, 1, 0] = (8.477 - 6.41 - (9.399 - 7.612));
            mapSegmentCorners[14, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984);//pillars

            mapSegmentCorners[15, 0, 0] = (8.477 - 6.41 - (9.399 - 10.973));
            mapSegmentCorners[15, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984);
            mapSegmentCorners[15, 1, 0] = (8.477 - 6.41 - (9.399 - 11.302));
            mapSegmentCorners[15, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984);//pillars

            mapSegmentCorners[16, 0, 0] = (8.477 - 6.41 - (9.399 - 14.630));
            mapSegmentCorners[16, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984);
            mapSegmentCorners[16, 1, 0] = (8.477 - 6.41 - (9.399 - 14.966));
            mapSegmentCorners[16, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984);//pillars

            mapSegmentCorners[17, 0, 0] = (8.477 - 6.41 - (9.399 - 18.637));
            mapSegmentCorners[17, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984);
            mapSegmentCorners[17, 1, 0] = (8.477 - 6.41 - (9.399 - 18.315));
            mapSegmentCorners[17, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984);//pillars
            ////other side

            mapSegmentCorners[18, 0, 0] = (8.477 - 6.41 - (9.399 - 3.671));
            mapSegmentCorners[18, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);
            mapSegmentCorners[18, 1, 0] = (8.477 - 6.41 - (9.399 - 3.984));
            mapSegmentCorners[18, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);

            mapSegmentCorners[23, 0, 0] = (8.477 - 6.41 - (9.399 - 3.671));
            mapSegmentCorners[23, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162+(6.8-6.498));
            mapSegmentCorners[23, 1, 0] = (8.477 - 6.41 - (9.399 - 3.984));
            mapSegmentCorners[23, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162+(6.8 - 6.498));

            mapSegmentCorners[19, 0, 0] = (8.477 - 6.41 - (9.399 - 7.308));
            mapSegmentCorners[19, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);
            mapSegmentCorners[19, 1, 0] = (8.477 - 6.41 - (9.399 - 7.612));
            mapSegmentCorners[19, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);//pillars

            mapSegmentCorners[20, 0, 0] = (8.477 - 6.41 - (9.399 - 10.973));
            mapSegmentCorners[20, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);
            mapSegmentCorners[20, 1, 0] = (8.477 - 6.41 - (9.399 - 11.302));
            mapSegmentCorners[20, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);//pillars

            mapSegmentCorners[21, 0, 0] = (8.477 - 6.41 - (9.399 - 14.630));
            mapSegmentCorners[21, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);
            mapSegmentCorners[21, 1, 0] = (8.477 - 6.41 - (9.399 - 14.966));
            mapSegmentCorners[21, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);//pillars

            mapSegmentCorners[22, 0, 0] = (8.477 - 6.41 - (9.399 - 18.637));
            mapSegmentCorners[22, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);
            mapSegmentCorners[22, 1, 0] = (8.477 - 6.41 - (9.399 - 18.315));
            mapSegmentCorners[22, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162);//pillars

            //pillar with width
            mapSegmentCorners[24, 0, 0] = (8.477 - 6.41 - (9.399 - 7.308));
            mapSegmentCorners[24, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));
            mapSegmentCorners[24, 1, 0] = (8.477 - 6.41 - (9.399 - 7.612));
            mapSegmentCorners[24, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));//pillars

            mapSegmentCorners[25, 0, 0] = (8.477 - 6.41 - (9.399 - 10.973));
            mapSegmentCorners[25, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));
            mapSegmentCorners[25, 1, 0] = (8.477 - 6.41 - (9.399 - 11.302));
            mapSegmentCorners[25, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));//pillars

            mapSegmentCorners[26, 0, 0] = (8.477 - 6.41 - (9.399 - 14.630));
            mapSegmentCorners[26, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));
            mapSegmentCorners[26, 1, 0] = (8.477 - 6.41 - (9.399 - 14.966));
            mapSegmentCorners[26, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));//pillars

            mapSegmentCorners[27, 0, 0] = (8.477 - 6.41 - (9.399 - 18.637));
            mapSegmentCorners[27, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));
            mapSegmentCorners[27, 1, 0] = (8.477 - 6.41 - (9.399 - 18.315));
            mapSegmentCorners[27, 1, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498));//pillars


            /////////////////////////////////////
            mapSegmentCorners[28, 0, 0] = (8.477 - 6.41 +(9.401-9.317));
            mapSegmentCorners[28, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498)+6.425);
            mapSegmentCorners[28, 1, 0] = (8.477 - 6.41 + (9.401 - 9.317)-(8.478-2.964));
            mapSegmentCorners[28, 1, 1] = -(1.865 - 0.07 + 1.13 + 17.163);//other Wall

            mapSegmentCorners[29, 0, 0] = (8.477 - 6.41 + (9.401 - 9.317) + 3.518);
            mapSegmentCorners[29, 0, 1] = -(1.865 - 0.07 + 1.13 + 6.984 + 3.162 + (6.8 - 6.498) + 6.425);
            mapSegmentCorners[29, 1, 0] = (8.477 - 6.41 + (9.401 - 9.317) + 3.518 + (20.025-6.936));
            mapSegmentCorners[29, 1, 1] = -(1.865 - 0.07 + 1.13 + 17.163);//other Wall
            //vertical walls 
            
            /////////////////////////////////////////////////////////////s
           /* numMapSegments = 8;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];

            mapSegmentCorners[0, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
            mapSegmentCorners[0, 0, 1] = 2.794;
            mapSegmentCorners[0, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[0, 1, 1] = 2.794;

            mapSegmentCorners[1, 0, 0] = -3.55 / 2;
            mapSegmentCorners[1, 0, 1] = 0.0;
            mapSegmentCorners[1, 1, 0] = -3.55 / 2;
            mapSegmentCorners[1, 1, 1] = -2.74;

            mapSegmentCorners[2, 0, 0] = 3.55 / 2;
            mapSegmentCorners[2, 0, 1] = 0.0;
            mapSegmentCorners[2, 1, 0] = 3.55 / 2;
            mapSegmentCorners[2, 1, 1] = -2.74;

            mapSegmentCorners[3, 0, 0] = 3.55 / 2;
            mapSegmentCorners[3, 0, 1] = 0.0;
            mapSegmentCorners[3, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[3, 1, 1] = 0.0;

            mapSegmentCorners[4, 0, 0] = -3.55 / 2;
            mapSegmentCorners[4, 0, 1] = 0.0;
            mapSegmentCorners[4, 1, 0] = -3.55 / 2 - 5.79;
            mapSegmentCorners[4, 1, 1] = 0.0;

            mapSegmentCorners[5, 0, 0] = -3.55 / 2;
            mapSegmentCorners[5, 0, 1] = -2.74;
            mapSegmentCorners[5, 1, 0] = -3.55 / 2 - 3.05;
            mapSegmentCorners[5, 1, 1] = -2.74;

            mapSegmentCorners[6, 0, 0] = 3.55 / 2;
            mapSegmentCorners[6, 0, 1] = -2.74;
            mapSegmentCorners[6, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[6, 1, 1] = -2.74;

            mapSegmentCorners[7, 0, 0] = 5.03 / 2;
            mapSegmentCorners[7, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[7, 1, 0] = -5.03 / 2;
            mapSegmentCorners[7, 1, 1] = -2.74 - 2.31;*/
            
            // ****************** Additional Student Code: End   ************
#endregion
            // Set map parameters
            // These will be useful in your future coding.
            minX = 9999; minY = 9999; maxX = -9999; maxY = -9999;
            for (int i = 0; i < numMapSegments; i++)
            {

                // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i, 0, 0], mapSegmentCorners[i, 1, 0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i, 0, 1], mapSegmentCorners[i, 1, 1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i, 0, 0], mapSegmentCorners[i, 1, 0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i, 0, 1], mapSegmentCorners[i, 1, 1]));

                // Set wall segments to be horizontal
                slopes[i] = (mapSegmentCorners[i, 0, 1] - mapSegmentCorners[i, 1, 1]) / (0.001 + mapSegmentCorners[i, 0, 0] - mapSegmentCorners[i, 1, 0]);
                intercepts[i] = mapSegmentCorners[i, 0, 1] - slopes[i] * mapSegmentCorners[i, 0, 0];

                // Set wall segment lengths
                segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i, 0, 0] - mapSegmentCorners[i, 1, 0], 2) + Math.Pow(mapSegmentCorners[i, 0, 1] - mapSegmentCorners[i, 1, 1], 2));
            }

            //hardcode the throw zones
            numThrowZ = 4;
            Zone z1 = new Zone(-10.945, 0, 0, 2.74);
            Zone z2 = new Zone( 0, 10.945, 0, 2.74);
            Zone z3 = new Zone(-1.775, 1.775, -2.74, 0);
            Zone z4 = new Zone(-4.825, 4.825, -5.05, -2.74);
            mapthrowZ = new List<Zone>();
            mapthrowZ.Add(z1);
            mapthrowZ.Add(z2);
            mapthrowZ.Add(z3);
            mapthrowZ.Add(z4);

        }



        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)
        double GetWallDistance(double x, double y, double t, int segment)
        {

            double wallDist;

            double X1 = mapSegmentCorners[segment, 0, 0];
            double Y1 = mapSegmentCorners[segment, 0, 1];
            double X2 = mapSegmentCorners[segment, 1, 0];
            double Y2 = mapSegmentCorners[segment, 1, 1];

         
            double intersectX = (intercepts[segment] + Math.Tan(t) * x - y) / (Math.Tan(t) - slopes[segment]);
            double intersectY = slopes[segment] * intersectX + intercepts[segment];


            bool exists = inRange(intersectX, X1, X2) && inRange(intersectY, Y1, Y2) && inFront(intersectX - x, intersectY - y, t);
            if (exists)
                wallDist = Math.Sqrt(Math.Pow(x - intersectX, 2) + Math.Pow(y - intersectY, 2));
            else
                wallDist = 0;
            // ****************** Additional Student Code: End   ************

            return wallDist;
        }


        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        /*public double GetClosestWallDistance(double x, double y, double t)
        {

            double minDist = 36.000;

            // ****************** Additional Student Code: Start ************

            // Put code here that loops through segments, calling the
            // function GetWallDistance.
            for (int i = 0; i < numMapSegments; i++)
                minDist = Math.Min(minDist, GetWallDistance(x, y, t, i));


            // ****************** Additional Student Code: End   ************

            return minDist;
        }*/
        public double GetClosestWallDistance(double x, double y, double t)
        {
            double maxDist = 6.0;
            double minDist = maxDist;
            double dist = 0;
            int seg = 0;
            // ****************** Additional Student Code: Start ************

            for (int i = 0; i < numMapSegments; i++)
            {
                if (!WallClose(x, y, i))
                    continue;

                dist = GetWallDistance(x, y, t, i);
                if (dist > 0)
                {
                    minDist = Math.Min(minDist, dist);
                    if (minDist == dist)
                        seg = i;
                }
            }

            // ****************** Additional Student Code: End   ************

            if (minDist == maxDist)
                return 0;

            return minDist;
        }

        // Checks if a map in the wall is close enough to the robot to consider for GetClosestWall. 
        // Returns True if the wall is "close" as defined by thres, false otherwise.
        private bool WallClose(double botX, double botY, int segment)
        {
            double thres = 6;

            double dx1 = botX - mapSegmentCorners[segment, 0, 0];
            double dy1 = botY - mapSegmentCorners[segment, 0, 1];
            double dx2 = botX - mapSegmentCorners[segment, 1, 0];
            double dy2 = botY - mapSegmentCorners[segment, 1, 1];

            if (dx1 * dx1 + dy1 * dy1 < thres * thres) return true;
            if (dx2 * dx2 + dy2 * dy2 < thres * thres) return true;
            return false;
        }

    
        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        public bool CollisionFound(Navigation.Node n1, Navigation.Node n2, double tol)
        {


            // Check that within boundaries
            if (n2.x > maxWorkspaceX || n2.x < minWorkspaceX || n2.y > maxWorkspaceY || n2.y < minWorkspaceY)
                return true;


            // Check for collision with walls
            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x);
            double edgeSize = Math.Sqrt(Math.Pow(n2.y - n1.y, 2) + Math.Pow(n2.x - n1.x, 2));
            double sinTheta = Math.Sin(theta);
            double cosTheta = Math.Cos(theta);

            // Loop through segments
            for (int segment = 0; segment < numMapSegments; segment++)
            {

                double distTravelledOnEdge = 0;
                double ex = n1.x, ey = n1.y;
                double distToSegment;
                while (distTravelledOnEdge - tol < edgeSize)
                {
                    distToSegment = GetWallDistance(ex, ey, segment, tol, n2.x, n2.y);
                    if (distToSegment - tol < 0.05)
                        return true;
                    ex += cosTheta * distToSegment;
                    ey += sinTheta * distToSegment;
                    distTravelledOnEdge += distToSegment;
                }

            }
            return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y)
        {
            // Set wall vars
            double X1 = mapSegmentCorners[segment, 0, 0];
            double Y1 = mapSegmentCorners[segment, 0, 1];
            double X2 = mapSegmentCorners[segment, 1, 0];
            double Y2 = mapSegmentCorners[segment, 1, 1];
            double dist = 9999;

            // Put code here to calculated dist.
            // Calculate slope and intercept
            double angleSegmentPerpendicular = Math.PI / 2 + Math.Atan((Y2 - Y1) / (0.000001 + X2 - X1));
            double m = Math.Tan(angleSegmentPerpendicular);
            double b = y - m * x;

            // Get line intersection
            double x_intersect = (b - intercepts[segment]) / (slopes[segment] - m);
            double y_intersect = m * x_intersect + b;

            // Check for horiz/vert slopes
            if (Math.Abs(Y2 - Y1) < 0.001)
                y_intersect = Y1;
            if (Math.Abs(X2 - X1) < 0.001)
                x_intersect = X1;


            // Check to see if intersection LIES within segment
            double dist_intersect_corner1 = Math.Sqrt(Math.Pow(x_intersect - X1, 2) + Math.Pow(y_intersect - Y1, 2));
            double dist_intersect_corner2 = Math.Sqrt(Math.Pow(x_intersect - X2, 2) + Math.Pow(y_intersect - Y2, 2));
            if (dist_intersect_corner1 <= (segmentSizes[segment] + tol) && dist_intersect_corner2 <= (segmentSizes[segment] + tol))
            {
                dist = Math.Sqrt(Math.Pow(x - x_intersect, 2) + Math.Pow(y - y_intersect, 2));
            }

            // Check for distance to corners (for case where no intersection with segment
            double dist_point_corner1 = Math.Sqrt(Math.Pow(x - X1, 2) + Math.Pow(y - Y1, 2));
            double dist_point_corner2 = Math.Sqrt(Math.Pow(x - X2, 2) + Math.Pow(y - Y2, 2));
            dist = Math.Min(dist, dist_point_corner1);
            dist = Math.Min(dist, dist_point_corner2);

            return dist;
        }


        /*********************************************************************************************************
         * 
         *                          HELPER FUNCTIONS
         * ***********************************************************************************************************
         * 
         */
 
        private bool inRange(double a, double one, double two)
        {
            return (a <= Math.Max(one, two) && a >= Math.Min(one, two));
        }

        private bool inFront(double dx, double dy, double t)
        {
            return (Math.Abs(Math.Atan2(dy, dx) - t) < 0.01);
        }

        public bool inMapArea(double xloc, double yloc)
        {
            for (int i = 0; i < numThrowZ; i++)
            {
                double boundxMin = mapthrowZ[i].minx();
                double boundxMax = mapthrowZ[i].maxx();
                double boundyMin = mapthrowZ[i].miny();
                double boundyMax = mapthrowZ[i].maxy();

                if (inRange(xloc, boundxMin, boundxMax) && inRange(yloc, boundyMax, boundyMin))
                    return true;
            }
            return false;
        }

    }
}
