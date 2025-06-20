using MissionPlanner;
using MissionPlanner.Plugin;
using MissionPlanner.Utilities;
using MissionPlanner.Controls;
using MissionPlanner.Comms;
using MissionPlanner.ArduPilot;
using MissionPlanner.Controls.PreFlight;
using MissionPlanner.GCSViews;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Windows.Forms;
using System.Net.Sockets;
using System.Diagnostics;
using System.Linq;
using System.Drawing;
using System.Runtime.Serialization;
using GMap.NET;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;
using System.Linq.Expressions;
using MissionPlanner.Radio;

namespace RoboCamp_test1
{
    public class MyNewPlugin : Plugin

    {
        private readonly string _Name = "Marker Visualization";
        private readonly string _Version = "0-23.01.25.";
        private readonly string _Author = "Patrik - CRTA";

        public override string Name { get { return _Name; } }
        public override string Version { get { return _Version; } }
        public override string Author { get { return _Author; } }

        static bool threadrun = false;

        static internal ICommsSerial comPort = new SerialPort();

        public override bool Init()
        //Init called when the plugin dll is loaded
        {
            loopratehz = 0.25f;  //Loop runs every second (The value is in Hertz, so 2 means every 500ms, 0.1f means every 10 second...) 

            return true;	 // If it is false then plugin will not load
        }

        public override bool Loaded()
        //Loaded called after the plugin dll successfully loaded
        {
 
            comPort.BaudRate = 115200;
            //comPort.PortName = "COM5";
            comPort.PortName = "/dev/ttyROBO1";

            try
            {
                comPort.Open();
                threadrun = true;
                CustomMessageBox.Show("RoboCamp marker-viz plugin initialized");
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show("RoboCamp " + Strings.ErrorConnecting + "\n"); // + ex.ToString(), Strings.ERROR);
            }
            return true;     //If it is false plugin will not start (loop will not called)
        
        }

        public override bool Loop()
        //Loop is called in regular intervalls (set by loopratehz)
        {
            DateTime nextsend = DateTime.Now;

            StreamWriter sw = new StreamWriter(File.OpenWrite(Settings.GetUserDataDirectory() + "robocb_raw.txt"));

            if (threadrun)
            {
                string line = comPort.ReadLine();

                sw.WriteLine(line);

                //string line = string.Format("192.168.0.216,gps,1,lat,lon,alt")
                string[] items = line.Split(',');
                Console.WriteLine(line);
                Console.WriteLine(items[0]);
            }

            else  // comPort not yet open
            {
                try
                {
                    comPort.Open();
                    threadrun = true;
                    //Console.WriteLine("RoboCamp marker-viz connected");
                    CustomMessageBox.Show("RoboCamp marker-viz connected");
                }
                catch
                {

                }
            }
            return true;	//Return value is not used
        }

        public override bool Exit()
        //Exit called when plugin is terminated (usually when Mission Planner is exiting)
        {
            CustomMessageBox.Show("Exiting RoboCamp");
            return true;	//Return value is not used
        }
    }
}
