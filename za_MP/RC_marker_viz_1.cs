using MissionPlanner;
using MissionPlanner.Plugin;
using MissionPlanner.Utilities;
//using MissionPlanner.Controls;
using MissionPlanner.Comms;
//using MissionPlanner.ArduPilot;
//using MissionPlanner.Controls.PreFlight;
using MissionPlanner.GCSViews;
using System;
using System.Collections.Generic;
using System.Globalization;
//using System.IO;
//using System.Windows.Forms;
//using System.Net.Sockets;
//using System.Diagnostics;
//using System.Linq;
//using System.Drawing;
//using System.Runtime.Serialization;
using GMap.NET;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;
//using System.Linq.Expressions;
using System.Threading;
//using MissionPlanner.Radio;

namespace RoboCamp_test1
{
    public class MyNewPlugin : Plugin
    {
        private readonly string _Name = "RoboCamp marker visualization";
        private readonly string _Version = "1-27.01.25.";
        private readonly string _Author = "Patrik - CRTA";

        public override string Name { get { return _Name; } }
        public override string Version { get { return _Version; } }
        public override string Author { get { return _Author; } }

        static bool threadrun = false;
        static internal ICommsSerial comPort = new SerialPort();
        private Thread _readThread;

        private GMapOverlay _markerOverlay;
        private Dictionary<string, GMarkerGoogle> _markers;

        public override bool Init()
        {

            _markerOverlay = new GMapOverlay("Markers");
            _markers = new Dictionary<string, GMarkerGoogle>();
            FlightData.instance.gMapControl1.Overlays.Add(_markerOverlay);

            loopratehz = 0.25f;  // Loop runs every 4 seconds

            return true;  // If it is false then plugin will not load
        }

        public override bool Loaded()
        {
            comPort.BaudRate = 115200;
            comPort.PortName = "/dev/ttyROBO1";

            try
            {
                comPort.Open();
                threadrun = true;
                CustomMessageBox.Show("RoboCamp marker-viz plugin initialized");

                // Start the read thread
                _readThread = new Thread(ReadSerialData);
                _readThread.Start();
            }
            catch (Exception ex)
            {
                CustomMessageBox.Show("RoboCamp " + Strings.ErrorConnecting); // + "\n" + ex.ToString(), Strings.ERROR);
            }
            return true;  // If it is false plugin will not start (loop will not be called)
        }

        private void ReadSerialData()
        {
            while (threadrun)
            {
                try
                {
                    if (comPort.IsOpen)
                    {
                        string line = comPort.ReadLine();
                        Console.WriteLine(line);
                        ProcessData(line);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error reading from serial port: " + ex.Message);
                }
            }
        }

        private void ProcessData(string data)
        {
            var items = data.Split(',');

            if (items.Length == 6)
            {
                string name = items[0];
                double lat = double.Parse(items[3], CultureInfo.InvariantCulture);
                double lng = double.Parse(items[4], CultureInfo.InvariantCulture);
                int type = int.Parse(items[1]);
                int state = int.Parse(items[2]);

                PointLatLng point = new PointLatLng(lat, lng);
                GMarkerGoogle marker = CreateMarker(point, type, state, name);

                // Update or add marker
                // (Assuming _markers and _markerOverlay are defined and initialized)
                if (_markers.ContainsKey(name))
                {
                    _markers[name] = marker;
                }
                else
                {
                    _markers.Add(name, marker);
                }

                // Refresh the overlay
                RefreshOverlay();
            }
            else if (items.Length == 2)
            {
            	string name = items[0];
            	int cmd = int.Parse(items[1]);
            	
            	if (cmd == 0)		// clear all markers, name is irrelevant
            	{
            	    _markers.Clear();
            	}
            	else if (cmd == 1)	// clear selected marker, key is name
            	{
            	    if (_markers.ContainsKey(name))
            	    {
            		_markers.Remove(name);
            	    }
            	}
            	else if (cmd == 2)	// modify selected marker to inactive state
            	{
            	    ModifyMarkerType(name, GMarkerGoogleType.red);
            	}
            	else if (cmd == 3)	// modify selected marker to confirmed inactive state
            	{
            	    ModifyMarkerType(name, GMarkerGoogleType.red_dot);
            	}
            	RefreshOverlay();
            }
        }

        private GMarkerGoogle CreateMarker(PointLatLng point, int type, int state, string name)
        {
            GMarkerGoogle marker;
            switch (type)
            {
                case 0:
                    marker = new GMarkerGoogle(point, GMarkerGoogleType.red_dot);
                    break;
                case 1:
                    if (state == 1)
                    	marker = new GMarkerGoogle(point, GMarkerGoogleType.green_dot);
		     else
		     	marker = new GMarkerGoogle(point, GMarkerGoogleType.green);
                    break;
                case 2:
                    if (state == 1)
                    	marker = new GMarkerGoogle(point, GMarkerGoogleType.blue_dot);
		     else
		     	marker = new GMarkerGoogle(point, GMarkerGoogleType.blue);
                    break;
                case 3:
                    if (state == 1)
                    	marker = new GMarkerGoogle(point, GMarkerGoogleType.yellow_dot);
		     else
		     	marker = new GMarkerGoogle(point, GMarkerGoogleType.yellow);
                    break;
                default:
                    marker = new GMarkerGoogle(point, GMarkerGoogleType.red);
                    break;
            }
            marker.ToolTipText = name;
            return marker;
        }

        private void RefreshOverlay()
        {
            _markerOverlay.Markers.Clear();

            foreach (var marker in _markers.Values)
            {
                _markerOverlay.Markers.Add(marker);
            }
        }

        public override bool Loop()
        {
            if (!threadrun)
            {
                try
                {
                    comPort.Open();
                    threadrun = true;
                    Console.WriteLine("RoboCamp marker-viz connected");
                    CustomMessageBox.Show("RoboCamp marker-viz connected");

                    // Start the read thread
                    _readThread = new Thread(ReadSerialData);
                    _readThread.Start();
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error opening serial port: " + ex.Message);
                }
            }

            return true;  // Return value is not used
        }

        public override bool Exit()
        {
            // Clean up resources
            threadrun = false;
            if (_readThread != null && _readThread.IsAlive)
            {
                _readThread.Join();
            }

            if (comPort != null && comPort.IsOpen)
            {
                comPort.Close();
            }

            CustomMessageBox.Show("Exiting RoboCamp");

            return true;  // Return value is not used
        }
        
        private void ModifyMarkerType(string name, GMarkerGoogleType newType)
        {
            if (_markers.ContainsKey(name))
            {
                var marker = _markers[name];
                var newMarker = new GMarkerGoogle(marker.Position, newType)
                {
                    ToolTipText = marker.ToolTipText
                };
                _markers[name] = newMarker;
                //RefreshOverlay();
            }
        }
    }
}
