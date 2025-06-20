using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using MissionPlanner.Plugin;
using MissionPlanner.Utilities;
using MissionPlanner.Controls;
using MissionPlanner.Maps;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;

public class RoboCampPlugin : Plugin
{
    private string _Name = "Marker-viz plugin";
    private string _Version = "1.0 23.01.2025.";
    private string _Author = "Patrik - CRTA";
    private SerialPort _serialPort;
    private GMapOverlay _poiOverlay;
    private Dictionary<string, GMapMarker> _markers;
    private Control _uiControl;

    public override string Name => _Name;
    public override string Version => _Version;
    public override string Author => _Author;

    public override bool Init()
    {
        // Initialize the overlay and markers dictionary
        _poiOverlay = new GMapOverlay("POIs");
        _markers = new Dictionary<string, GMapMarker>();

        // Find the main form control to use for UI updates
        _uiControl = Application.OpenForms[0];

        loopratehz = 0;

        return true;
    }

    public override bool Loaded()
    {
        // Initialize the COM port
        _serialPort = new SerialPort("/dev/ttyROBO1", 115200);
        _serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
        _serialPort.Open();

        // Add the overlay to the map control
        if (Host.FlightPlanner != null)
        {
            Host.FlightPlanner.Map.Overlays.Add(_poiOverlay);
        }

        return true;
    }

    private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
    {
        SerialPort sp = (SerialPort)sender;
        string data = sp.ReadLine();
        _uiControl.Invoke(new Action(() => ProcessData(data)));
    }

    private void ProcessData(string data)
    {
        var items = data.Split(',');

        if (items.Length == 6)
        {
            string name = items[0];
            double lat = double.Parse(items[3], CultureInfo.InvariantCulture);
            double lng = double.Parse(items[4], CultureInfo.InvariantCulture);
            int type = int.Parse(items[2]);

            PointLatLng point = new PointLatLng(lat, lng);
            GMapMarker marker = CreateMarker(point, type, name);

            // Update or add marker
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
    }

    private GMapMarker CreateMarker(PointLatLng point, int type, string name)
    {
        GMapMarker marker;
        switch (type)
        {
            case 0:
                marker = new GMarkerGoogle(point, GMarkerGoogleType.red_dot);
                break;
            case 1:
                marker = new GMarkerGoogle(point, GMarkerGoogleType.green_dot);
                break;
            case 2:
                marker = new GMarkerGoogle(point, GMarkerGoogleType.blue_dot);
                break;
            default:
                marker = new GMarkerGoogle(point, GMarkerGoogleType.yellow_dot);
                break;
        }
        marker.ToolTipText = name;
        return marker;
    }

    private void RefreshOverlay()
    {
        _poiOverlay.Clear();

        foreach (var marker in _markers.Values)
        {
            _poiOverlay.Markers.Add(marker);
        }
    }

    public override bool Loop()
    {
        // Perform any periodic tasks here
        return true;
    }

    public override bool Exit()
    {
        // Clean up resources
        if (_serialPort != null && _serialPort.IsOpen)
        {
            _serialPort.Close();
        }
        return true;
    }
}
