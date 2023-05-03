using System.IO.Ports;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Linq;
using System.Text;
using System.Windows.Input;
using Avalonia;
using Avalonia.Threading;
using canApp.Models;
using ReactiveUI;
using canApp.Services;

namespace canApp.ViewModels;

public class MainWindowViewModel : ViewModelBase
{
    public ICommand RefreshComPortsCommand { get; }
    public ICommand ConnectToComCommand { get; }
    public ComViewModel ComList { get; }

    private SerialPort serial = new SerialPort();

    public MainWindowViewModel(DebugComList cp)
    {

        ComList = new ComViewModel(cp.GetItems());
        Selected_com = 0;

        RefreshComPortsCommand = ReactiveCommand.Create(() =>
        {
            string[] serialPortList = SerialPort.GetPortNames();
            try
            {
                ComList.Items.Clear();
            }
            catch (Exception e)
            {
                Console.WriteLine(e);
                throw;
            }

            for (int i = 0; i < serialPortList.Length; i++)
            {
                ComList.Items.Add(new ComPort() { Name = serialPortList[i] });
            }
        });

        ConnectToComCommand = ReactiveCommand.Create(() =>
        {

            serial.PortName = ComList.Items.ElementAt(Selected_com).Name;
            serial.BaudRate = 9600;
            serial.Handshake = System.IO.Ports.Handshake.None;
            serial.Parity = Parity.None;
            serial.DataBits = 8;
            serial.StopBits = StopBits.Two;
            serial.ReadTimeout = 200;
            serial.WriteTimeout = 50;
            try
            {
                serial.Open();
                serial.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(Recieve);

            }
            catch (Exception e)
            {
                Console.WriteLine(e);
                throw;
            }

        });

    }

    private void Recieve(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
    {
        // Collecting the characters received to our 'buffer' (string).
        //Recived_data = "";
        //Recived_data = serial.ReadExisting(); 
        var serialDevice = sender as SerialPort;
        var buffer = new byte[serial.BytesToRead];
        serialDevice.Read(bytes, 0, buffer.Length);

        // process data on the GUI thread
        Application.Current.Dispatcher.Invoke(new Action(() => {
            
            
        }));
    }
    

}