using System.IO.Ports;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Linq;
using System.Text;
using System.Threading;
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
    public ICommand SendUserMessage { get; }

    public ICommand ClearReceivedData { get; }

    public ComViewModel ComList { get; }

    private readonly SerialPort _serial = new SerialPort();

    public MainWindowViewModel(DebugComList cp)
    {
        ComList = new ComViewModel(cp.GetItems());

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

            foreach (var port in serialPortList)
            {
                ComList.Items.Add(new ComPort() { Name = port });
            }
        });
        
        SendUserMessage = ReactiveCommand.Create(() =>
        {
            if (IsConnected)
            {
                try
                {
                    // Send the binary data out the port
                    byte[] hexstring = Encoding.ASCII.GetBytes(UserInput += '\n');
                    //There is a intermitant problem that I came across
                    //If I write more than one byte in succesion without a 
                    //delay the PIC i'm communicating with will Crash
                    //I expect this id due to PC timing issues ad they are
                    //not directley connected to the COM port the solution
                    //Is a ver small 1 millisecound delay between chracters
                    foreach (byte hexValue in hexstring)
                    {
                        byte[] _hexValue = new byte[] { hexValue }; // need to convert byte to byte[] to write
                        _serial.Write(_hexValue, 0, 1);
                        Thread.Sleep(1);
                    }

                    var dat = DateTime.Now.ToString("HH:mm:ss.ffff");
                    SerialData += $"==> TX<{dat}> ~ {UserInput}";
                    UserInput = "";

                }
                catch (Exception)
                {
                    SerialData += "~~~Failed to send data!~~~\n";

                }
            }
        });     
            
        ClearReceivedData = ReactiveCommand.Create(() =>
        {
            SerialData = "";
        });

        ConnectToComCommand = ReactiveCommand.Create(() =>
        {
            if (ConnectionButton == "Connect")
            {
                //TODO Exception when choosing wrong com port (unix) 
                _serial.PortName = ComList.Items.ElementAt(SelectedCom).Name;
                _serial.BaudRate = 115200;
                _serial.Handshake = System.IO.Ports.Handshake.None;
                _serial.Parity = Parity.None;
                _serial.DataBits = 8;
                _serial.StopBits = StopBits.Two;
                _serial.ReadTimeout = 200;
                _serial.WriteTimeout = 50;
                try
                {
                    _serial.Open();
                    IsConnected = _serial.IsOpen;
                    if (IsConnected == true)
                    {
                        StatusColor = "Green";
                    }
                    SerialData += "~~~Connected Successful!~~~\n";
                    Thread.Sleep(10);
                    //_serial.ReadLine() += new System.IO.Ports.SerialDataReceivedEventHandler(Receive);
                    _serial.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(Receive);
                    ConnectionButton = "Disconnect";


                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                    throw;
                }

            }
            else
            {
                try
                {
                    _serial.Close();
                    IsConnected = _serial.IsOpen;
                    if (IsConnected == false)
                    {
                        StatusColor = "Red";
                    }

                    ConnectionButton = "Connect";

                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                    throw;
                }
                
            }
            
        });

    }

    private void Receive(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
    {
        try
        {
            var date = DateTime.Now.ToString("HH:mm:ss.ffff");
            string message = _serial.ReadLine().Trim('\r', '\n');
            if (message.Length == 32)
            {
                SerialData += $"<== Rx<{date}> ~ {message}\n";
            }
            
        }
        catch (Exception exception)
        {
            Console.WriteLine(exception);
        }
        
    }

}
