using System;
using System.Collections.Generic;
using System.Reactive.Linq;
using System.Text;
using System.Windows.Input;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Threading;
using Avalonia.VisualTree;
using ReactiveUI;


namespace canApp.ViewModels;

public class ViewModelBase : ReactiveObject
{
    
    private string _statusColor = "Red";

    protected string StatusColor
    {
        get => _statusColor;
        set => this.RaiseAndSetIfChanged(ref _statusColor, value);
    }
    
    
    private bool _isConnected = false;

    protected bool IsConnected
    {
        get => _isConnected;
        set => this.RaiseAndSetIfChanged(ref _isConnected, value);
    }
    
    private string _connectionButton = "Connect";

    protected string ConnectionButton
    {
        get => _connectionButton;
        set => this.RaiseAndSetIfChanged(ref _connectionButton, value);
    }
    
    private string _userInput = "";
    public string UserInput
    {
        get => _userInput;
        set => this.RaiseAndSetIfChanged(ref _userInput, value);
    }
    
    private int _selectedCom = 0;
    public int SelectedCom
    {
        get => _selectedCom;
        set => this.RaiseAndSetIfChanged(ref _selectedCom, value);
    }
    
    private int _selectedSpeed = 0;
    public int SelectedSpeed
    {
        get => _selectedSpeed;
        set => this.RaiseAndSetIfChanged(ref _selectedSpeed, value);
    }
    
    private string _serialData = "";
    public string SerialData
    {
        get => _serialData;
        set { this.RaiseAndSetIfChanged(ref _serialData, value);
        }
    }
    
    
}
    
    
    
