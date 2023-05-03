using System;
using System.Collections.Generic;
using System.Reactive.Linq;
using System.Text;
using System.Windows.Input;
using ReactiveUI;


namespace canApp.ViewModels;

public class ViewModelBase : ReactiveObject
{
    private int _selected_com;
    public int Selected_com
    {
        get => _selected_com;
        set => this.RaiseAndSetIfChanged(ref _selected_com, value);
    }
    
    private string _recived_data;
    public string Recived_data
    {
        get => _recived_data;
        set => this.RaiseAndSetIfChanged(ref _recived_data, value);
    }
}
    
    
    
