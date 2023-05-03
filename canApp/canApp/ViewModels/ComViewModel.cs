using System.Collections;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using canApp.Models;

namespace canApp.ViewModels;

public class ComViewModel : ViewModelBase
{
    public ComViewModel(IEnumerable<ComPort> items)
    {
        Items = new ObservableCollection<ComPort>(items);
    }
    
    public ObservableCollection<ComPort> Items { get; }


}