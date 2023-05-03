using System.Collections.Generic;
using canApp.Models;

namespace canApp.Services;

public class DebugComList
{
    public IEnumerable<ComPort> GetItems() => new[]
    {
        new ComPort { Name = "Refresh Com List" },

    };
}