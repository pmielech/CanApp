using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Markup.Xaml;
using canApp.Services;
using canApp.ViewModels;
using canApp.Views;

namespace canApp;

public partial class App : Application
{
    public override void Initialize()
    {
        AvaloniaXamlLoader.Load(this);
    }

    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            var cp = new DebugComList();
            desktop.MainWindow = new MainWindow
            {
                DataContext = new MainWindowViewModel(cp),
            };
        }

        base.OnFrameworkInitializationCompleted();
    }
}