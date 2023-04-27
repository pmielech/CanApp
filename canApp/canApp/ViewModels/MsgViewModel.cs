using System.Text;
using System.Threading.Tasks;
using ReactiveUI;

namespace canApp.ViewModels
{

    public class MsgViewModel: ViewModelBase
    {
        private int _msgCount;
        private string _msgText;
        private string _toBeSentText;
        
        public int MsgCount
        {
            get => _msgCount;
            set => this.RaiseAndSetIfChanged(ref _msgCount, value);
        }
        public string MsgText
        {
            get => _msgText;
            set => this.RaiseAndSetIfChanged(ref _msgText, value);
        }
        public string ToBeSentText
        {
            get => _toBeSentText;
            set => this.RaiseAndSetIfChanged(ref _toBeSentText, value);
        }

        public MsgViewModel()
        {
            MsgCount = 0;
            MsgText = "";
            ToBeSentText = "";

        }

        public void AddSentMessage(string msg)
        {
            
        }

        public void WriteToBox(string text)
        {

            MsgText += text + "\n";
        }
        

    }


}
