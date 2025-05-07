using LiveCharts;
using LiveCharts.Defaults;
using LiveCharts.Wpf;
using MyFisrtApp.Helpers;
using MyFisrtApp.SensorData;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;

namespace MyFisrtApp.ViewModel
{
    public partial class MainViewModel : INotifyPropertyChanged
    {
        private SerialPort serialPort = new SerialPort();

        private string _currentDateTimeDisplay;
        private string _temperatureDisplay;
        private string _axDisplay;
        private string _ayDisplay;
        private string _azDisplay;
        private string _gxDisplay;
        private string _gyDisplay;
        private string _gzDisplay;
        private string _rollDisplay;
        private string _pitchDisplay;

        private string csvFilePath;   // Verilerin kaydedileceği dosyanın yolunu tutan değişken.


        private readonly DispatcherTimer _timer;

        public ObservableCollection<string> LogList { get; set; } = new ObservableCollection<string>();
        public Brush TemperatureBackground { get; set; }
        public Brush AxBackground { get; set; }
        public Brush AyBackground { get; set; }
        public Brush AzBackground { get; set; }
        public Brush GxBackground { get; set; }
        public Brush GyBackground { get; set; }
        public Brush GzBackground { get; set; }
        public Brush RollDegreeBackground { get; set; }
        public Brush PitchDegreeBackground { get; set; }

        public ChartValues<DateTimePoint> TemperatureValues { get; set; } // Sıcaklık verilerini tutacak.
        public SeriesCollection TemperatureSeries { get; set; }  // Sıcaklık grafiğindeki çizgilerin listesini tutacak.
        public Func<double, string> TimeFormatter { get; set; } = val => new DateTime((long)val).ToString("HH:mm:ss"); // Sıcaklık grafiğinde zamanın gösterilmesi için oluşturuldu.

        public ChartValues<float> AxValues { get; set; }
        public ChartValues<float> AyValues { get; set; }
        public ChartValues<float> AzValues { get; set; }
        public SeriesCollection AccellSeries { get; set; }

        public ChartValues<float> GxValues { get; set; }
        public ChartValues<float> GyValues { get; set; }
        public ChartValues<float> GzValues { get; set; }
        public SeriesCollection GyroSeries { get; set; }

        public ICommand StartRecordingCommand { get; }   // GUI'de kayıt başlat butonuna tıklanınca tetiklenir.
        public ICommand StopRecordingCommand { get; }    // GUI'de kayıt durdur butonuna tıklanınca tetiklenir.

        private bool isRecording = false;                // Kayıt ile ilgili buton durumunu tutan değişken.

        private string _notificationMessage;
        public string NotificationMessage
        {
            get => _notificationMessage;
            set
            {
                _notificationMessage = value;
                OnPropertyChanged(nameof(NotificationMessage));
            }
        }
        public string CurrentDateTimeDisplay 
        { 
            get => _currentDateTimeDisplay;
            set
            {
                _currentDateTimeDisplay = value;
                OnPropertyChanged(nameof(CurrentDateTimeDisplay));
            }
        }

        public string TemperatureDisplay
        {
            get => _temperatureDisplay;
            set
            {
                _temperatureDisplay = value;
                OnPropertyChanged(nameof(TemperatureDisplay));

                float parsedTempValue = float.Parse(value.Replace(" °C", ""));

                TemperatureValues.Add(new DateTimePoint(DateTime.Now, parsedTempValue));
                if (TemperatureValues.Count > 50) TemperatureValues.RemoveAt(0);

                if (parsedTempValue >= SensorLimits.Temperature)
                {
                    TemperatureBackground = Brushes.Red;
                    OnPropertyChanged(nameof(TemperatureBackground));
                    LogList.Insert(0, $"⚠️ Temperature limit exceeded: {TemperatureDisplay} at {DateTime.Now:HH:mm:ss}");
                }
                else
                {
                    TemperatureBackground = (Brush)new BrushConverter().ConvertFrom("#191d22");
                    OnPropertyChanged(nameof(TemperatureBackground));
                    LogList.Insert(0, $" Temperature value changed: {TemperatureDisplay} at {DateTime.Now:HH:mm:ss}");
                }
            }
        }

        public string AxDisplay
        { 
            get => _axDisplay; 
            set 
            {
                _axDisplay = value;
                OnPropertyChanged(nameof(AxDisplay));
                float parsedAxValue = float.Parse(value.Replace(" g",""));

                AxValues.Add(parsedAxValue);
                if (AxValues.Count > 50) AxValues.RemoveAt(0);


                if (parsedAxValue > SensorLimits.Ax) 
                {
                    AxBackground = Brushes.Red;
                    OnPropertyChanged(nameof(AxBackground));
                    LogList.Insert(0, $"⚠️ Ax limit exceeded: {AxDisplay} at {DateTime.Now:HH:mm:ss}");
                }

                else
                {
                    AxBackground = (Brush)new BrushConverter().ConvertFrom("#191d22");
                    OnPropertyChanged(nameof(AxBackground));
                    LogList.Insert(0, $"Ax value changed: {AxDisplay} at {DateTime.Now:HH:mm:ss}");

                }
            } 
        }


        public string AyDisplay 
        { 
            get => _ayDisplay;
            set
            {
                _ayDisplay = value;
                OnPropertyChanged(nameof(AyDisplay));

                float parsedAyValue = float.Parse(value.Replace(" g", ""));

                AyValues.Add(parsedAyValue);
                if (AyValues.Count > 50) AyValues.RemoveAt(0);


                if (parsedAyValue >= SensorLimits.Ay)
                {
                    AyBackground = Brushes.Red;
                    OnPropertyChanged(nameof(AyBackground));
                    LogList.Insert(0, $"⚠️ Ay limit exceeded: {AyDisplay} at {DateTime.Now:HH:mm:ss}");
                }
                else
                {
                    AyBackground = (Brush)new BrushConverter().ConvertFrom("#191d22");
                    OnPropertyChanged(nameof(AyBackground));
                    LogList.Insert(0, $"Ay value changed: {AyDisplay} at {DateTime.Now:HH:mm:ss}");
                }

            }
        }

        public string AzDisplay
        {
            get => _azDisplay;
            set
            {
                _azDisplay = value;
                OnPropertyChanged(nameof(AzDisplay));

                float parsedAzValue = float.Parse(value.Replace(" g", ""));

                AzValues.Add(parsedAzValue);
                if (AzValues.Count > 100) AzValues.RemoveAt(0);


                if (parsedAzValue >= SensorLimits.Az)
                {
                    AzBackground = Brushes.Red;
                    OnPropertyChanged(nameof(AzBackground));
                    LogList.Insert(0, $"⚠️ Az limit exceeded: {AzDisplay} at {DateTime.Now:HH:mm:ss}");
                }
                else
                {
                    AzBackground = (Brush)new BrushConverter().ConvertFrom("#191d22");
                    OnPropertyChanged(nameof(AzBackground));
                    LogList.Insert(0, $"Az value changed: {AzDisplay} at {DateTime.Now:HH:mm:ss}");
                }


            }
        }
        public string GxDisplay { 
            get => _gxDisplay;
            set
            {
                _gxDisplay = value;
                OnPropertyChanged(nameof(GxDisplay));
                LogList.Insert(0, $"Gx value changed: {GxDisplay} at {DateTime.Now:HH:mm:ss}");

                float parsedGxValue = float.Parse(value.Replace(" °/s", ""));

                GxValues.Add(parsedGxValue);
                if (GxValues.Count > 50) GxValues.RemoveAt(0);

                if (parsedGxValue >= SensorLimits.Gx)
                {
                    GxBackground = Brushes.Red;
                    OnPropertyChanged(nameof(GxBackground));
                    LogList.Insert(0, $"⚠️ Gx limit exceeded: {GxDisplay} at {DateTime.Now:HH:mm:ss}");
                }
                else
                {
                    GxBackground = (Brush)new BrushConverter().ConvertFrom("#191d22");
                    OnPropertyChanged(nameof(GxBackground));
                    LogList.Insert(0, $"Gx value changed:  {GxDisplay} at {DateTime.Now:HH:mm:ss}");
                }
            }
        }

        public string GyDisplay { 
            get => _gyDisplay;
            set
            {
                _gyDisplay = value;
                OnPropertyChanged(nameof(GyDisplay));

                float parsedGyValue = float.Parse(value.Replace(" °/s", ""));

                GyValues.Add(parsedGyValue);
                if (GyValues.Count > 50) GyValues.RemoveAt(0);


                if (parsedGyValue >= SensorLimits.Gy)
                {
                    GyBackground = Brushes.Red;
                    OnPropertyChanged(nameof(GyBackground));
                    LogList.Insert(0, $"⚠️ Gy limit exceeded: {GyDisplay} at {DateTime.Now:HH:mm:ss}");
                }
                else
                {
                    GyBackground = (Brush)new BrushConverter().ConvertFrom("#191d22");
                    OnPropertyChanged(nameof(GyBackground));
                    LogList.Insert(0, $"Gy value changed:  {GyDisplay} at {DateTime.Now:HH:mm:ss}");
                }
            }
        }

        public string GzDisplay
        {
            get => _gzDisplay;
            set
            {
                _gzDisplay = value;
                OnPropertyChanged(nameof(GzDisplay));

                float parsedGzValue = float.Parse(value.Replace(" °/s", ""));

                GzValues.Add(parsedGzValue);
                if (GzValues.Count > 50) GzValues.RemoveAt(0);


                if (parsedGzValue >= SensorLimits.Gz)
                {
                    GzBackground = Brushes.Red;
                    OnPropertyChanged(nameof(GzBackground));
                    LogList.Insert(0, $"⚠️ Gz limit exceeded: {GzDisplay} at {DateTime.Now:HH:mm:ss}");
                }
                else
                {
                    GzBackground = (Brush)new BrushConverter().ConvertFrom("#191d22");
                    OnPropertyChanged(nameof(GzBackground));
                    LogList.Insert(0, $"Gz value changed:  {GzDisplay} at {DateTime.Now:HH:mm:ss}");
                }
            }
        }

        public string RollDisplay
        {
            get => _rollDisplay;
            set
            {
                _rollDisplay = value;
                OnPropertyChanged(nameof(RollDisplay));

                float parsedRollValue = float.Parse(value.Replace("°", ""));

                if (parsedRollValue >= SensorLimits.Roll)
                {
                    RollDegreeBackground = Brushes.Red;
                    OnPropertyChanged(nameof(RollDegreeBackground));
                    LogList.Insert(0, $"⚠️ Roll degree exceeded: {RollDisplay} at {DateTime.Now:HH:mm:ss}");
                }
                else
                {
                    RollDegreeBackground = (Brush)new BrushConverter().ConvertFrom("#191d22");
                    OnPropertyChanged(nameof(RollDegreeBackground));
                    LogList.Insert(0, $"Roll degree changed:  {RollDisplay} at {DateTime.Now:HH:mm:ss}");
                }
            }
        }

        public string PitchDisplay
        {
            get => _pitchDisplay;
            set
            {
                _pitchDisplay = value;
                OnPropertyChanged(nameof(PitchDisplay));

                float parsedPitchValue = float.Parse(value.Replace("°", ""));

                if (parsedPitchValue >= SensorLimits.Pitch)
                {
                    PitchDegreeBackground = Brushes.Red;
                    OnPropertyChanged(nameof(PitchDegreeBackground));
                    LogList.Insert(0, $"⚠ Pitch degree exceeded: {PitchDisplay} at {DateTime.Now:HH:mm:ss}");
                }
                else
                {
                    PitchDegreeBackground = (Brush)new BrushConverter().ConvertFrom("#191d22");
                    OnPropertyChanged(nameof(PitchDegreeBackground));
                    LogList.Insert(0, $"Pitch degree changed:  {PitchDisplay} at {DateTime.Now:HH:mm:ss}");
                }
            }
        }


        public MainViewModel()
        {
            serialPort.PortName = "COM15";
            serialPort.BaudRate = 115200;
            serialPort.DataReceived += SerialPort_DataReceived;
            serialPort.Open();

            
            _timer = new DispatcherTimer();
            _timer.Interval = TimeSpan.FromSeconds(1);
            _timer.Tick += (s, e) =>
            {
                CurrentDateTimeDisplay = DateTime.Now.ToString("dd.MM.yyyy  HH:mm:ss");
            };
            _timer.Start();

            TemperatureValues = new ChartValues<DateTimePoint>();  // Sıcaklık değerini yeni bir grafik değerine atama.
            TemperatureSeries = new SeriesCollection
            {
                new LineSeries
                {
                    Title = "Sıcaklık",
                    Values = TemperatureValues,
                    PointGeometrySize = 5,
                    Stroke = Brushes.Orange,
                    Fill = Brushes.Transparent,
                }
            };                                          // TemperatureValues verilerini kullanan bir LineSeries oluşturarak,
                                                        // bu seriyi TemperatureSeries koleksiyonuna ekler.
                                                        // Bu sayede grafik, sıcaklık değerlerini çizgi olarak gösterecek.

            AxValues = new ChartValues<float>();
            AyValues = new ChartValues<float>();
            AzValues = new ChartValues<float>();

            AccellSeries = new SeriesCollection
            {
                new ColumnSeries
                {
                    Title = "Ax",
                    Values = AxValues,
                },

                new ColumnSeries 
                {
                    Title = "Ay",
                    Values = AyValues
                },

                new ColumnSeries
                {
                    Title = "Az",
                    Values = AzValues
                }
            };

            GxValues = new ChartValues<float>();
            GyValues = new ChartValues<float>();
            GzValues = new ChartValues<float>();

            GyroSeries = new SeriesCollection
            {
                new ColumnSeries
                {
                    Title = "Gx",
                    Values = AxValues,
                },

                new ColumnSeries
                {
                    Title = "Gy",
                    Values = AyValues
                },

                new ColumnSeries
                {
                    Title = "Gz",
                    Values = AzValues
                }
            };

            string desktopPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);  // Masaüstü dizinini alma.

            string fileName = $"mpu6050Data_{DateTime.Now:yyyyMMdd_HHmmss}.csv";               // Verilerin yazılacağı dosyanın adı.

            csvFilePath = Path.Combine(desktopPath, fileName);                                // Dosyanın tam yolunu oluşturma.   

            File.AppendAllText(csvFilePath, "Time;Temperature;Ax;Ay;Az;Gx;Gy;Gz\n");         // Dosyaya başlıkları ekler.

            StartRecordingCommand = new RelayCommand(StartRecording);                       // Kayıt başlat butonuna tıklanınca StartRecording fonksiyonu çalışmaya başlar.
            StopRecordingCommand = new RelayCommand(StopRecording);                         // Kayıt durdur butonuna tıklanınca StopRecording fonksiyonu çalışmaya başlar.


        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            string jsonLine = serialPort.ReadLine();

            try
            {
                MPU6050SensorData data = JsonConvert.DeserializeObject<MPU6050SensorData>(jsonLine);

                App.Current.Dispatcher.Invoke(() =>
                {
                    float tempC = data.temperature / 100.0f;

                    float ax = data.ax / 10000.0f;
                    float ay = data.ay / 10000.0f;
                    float az = data.az / 10000.0f;

                    float gx = data.gx / 1000.0f;
                    float gy = data.gy / 1000.0f;
                    float gz = data.gz / 1000.0f;

                    float roll = data.roll / 100.0f;
                    float pitch = data.pitch / 100.0f;

                    TemperatureDisplay = $"{tempC} °C";

                    AxDisplay = $"{ax} g";
                    AyDisplay = $"{ay} g";
                    AzDisplay = $"{az} g";
                    
                    GxDisplay = $"{gx} °/s";
                    GyDisplay = $"{gy} °/s";
                    GzDisplay = $"{gz} °/s";

                    RollDisplay = $"{roll}°";
                    PitchDisplay = $"{pitch}°";

                    string time = DateTime.Now.ToString("HH:mm:ss");                                    // Şuan ki zamanı string formatında alma.

                    string csvLine = $"{time};{tempC};{ax};{ay};{az};{gx};{gy};{gz};{roll};{pitch}\n";   // Tüm verileri csv formatında birleştirir.

                    if (isRecording)
                    {
                        try
                        {
                            File.AppendAllText(csvFilePath, csvLine);                                      // csv formatındaki veriyi dosyaya yazma. 
                        }

                        catch
                        {
                            LogList.Insert(0, "⚠️ CSV write error !");
                        }
                    }
                });
            }
            catch { }
        }

        public void StartRecording()
        {
            isRecording = true;                                                                     
            LogList.Insert(0, $"🔴 Veri kaydı başlatıldı: {DateTime.Now:HH:mm:ss}");                
        }

        public void StopRecording() 
        { 
            isRecording = false;
            LogList.Insert(0, $"⏹ Veri kaydı durduruldu: {DateTime.Now:HH:mm:ss}");
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }

}
