using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MyFisrtApp.SensorData
{
    public class MPU6050SensorData
    {
        public float temperature { get; set; }
        public float ax { get; set; }
        public float ay { get; set; }
        public float az { get; set; }
        public float gx { get; set; }
        public float gy { get; set; }
        public float gz { get; set; }
        public float roll { get; set; }
        public float pitch { get; set; }


    }
}
