# STM32 DC Motor PID Kontrol (Encoder) + Qt (QML) GUI

**STM32F407** üzerinde **quadrature encoder** geri beslemesiyle çalışan **kapalı çevrim PID** tabanlı DC motor kontrolü ve bu sistemi gerçek zamanlı izlemek, günlüğe almak ve PID ayarlarını değiştirmek için modern bir **Qt/QML** panosu.

> Bu README, projeyi ve son arayüzü özetler; kurulum veya nasıl kullanılır bölümleri özellikle yer almaz.

---

## 🔎 Genel Bakış

- **Firmware** (C/HAL): Encoder geri beslemesi, PWM sürüşü ve bir PID döngüsü. UART üzerinden deterministik JSON telemetrisi üretir; setpoint ve kazançlar komutlarla yönetilir.
- **GUI** (Qt 6/QML + C++): Gösterge (RPM, Speed, Pulse), encoder açı ve ivme kartları, PID sürgüleri, bağlantı paneli (COM/baud, data/stop bits, parity, flow control), keep-alive ve arama/dışa aktarma destekli kapsamlı bir log sistemi.

---

## 🧠 Temel Özellikler

### Gömülü (STM32)
- Kapalı çevrim **PID kontrol** (ayarlanabilir Kp/Ki/Kd)
- **Quadrature encoder** ile RPM / hız / açı hesaplama
- **JSON-over-UART** telemetri (satır sonu ile ayrılmış)
- **İnsan okunur komutlar** ile tuning ve setpoint yönetimi

### Masaüstü (Qt/QML)
- **Canlı göstergeler**: RPM, Pulse, Speed  
- **Açı / İvme** kartları (dinamik renk geri bildirimi)  
- **PID Parametreleri** bölümü (Submit / Reset)  
- **Connection** paneli: port/baud, data/stop bits, parity, flow control  
- **Keep-Alive** penceresi: payload, CR/LF anahtarları, beklenen cevap  
- **Loglar**: event & periodic akışı, **geniş görünüm**, **arama**, **kaydet**, **CSV/JSON dışa aktarma**  
- **Profiller**: seri bağlantı profili kaydet/yükle

---

## 🧾 Telemetri ve Komutlar (özet)

**Telemetri (MCU → PC)** — her satır bir JSON nesnesidir:
```json
{"rpm":177,"speed":87.5,"pwm":999,"angle":280,"acc":0.0,"kp":1.0,"ki":0.02,"kd":0.0,"err":1500}
```

**Komutlar (PC → MCU)** — insan okunur metin:
```
PID, Kp:1.20 , Ki:0.02 , Kd:0.00
Speed: 1500
Rpm: 800
Pulse: 1200
TXT:OK
```

**Log Dışa Aktarım (JSON örneği)** — periodic ve event girdileri:
```json
[
  { "type":"periodic","speed":75,"rpm":112,"pwm":999,"msg":"...Speed=75.00, RPM=112, PWM=999, Angle=160.00, Acc=312.50, Error=1425.00" },
  { "type":"event","msg":"...Pid parametreleri değiştirildi. Yeni Kp: 1.00, Yeni Ki: 0.02, Yeni Kd: 0.00" },
  { "type":"event","msg":"...Hedef hız değiştirildi. Yeni Hız: 1500.00" }
]
```

---

## 🎬 Demo Video

**Demoyu izle:** [docs/demo.mp4](docs/demo.mp4)

[![Demoya gitmek için tıkla](docs/1.png)](docs/demo.mp4)

---

## 📷 Ekran Görüntüleri

![Ana pano: göstergeler, açı/ivme kartları, PID bölümü, bağlantı ve loglar](docs/1.png)

![Logların geniş görünümü ve log dışa aktarma düğmeleri (CSV/JSON)](docs/2.png)

![Seri profil kaydetme penceresi](docs/3.png)

![Keep-Alive penceresi](docs/4.png)

![Porta Mesaj penceresi (bağlanınca gönder, CR/LF anahtarları)](docs/5.png)

![Log günlük kayıt (JSON)](docs/6.png)

---

## 📁 Proje Dosyaları

```text
├── DC_MOTOR_CONTROL_WITH_PID/
│   ├── Core
│   │   ├── Inc                   # pid_control.h, JGB37-520_Encoder.h, QtDataExchange.h, vb.
│   │   └── Src                   # main.c, pid_control.c, JGB37-520_Encoder.c, QtDataExchange.c, vb.
│   ├── Drivers                   # CMSIS + HAL
│   └── *.ioc                     # STM32CubeMX proje ayarı
├── DC_Motor_Control_GUI/
│   ├── backend                   # SerialManager, SpeedController, PidManager, RpmReceiver, vb.
│   ├── components                # QML bileşenleri (gauge, panel, vs.)
│   ├── uihelpers                 # QML yardımcıları / utils
│   ├── Main.qml
│   ├── main.cpp
│   └── CMakeLists.txt
├── docs/
│   ├── 1.png                     # Ana pano
│   ├── 2.png                     # Logların geniş görünümü
│   ├── 3.png                     # Profil kaydetme
│   ├── 4.png                     # Keep-Alive
│   ├── 5.png                     # Porta mesaj
│   ├── 6.png                     # Log JSON
│   └── demo.mp4                  # Kısa tanıtım videosu
```

---

## 🧩 Üst Düzey Mimari

```
┌───────────────┐     JSON (newline)      ┌────────────────────┐
│  STM32 (HAL)  │ ───────────────────────▶ │  Qt/C++ Backend    │
│  PID + Encoder│ ◀─────────────────────── │  QSerialPort I/O   │
└───────────────┘  Commands (text)        └─────────┬──────────┘
                                                    │
                                             ┌──────▼──────┐
                                             │   QML UI    │
                                             │ Gauge/Log   │
                                             └─────────────┘
```

---

## 📜 Yazar
 
- **Yazar**: *Hüseyin Yanar*
