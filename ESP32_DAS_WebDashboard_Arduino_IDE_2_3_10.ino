#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

// ESP32 DEVKIT V1 / Arduino IDE 2.3.10
// Stand-alone Wi-Fi dashboard: connect to ENGINE_DAS, open http://192.168.4.1
// Pin mapping follows the supplied DAS document where possible.

const char* AP_SSID = "ENGINE_DAS";
const char* AP_PASSWORD = "DAS123456";
WebServer server(80);

#define HX711_DOUT 4
#define HX711_SCK 5
#define MAX_CS 15
#define MAX_SCK 18
#define MAX_MISO 19
#define HALL_PIN 14
#define FLOW_PIN 27
#define SDA_PIN 21
#define SCL_PIN 22
#define ADS_ADDR 0x48
#define MPU_ADDR 0x68

// Calibration/settings. Change after physical calibration.
float LOAD_CAL_FACTOR = 1.0f;
uint8_t MAGNETS_PER_REV = 1;
float FLOW_PULSES_PER_LITRE = 1380.0f; // starting assumption for YF-S402C; verify experimentally

volatile uint32_t hallPulses = 0, flowPulses = 0;
void IRAM_ATTR hallISR(){ hallPulses++; }
void IRAM_ATTR flowISR(){ flowPulses++; }

class HX711Simple {
  uint8_t d,s;
public:
  HX711Simple(uint8_t dout,uint8_t sck):d(dout),s(sck){}
  void begin(){pinMode(d,INPUT);pinMode(s,OUTPUT);digitalWrite(s,LOW);}
  bool ready(){return digitalRead(d)==LOW;}
  int32_t read(){
    unsigned long t=millis(); while(!ready()){if(millis()-t>1000)return 0;delay(1);}
    uint32_t v=0; noInterrupts();
    for(int i=0;i<24;i++){digitalWrite(s,HIGH);delayMicroseconds(1);v=(v<<1)|digitalRead(d);digitalWrite(s,LOW);delayMicroseconds(1);}
    digitalWrite(s,HIGH);delayMicroseconds(1);digitalWrite(s,LOW);delayMicroseconds(1); interrupts();
    if(v&0x800000UL)v|=0xFF000000UL; return (int32_t)v;
  }
  int32_t average(uint8_t n){int64_t x=0;for(uint8_t i=0;i<n;i++)x+=read();return (int32_t)(x/n);}
};
HX711Simple hx(HX711_DOUT,HX711_SCK);
long tare=0; bool tareOK=false;

struct Data {
 float temp=NAN,rpm=0,flow=0,load=NAN,adcV=0,ax=NAN,ay=NAN,az=NAN,gx=NAN,gy=NAN,gz=NAN;
 int32_t loadRaw=0; int16_t adcRaw=0; uint8_t fault=0;
 bool thermo=false,hxok=false,ads=false,mpu=false;
} d;

bool maxRead(float &tc,uint8_t &fault){
 uint32_t x=0; SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0)); digitalWrite(MAX_CS,LOW);
 x|=(uint32_t)SPI.transfer(0)<<24;x|=(uint32_t)SPI.transfer(0)<<16;x|=(uint32_t)SPI.transfer(0)<<8;x|=SPI.transfer(0);
 digitalWrite(MAX_CS,HIGH);SPI.endTransaction(); fault=x&7; if(x&0x10000UL){tc=NAN;return false;}
 int32_t r=(int32_t)(x>>18);if(r&0x2000)r|=0xFFFFC000;tc=r*0.25f;return true;
}
void adsWrite(uint8_t r,uint16_t v){Wire.beginTransmission(ADS_ADDR);Wire.write(r);Wire.write(v>>8);Wire.write(v);Wire.endTransmission();}
uint16_t adsRead(uint8_t r){Wire.beginTransmission(ADS_ADDR);Wire.write(r);if(Wire.endTransmission(false)!=0)return 0;Wire.requestFrom(ADS_ADDR,(uint8_t)2);if(Wire.available()<2)return 0;return ((uint16_t)Wire.read()<<8)|Wire.read();}
bool adsOK(){Wire.beginTransmission(ADS_ADDR);return Wire.endTransmission()==0;}
int16_t adsA0(){adsWrite(1,0xC383);delay(10);return (int16_t)adsRead(0);}

uint8_t mpuByte(uint8_t r){Wire.beginTransmission(MPU_ADDR);Wire.write(r);if(Wire.endTransmission(false)!=0)return 0xFF;Wire.requestFrom(MPU_ADDR,(uint8_t)1);return Wire.available()?Wire.read():0xFF;}
void mpuWrite(uint8_t r,uint8_t v){Wire.beginTransmission(MPU_ADDR);Wire.write(r);Wire.write(v);Wire.endTransmission();}
bool mpuBytes(uint8_t r,uint8_t*b,uint8_t n){Wire.beginTransmission(MPU_ADDR);Wire.write(r);if(Wire.endTransmission(false)!=0)return false;if(Wire.requestFrom(MPU_ADDR,n)!=n)return false;for(uint8_t i=0;i<n;i++)b[i]=Wire.read();return true;}
bool mpuInit(){uint8_t w=mpuByte(0x75);if(w!=0x68&&w!=0x69)return false;mpuWrite(0x6B,0);mpuWrite(0x1C,0);mpuWrite(0x1B,0);delay(100);return true;}
bool mpuRead(){uint8_t b[14];if(!mpuBytes(0x3B,b,14))return false;int16_t ax=(b[0]<<8)|b[1],ay=(b[2]<<8)|b[3],az=(b[4]<<8)|b[5],gx=(b[8]<<8)|b[9],gy=(b[10]<<8)|b[11],gz=(b[12]<<8)|b[13];d.ax=ax/16384.0f;d.ay=ay/16384.0f;d.az=az/16384.0f;d.gx=gx/131.0f;d.gy=gy/131.0f;d.gz=gz/131.0f;return true;}

void updateSensors(){
 static unsigned long last=millis(); unsigned long now=millis(); float sec=(now-last)/1000.0f; if(sec<=0)sec=0.5f; last=now;
 uint32_t hp,fp;noInterrupts();hp=hallPulses;hallPulses=0;fp=flowPulses;flowPulses=0;interrupts();
 d.rpm=(hp/sec)*60.0f/MAGNETS_PER_REV; d.flow=(fp/sec)*60.0f/FLOW_PULSES_PER_LITRE;
 d.thermo=maxRead(d.temp,d.fault);d.hxok=hx.ready();if(d.hxok){d.loadRaw=hx.average(2);if(tareOK)d.load=(d.loadRaw-tare)/LOAD_CAL_FACTOR;}
 d.ads=adsOK();if(d.ads){d.adcRaw=adsA0();d.adcV=d.adcRaw*0.000125f;}d.mpu=mpuRead();
}

const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>ENGINE DAS</title><style>
*{box-sizing:border-box}body{margin:0;background:#111827;color:#f3f4f6;font-family:Arial,sans-serif}.head{text-align:center;padding:20px;background:#1f2937;border-bottom:1px solid #374151}.head h1{margin:0;font-size:25px}.head p{color:#9ca3af;margin:7px}.online{display:inline-block;padding:6px 12px;border-radius:20px;background:#065f46;color:#d1fae5;font-size:13px}.wrap{max-width:1200px;margin:auto;padding:20px}.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}.card,.info,.small{background:#1f2937;border:1px solid #374151;border-radius:12px;padding:17px}.card h3{margin:0 0 10px;color:#9ca3af;font-size:13px;text-transform:uppercase}.value{font-size:31px;font-weight:bold}.unit,.sub{color:#9ca3af;font-size:13px}.sub{margin-top:9px}.sec{margin-top:20px}.title{margin:0 0 10px;color:#d1d5db}.mpu{display:grid;grid-template-columns:repeat(6,1fr);gap:12px}.small{text-align:center}.small b{display:block;margin-top:7px;font-size:21px}.small span{font-size:12px;color:#9ca3af}.info{margin-top:20px;line-height:1.8}.foot{text-align:center;color:#6b7280;padding:20px;font-size:12px}@media(max-width:850px){.grid{grid-template-columns:repeat(2,1fr)}.mpu{grid-template-columns:repeat(3,1fr)}}@media(max-width:520px){.grid{grid-template-columns:1fr}.mpu{grid-template-columns:repeat(2,1fr)}.head h1{font-size:20px}}
</style></head><body><div class="head"><h1>ENGINE DATA ACQUISITION SYSTEM</h1><p>ESP32 DEVKIT V1 &bull; Real-Time Monitoring</p><span class="online" id="st">● SYSTEM ONLINE</span></div><div class="wrap">
<div class="grid"><div class="card"><h3>Temperature</h3><div class="value" id="t">--</div><div class="unit">°C</div><div class="sub" id="ts">MAX31855</div></div><div class="card"><h3>Engine RPM</h3><div class="value" id="r">--</div><div class="unit">RPM</div><div class="sub">Hall Effect Sensor</div></div><div class="card"><h3>Load / Thrust</h3><div class="value" id="l">--</div><div class="unit">kg</div><div class="sub" id="ls">Calibration required</div></div><div class="card"><h3>Fuel Flow</h3><div class="value" id="f">--</div><div class="unit">L/min</div><div class="sub">YF-S402C</div></div></div>
<div class="sec"><h3 class="title">10K POTENTIOMETER / ADS1115</h3><div class="grid"><div class="card"><h3>ADC A0</h3><div class="value" id="a">--</div><div class="unit">counts</div></div><div class="card"><h3>Voltage</h3><div class="value" id="v">--</div><div class="unit">V</div></div></div></div>
<div class="sec"><h3 class="title">MPU6050</h3><div class="mpu"><div class="small"><span>ACCEL X</span><b id="ax">--</b><span>g</span></div><div class="small"><span>ACCEL Y</span><b id="ay">--</b><span>g</span></div><div class="small"><span>ACCEL Z</span><b id="az">--</b><span>g</span></div><div class="small"><span>GYRO X</span><b id="gx">--</b><span>°/s</span></div><div class="small"><span>GYRO Y</span><b id="gy">--</b><span>°/s</span></div><div class="small"><span>GYRO Z</span><b id="gz">--</b><span>°/s</span></div></div></div>
<div class="info"><b>System Information</b><br>Wi-Fi IP: 192.168.4.1<br>Uptime: <span id="up">--</span> s<br>Last update: <span id="tm">--</span></div></div><div class="foot">ENGINE DAS &bull; ESP32 Web Dashboard</div>
<script>const $=x=>document.getElementById(x),val=(id,x,n)=>$(id).textContent=x==null?'--':Number(x).toFixed(n);async function refresh(){try{let q=await fetch('/data',{cache:'no-store'}),d=await q.json();val('t',d.t,d.thermo?2:null);$('ts').textContent=d.thermo?'MAX31855 OK':'THERMOCOUPLE FAULT';val('r',d.rpm,0);if(d.cal){val('l',d.load,2);$('ls').textContent='HX711 • calibrated'}else{$('l').textContent='--';$('ls').textContent='HX711 • calibration required'}val('f',d.f,2);$('a').textContent=d.a;val('v',d.v,3);['ax','ay','az'].forEach(k=>val(k,d[k],3));['gx','gy','gz'].forEach(k=>val(k,d[k],2));$('up').textContent=d.up;$('tm').textContent=new Date().toLocaleTimeString();$('st').textContent='● SYSTEM ONLINE'}catch(e){$('st').textContent='● CONNECTION LOST'}}refresh();setInterval(refresh,500);</script></body></html>
)HTML";

void root(){server.send_P(200,"text/html",PAGE);}
void api(){String j="{";j+="\"thermo\":"+(d.thermo?String("true"):String("false"));j+=",\"t\":"+(isnan(d.temp)?String("null"):String(d.temp,2));j+=",\"r\":"+String(d.rpm,1);j+=",\"f\":"+String(d.flow,2);j+=",\"hx\":"+(d.hxok?String("true"):String("false"));j+=",\"raw\":"+String(d.loadRaw);bool cal=tareOK&&LOAD_CAL_FACTOR!=1.0f;j+=",\"cal\":"+(cal?String("true"):String("false"));j+=",\"load\":"+(isnan(d.load)?String("null"):String(d.load,2));j+=",\"ads\":"+(d.ads?String("true"):String("false"));j+=",\"a\":"+String(d.adcRaw);j+=",\"v\":"+String(d.adcV,4);j+=",\"mpu\":"+(d.mpu?String("true"):String("false"));j+=",\"ax\":"+(isnan(d.ax)?String("null"):String(d.ax,3));j+=",\"ay\":"+(isnan(d.ay)?String("null"):String(d.ay,3));j+=",\"az\":"+(isnan(d.az)?String("null"):String(d.az,3));j+=",\"gx\":"+(isnan(d.gx)?String("null"):String(d.gx,2));j+=",\"gy\":"+(isnan(d.gy)?String("null"):String(d.gy,2));j+=",\"gz\":"+(isnan(d.gz)?String("null"):String(d.gz,2));j+=",\"up\":"+String(millis()/1000UL)+"}";server.send(200,"application/json",j);}

void setup(){Serial.begin(115200);delay(800);Wire.begin(SDA_PIN,SCL_PIN);Wire.setClock(100000);pinMode(MAX_CS,OUTPUT);digitalWrite(MAX_CS,HIGH);SPI.begin(MAX_SCK,MAX_MISO,23,MAX_CS);hx.begin();pinMode(HALL_PIN,INPUT);pinMode(FLOW_PIN,INPUT);attachInterrupt(digitalPinToInterrupt(HALL_PIN),hallISR,RISING);attachInterrupt(digitalPinToInterrupt(FLOW_PIN),flowISR,RISING);Serial.print("MPU6050: ");Serial.println(mpuInit()?"OK":"NOT DETECTED");Serial.print("ADS1115: ");Serial.println(adsOK()?"OK":"NOT DETECTED");Serial.println("Keep load cell unloaded. Taring in 3 seconds...");delay(3000);if(hx.ready()){tare=hx.average(10);tareOK=true;Serial.print("Tare: ");Serial.println(tare);}else Serial.println("HX711 not ready");WiFi.mode(WIFI_AP);WiFi.softAP(AP_SSID,AP_PASSWORD);Serial.println("\nWi-Fi AP started");Serial.print("SSID: ");Serial.println(AP_SSID);Serial.print("Password: ");Serial.println(AP_PASSWORD);Serial.print("Open: http://");Serial.println(WiFi.softAPIP());server.on("/",HTTP_GET,root);server.on("/data",HTTP_GET,api);server.onNotFound([](){server.send(404,"text/plain","Not found");});server.begin();updateSensors();}

void loop(){server.handleClient();static unsigned long last=0;if(millis()-last>=500){last=millis();updateSensors();Serial.print("T=");if(d.thermo)Serial.print(d.temp,2);else Serial.print("FAULT");Serial.print(" C | RPM=");Serial.print(d.rpm,0);Serial.print(" | Flow=");Serial.print(d.flow,2);Serial.print(" L/min | LoadRaw=");Serial.print(d.loadRaw);Serial.print(" | ADC=");Serial.println(d.adcRaw);}}
