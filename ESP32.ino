#include <WiFi.h>
#include <WebServer.h>


// ================== CONFIG WIFI ==================
const char* ssid = "Centro Investigacion FW";
const char* password = "FacultadIng2025$";


IPAddress local_IP(192, 168, 1, 77);     // IP fija
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   // Opcional
IPAddress secondaryDNS(8, 8, 4, 4); // Opcional


WebServer server(5006); // Puerto cambiado a 5006


// ================== Pines motor ==================
#define PWM_PIN 5
#define DIR_PIN 23
const int pwmChannel = 0;
const int freq = 20000;
const int resolution = 8;


// ================== Encoder ==================
#define EncoderPinA 33
#define EncoderPinB 32
volatile long Encodervalue = 0;
volatile double grados = 0;
double PPR = 1480;
volatile long lastEncoded = 0;


// ================== Variables PID ==================
volatile double Kp = 0.0, Ki = 0.0, Kd = 0.0;
volatile double Setpoint = 0;
volatile bool pidActive = false;
volatile double integral = 0;
volatile double lastError = 0;
volatile double integralLimit = 100.0;
volatile double derivFilterAlpha = 0.7;
volatile double lastDerivative = 0.0;


// ================== Límites PID ==================
const double KP_MIN = 0.0;
const double KP_MAX = 15.0;
const double KI_MIN = 0.0;
const double KI_MAX = 5.0;
const double KD_MIN = 0.0;
const double KD_MAX = 2.0;


// ================== Timeout PID ==================
unsigned long pidStartTime = 0;
const unsigned long pidTimeout = 5000; // 5 segundos


// ================== PWM ==================
volatile int dutyCycle = 0;
volatile int direccion = 1;


// ================== Timer ==================
hw_timer_t *Timer0_Cfg = NULL;


// ================== ISR Encoder ==================
void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(EncoderPinA);
  int LSB = digitalRead(EncoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;


  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) Encodervalue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) Encodervalue--;


  lastEncoded = encoded;
  grados = (Encodervalue * 360.0 / PPR);
}


// ================== ISR Timer ==================
void IRAM_ATTR Timer0_ISR() {
  if (!pidActive) return;


  double feedback = grados;
  double error = Setpoint - feedback;
  double dt = 0.01;


  integral += error * dt;
  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;


  double rawDeriv = (error - lastError) / dt;
  double deriv = derivFilterAlpha * lastDerivative + (1.0 - derivFilterAlpha) * rawDeriv;
  lastDerivative = deriv;


  double output = Kp * error + Ki * integral + Kd * deriv;
  lastError = error;


  if (output >= 0) {
    direccion = 1;
  } else {
    direccion = 0;
    output = -output;
  }


  dutyCycle = (int)constrain(output, 0, 255);
  digitalWrite(DIR_PIN, direccion);
  ledcWrite(pwmChannel, dutyCycle);
}


// ================== Página Web ==================
String htmlPage() {
  String page = "<!DOCTYPE html><html><head><title>PID Motor DC</title>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += "<style>";
  page += "body{font-family:Arial;margin:0;height:100vh;overflow:hidden;display:flex;background:#2b2b2b;color:white;}";
  page += ".sidebar{width:450px;background:#1f1f1f;padding:15px;text-align:center;box-shadow:2px 0 5px rgba(0,0,0,0.5);overflow-y:auto;}";
  page += ".img-row{display:flex;justify-content:center;align-items:center;gap:10px;margin-bottom:10px;}";
  page += ".img-row img{max-width:180px;}";
  page += ".content{flex:1;padding:15px;text-align:center;position:relative;}";
  page += ".video-box{margin:5px auto;}";
  page += ".top-bar{display:flex;justify-content:center;align-items:center;position:relative;margin-bottom:10px;}";
  page += ".top-bar h2{margin:0;}";
  page += ".logo-uni{position:absolute;right:10px;top:0;max-width:120px;}";
  page += "input{margin:2px;width:80px;}";
  page += "button{margin:3px;padding:6px 12px;}";
  page += "#pidBtn.on{background:green;color:white;}";
  page += "#pidBtn.off{background:gray;color:white;}";
  page += "#stopBtn{background:red;color:white;}";
  page += "table{margin:10px auto;border-collapse:collapse;font-size:14px;background:#333;}";
  page += "td{padding:4px 6px;border:1px solid #555;}";
  page += "</style></head><body>";


  page += "<script>";
  page += "function actualizarValores(e){e.preventDefault();";
  page += "let kp=parseFloat(document.getElementById('kp').value);";
  page += "let ki=parseFloat(document.getElementById('ki').value);";
  page += "let kd=parseFloat(document.getElementById('kd').value);";
  page += "let sp=parseFloat(document.getElementById('sp').value);";
  page += "if(kp<"+String(KP_MIN)+") kp="+String(KP_MIN)+";";
  page += "if(kp>"+String(KP_MAX)+") kp="+String(KP_MAX)+";";
  page += "if(ki<"+String(KI_MIN)+") ki="+String(KI_MIN)+";";
  page += "if(ki>"+String(KI_MAX)+") ki="+String(KI_MAX)+";";
  page += "if(kd<"+String(KD_MIN)+") kd="+String(KD_MIN)+";";
  page += "if(kd>"+String(KD_MAX)+") kd="+String(KD_MAX)+";";
  page += "if(sp>360) sp=360;if(sp<-360) sp=-360;";
  page += "fetch(`/set?kp=${kp}&ki=${ki}&kd=${kd}&sp=${sp}`).then(()=>location.reload());}";
  page += "function encenderPID(){let btn=document.getElementById('pidBtn');";
  page += "btn.innerText='PID Encendido';btn.className='on';";
  page += "fetch('/on');";
  page += "setTimeout(()=>{btn.innerText='Prueba de control';btn.className='off';fetch('/off');},5000);}";
  page += "function apagarPID(){let btn=document.getElementById('pidBtn');";
  page += "btn.innerText='Prueba de control';btn.className='off';";
  page += "fetch('/off').then(()=>location.reload());}";
  page += "setInterval(()=>{";
  page += "fetch('/estado').then(r=>r.json()).then(data=>{";
  page += "document.getElementById('posicion').innerHTML = data.posicion + '&deg;';";
  page += "document.getElementById('setpoint').innerHTML = data.setpoint;";
  page += "document.getElementById('pwm').innerHTML = data.pwm;";
  page += "});";
  page += "},500);";
  page += "</script>";


  page += "<div class='sidebar'>";
  page += "<h3>Control y Sentido</h3>";
  page += "<div class='img-row'>";
  page += "<div><img src='https://imgur.com/6b9TZpc.png' alt='PID Paralelo'><br><small>Control PID Paralelo</small></div>";
  page += "<div><img src='https://imgur.com/vtZlUwx.png' alt='Sentido giro' style='max-width:140px;'><br><small>Sentido de giro</small></div>";
  page += "</div>";


  page += "<h3>Estado</h3>";
  page += "<table>";
  page += "<tr><td><b>Posicion</b></td><td id='posicion'>" + String(grados,1) + "&deg;</td></tr>";
  page += "<tr><td><b>Referencia</b></td><td id='setpoint'>" + String(Setpoint,1) + "</td></tr>";
  page += "<tr><td><b>PWM</b></td><td id='pwm'>" + String(dutyCycle) + "</td></tr>";
  page += "</table>";


  page += "<h3>Ayuda</h3>";
  page += "<p>Ajusta las variables Kp, Ki, Kd y la referencia.<br>Presione 'Enviar' y luego 'Prueba de control'.<br><br>Luego de activado el PID se apaga en 5s.<br><br>Una referencia positiva mueve el motor en sentido Horario mientras que una referencia negativa en sentido Anti-Horario.<br><br>El rango de grados aceptable para la referencia es de -360 a 360.</p>";


  page += "</div>";


  page += "<div class='content'>";
  page += "<div class='top-bar'>";
  page += "<h2>Control PID Motor DC</h2>";
  page += "<img class='logo-uni' src='https://imgur.com/OueoMrp.png' alt='Logo Universidad'>";
  page += "</div>";
  page += "<div class='video-box'>";
  page += "<iframe src='http//:192.168.1.138:8080/video' width='700' height='500' frameborder='0'></iframe>";
  page += "</div>";
  page += "<form onsubmit='actualizarValores(event)'>";
  page += "Proporcional Kp: <input id='kp' type='number' step='0.01' min='"+String(KP_MIN)+"' max='"+String(KP_MAX)+"' value='" + String(Kp) + "'> ";
  page += "Integral Ki: <input id='ki' type='number' step='0.01' min='"+String(KI_MIN)+"' max='"+String(KI_MAX)+"' value='" + String(Ki) + "'> ";
  page += "Derivada Kd: <input id='kd' type='number' step='0.01' min='"+String(KD_MIN)+"' max='"+String(KD_MAX)+"' value='" + String(Kd) + "'> ";
  page += "Referencia: <input id='sp' type='number' step='1' min='-360' max='360' value='" + String(Setpoint) + "'> ";
  page += "<input type='submit' value='Enviar'></form><br>";
  page += "<button id='pidBtn' class='off' onclick='encenderPID()'>Prueba de control</button> ";
  page += "<button id='stopBtn' onclick='apagarPID()'>Apagado de emergencia</button><br><br>";
  page += "</div></body></html>";
  return page;
}


// ================== Rutas ==================
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}


void handleSet() {
  if (server.hasArg("kp")) Kp = constrain(server.arg("kp").toFloat(), KP_MIN, KP_MAX);
  if (server.hasArg("ki")) Ki = constrain(server.arg("ki").toFloat(), KI_MIN, KI_MAX);
  if (server.hasArg("kd")) Kd = constrain(server.arg("kd").toFloat(), KD_MIN, KD_MAX);
  if (server.hasArg("sp")) {
    Setpoint = server.arg("sp").toFloat();
    Setpoint = constrain(Setpoint, -360, 360);
  }
  server.send(200, "text/html", htmlPage());
}


void handleOn() {
  pidActive = true;
  pidStartTime = millis();
  integral = 0;
  lastError = 0;
  lastDerivative = 0;
  server.send(200, "text/plain", "PID encendido");
}


void handleOff() {
  pidActive = false;
  dutyCycle = 0;
  ledcWrite(pwmChannel, dutyCycle);
  server.send(200, "text/html", htmlPage());
}


void handleEstado() {
  String json = "{";
  json += "\"posicion\":" + String(grados, 1) + ",";
  json += "\"setpoint\":" + String(Setpoint, 1) + ",";
  json += "\"pwm\":" + String(dutyCycle);
  json += "}";
  server.send(200, "application/json", json);
}


// ================== Setup ==================
void setup() {
  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;
  Setpoint = 0;
  pidActive = false;
  integral = 0;
  lastError = 0;
  dutyCycle = 0;


  Serial.begin(115200);


  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Error al configurar IP fija");
  }


  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.print("IP asignada: ");
  Serial.println(WiFi.localIP());


  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/on", handleOn);
  server.on("/off", handleOff);
  server.on("/estado", handleEstado);
  server.begin();


  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 10000, true);
  timerAlarmEnable(Timer0_Cfg);


  pinMode(DIR_PIN, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWM_PIN, pwmChannel);


  pinMode(EncoderPinA, INPUT_PULLUP);
  pinMode(EncoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderPinB), updateEncoder, CHANGE);
}


// ================== Loop ==================
void loop() {
  server.handleClient();
  if (pidActive && (millis() - pidStartTime > pidTimeout)) {
    pidActive = false;
    dutyCycle = 0;
    ledcWrite(pwmChannel, dutyCycle);
  }
}
