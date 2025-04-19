#define BLYNK_TEMPLATE_ID "TMPL6GlVCh-a4"
#define BLYNK_TEMPLATE_NAME "HutBui"
#define BLYNK_AUTH_TOKEN "y3iHgwTK4OuXGrpwcWmjIPq8_j2F-rBV"

#include <ESP32Servo.h>
#include <DHT.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Thông tin WiFi và Blynk
char auth[] = BLYNK_AUTH_TOKEN;    // Sử dụng token đã định nghĩa ở trên
char ssid[] = "POCO 2G";    // Thay đổi thành tên WiFi của bạn
char pass[] = "tu0dentam"; // Thay đổi thành mật khẩu WiFi của bạn

// PINS & CONSTANTS FOR OBSTACLE AVOIDANCE ROBOT
Servo myservo;  // Tạo đối tượng servo

// Chân kết nối cảm biến siêu âm
const int trig = 12;  
const int echo = 13;  

// Chân kết nối động cơ
int tien1 = 5;  // Tiến bánh xe bên trái      
int tien2 = 17;  // Tiến bánh xe bên phải        
int lui1 = 16;   // Lùi bánh xe bên trái        
int lui2 = 4;   // Lùi bánh xe bên phải        

// Chân kết nối servo
int dongcoservo = 14;    

// MOSFET control pin (cho máy hút bụi)
#define control 15

// Biến toàn cục cho robot
int gioihan = 25; // Khoảng cách nhận biết vật cản (cm)
unsigned long thoigian; 
int khoangcach;          
int khoangcachtrai, khoangcachphai;

// PINS & CONSTANTS FOR ENVIRONMENTAL SENSORS
#define DHTPIN 18     // Chân dữ liệu của DHT22 kết nối với GPIO18 của ESP32
#define DHTTYPE DHT22 // Loại cảm biến DHT
#define MQ2_PIN_A0 34 // Chân analog output (A0) của MQ2 kết nối với GPIO34 của ESP32
#define MQ2_PIN_D0 21 // Chân digital output (D0) của MQ2 kết nối với GPIO21 của ESP32

// Khởi tạo đối tượng DHT
DHT dht(DHTPIN, DHTTYPE);

// Biến tính thời gian cho các hệ thống
unsigned long previousMillisRobot = 0;
unsigned long previousMillisSensors = 0;
unsigned long previousMillisBlynk = 0;
const long intervalRobot = 200;    // Thời gian giữa các lần đọc và xử lý robot (ms)
const long intervalSensors = 2000; // Thời gian giữa các lần đọc cảm biến môi trường (ms)
const long intervalBlynk = 1000;   // Thời gian giữa các lần gửi dữ liệu lên Blynk (ms)

// Ngưỡng cảnh báo
const float TEMP_THRESHOLD = 45.0;  // Ngưỡng nhiệt độ cảnh báo (°C)
const int GAS_THRESHOLD = 550;      // Ngưỡng khí gas cảnh báo
const float HUMIDITY_THRESHOLD = 45.0; // Ngưỡng độ ẩm cảnh báo (%)

// Biến lưu trữ trạng thái
bool systemState = false;           // Trạng thái hệ thống (bật/tắt từ V1)
bool alarmState = false;            // Trạng thái cảnh báo

// Khai báo các hàm của robot
void dokhoangcach();
void dithang();
void disangtrai();
void disangphai();
void dilui();
void resetdongco();
void quaycbsangphai();
void quaycbsangtrai();
void resetservo();

// Khai báo các hàm cho cảm biến môi trường
void readEnvironmentalSensors();
void sendDataToBlynk();
void checkThresholds(float temperature, int gasValue, float humidity);

// Hàm callback khi nhận dữ liệu từ Blynk
BLYNK_WRITE(V1) {
  systemState = param.asInt();
  if (systemState) {
    digitalWrite(control, HIGH);  // Bật máy hút bụi
    Serial.println("[BLYNK] Hệ thống BẬT (máy hút bụi và robot tránh vật cản)");
  } else {
    digitalWrite(control, LOW);   // Tắt máy hút bụi
    resetdongco();                // Dừng động cơ robot
    Serial.println("[BLYNK] Hệ thống TẮT");
  }
}

void setup() {
  // Khởi tạo Serial Monitor
  Serial.begin(115200);
  Serial.println("ESP32 - Hệ thống kết hợp Robot tránh vật cản, Cảm biến môi trường và Blynk");
  Serial.println("----------------------------------------------------------");

  // Kết nối với Blynk
  Blynk.begin(auth, ssid, pass);
  Serial.println("Đang kết nối với Blynk...");
  
  // Gửi trạng thái kết nối thành công lên V2 (LED báo kết nối)
  Blynk.virtualWrite(V2, 1);  // Bật LED V2 để báo đã kết nối
  
  // KHỞI TẠO CHO ROBOT TRÁNH VẬT CẢN
  // Khởi tạo servo
  myservo.attach(dongcoservo); 
  
  // Cấu hình chân cảm biến siêu âm
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);   
  
  // Cấu hình chân động cơ
  pinMode(tien1, OUTPUT);
  pinMode(tien2, OUTPUT);
  pinMode(lui1, OUTPUT);
  pinMode(lui2, OUTPUT);
  
  // Đảm bảo động cơ dừng khi khởi động
  resetdongco();
  
  // Đặt servo ở vị trí trung tâm
  myservo.write(90);
  delay(500);
  
  Serial.println("Khởi tạo robot tránh vật cản hoàn tất");

  // KHỞI TẠO CHO CẢM BIẾN MÔI TRƯỜNG
  dht.begin();
  pinMode(MQ2_PIN_A0, INPUT); // Đảm bảo chân A0 của MQ2 được cấu hình là input
  pinMode(MQ2_PIN_D0, INPUT); // Đảm bảo chân D0 của MQ2 được cấu hình là input
  Serial.println("Khởi tạo cảm biến DHT22 và MQ2 hoàn tất");
  
  // KHỞI TẠO CHO MOSFET (máy hút bụi)
  pinMode(control, OUTPUT);    // Cấu hình chân điều khiển MOSFET là output
  digitalWrite(control, LOW);  // Tắt máy hút bụi ban đầu
  Serial.println("Khởi tạo điều khiển MOSFET cho máy hút bụi hoàn tất");
  
  // Gửi trạng thái ban đầu
  Blynk.virtualWrite(V1, 0);  // Đảm bảo V1 hiển thị trạng thái tắt
  Blynk.virtualWrite(V4, 0);  // Đảm bảo V4 (LED cảnh báo) tắt ban đầu
}

void loop() {
  // Chạy Blynk
  Blynk.run();
  
  unsigned long currentMillis = millis();

  // XỬ LÝ ROBOT TRÁNH VẬT CẢN - CHỈ KHI SYSTEMSTATE = TRUE
  if (systemState && currentMillis - previousMillisRobot >= intervalRobot) {
    previousMillisRobot = currentMillis;
    
    // Đo khoảng cách
    dokhoangcach();
    Serial.print("[ROBOT] Khoảng cách: ");
    Serial.println(khoangcach);
    
    // Kiểm tra khoảng cách và điều khiển động cơ
    if (khoangcach > gioihan || khoangcach == 0) {
      // Nếu không có vật cản, di chuyển về phía trước
      dithang();
    } else {
      // Nếu phát hiện vật cản, dừng lại và kiểm tra hướng tránh
      resetdongco();
      delay(300);
      
      // Quay servo sang trái và đo khoảng cách
      quaycbsangtrai();
      khoangcachtrai = khoangcach;
      
      // Quay servo sang phải và đo khoảng cách
      quaycbsangphai();
      khoangcachphai = khoangcach;
      
      // Đặt servo về vị trí trung tâm
      resetservo();
      
      // Quyết định hướng tránh vật cản
      if (khoangcachphai < 30 && khoangcachtrai < 30) {
        // Nếu cả hai bên đều có vật cản, lùi lại
        dilui();
      } else {
        // Nếu một bên có khoảng cách lớn hơn, rẽ sang hướng đó
        if (khoangcachphai >= khoangcachtrai) {
          disangphai();
        } else {
          disangtrai();
        }
      }
    }
  }

  // XỬ LÝ CẢM BIẾN MÔI TRƯỜNG (LUÔN HOẠT ĐỘNG)
  if (currentMillis - previousMillisSensors >= intervalSensors) {
    previousMillisSensors = currentMillis;
    readEnvironmentalSensors();
  }
  
  // GỬI DỮ LIỆU LÊN BLYNK (LUÔN HOẠT ĐỘNG)
  if (currentMillis - previousMillisBlynk >= intervalBlynk) {
    previousMillisBlynk = currentMillis;
    sendDataToBlynk();
  }
}

// ========== CÁC HÀM CHO ROBOT TRÁNH VẬT CẢN ==========

// Hàm di chuyển về phía trước
void dithang() {
  digitalWrite(tien1, HIGH);
  digitalWrite(tien2, HIGH);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
}

// Hàm rẽ phải
void disangphai() {
  resetdongco();
  digitalWrite(lui1, HIGH); // Lùi bánh xe trái
  digitalWrite(tien2, HIGH); // Tiến bánh xe phải
  delay(500); // Thời gian rẽ
  resetdongco();
}

// Hàm rẽ trái
void disangtrai() {
  resetdongco();
  digitalWrite(tien1, HIGH); // Tiến bánh xe trái
  digitalWrite(lui2, HIGH); // Lùi bánh xe phải
  delay(500); // Thời gian rẽ
  resetdongco();
}

// Hàm lùi lại
void dilui() {
  resetdongco();
  digitalWrite(lui1, HIGH);
  digitalWrite(lui2, HIGH);
  delay(1000); // Thời gian lùi
  resetdongco();
}

// Hàm dừng động cơ
void resetdongco() {
  digitalWrite(tien1, LOW);
  digitalWrite(tien2, LOW);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
}

// Hàm đo khoảng cách bằng cảm biến siêu âm
void dokhoangcach() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  thoigian = pulseIn(echo, HIGH);
  khoangcach = thoigian / 2 / 29.412; // Tính khoảng cách (cm)
}

// Hàm quay servo sang trái và đo khoảng cách
void quaycbsangtrai() {
  myservo.write(180); // Quay servo sang trái
  delay(1000); // Đợi servo quay
  dokhoangcach(); // Đo khoảng cách
  myservo.write(90); // Đặt servo về vị trí trung tâm
  delay(500);
}

// Hàm quay servo sang phải và đo khoảng cách
void quaycbsangphai() {
  myservo.write(0); // Quay servo sang phải
  delay(1000); // Đợi servo quay
  dokhoangcach(); // Đo khoảng cách
  myservo.write(90); // Đặt servo về vị trí trung tâm
  delay(500);
}

// Hàm đặt servo về vị trí trung tâm
void resetservo() {
  myservo.write(90);
}

// ========== CÁC HÀM CHO CẢM BIẾN MÔI TRƯỜNG ==========

// Hàm đọc giá trị từ các cảm biến môi trường
void readEnvironmentalSensors() {
  Serial.println("\n[CẢM BIẾN] Đọc dữ liệu từ cảm biến môi trường");
  
  // Đọc dữ liệu từ cảm biến DHT22
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Kiểm tra xem có lỗi đọc dữ liệu DHT22 không
  if (isnan(h) || isnan(t)) {
    Serial.println("[CẢM BIẾN] Lỗi đọc dữ liệu từ cảm biến DHT22!");
  } else {
    Serial.print("[CẢM BIẾN] Độ ẩm: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Nhiệt độ: ");
    Serial.print(t);
    Serial.println(" *C");
    
    // Gửi dữ liệu nhiệt độ lên Blynk (V0)
    Blynk.virtualWrite(V0, t);
    
    // Gửi dữ liệu độ ẩm lên Blynk (V6)
    Blynk.virtualWrite(V5, h);
  }

  // Đọc dữ liệu từ cảm biến MQ2 (Analog Output - A0)
  int mq2_value_a0 = analogRead(MQ2_PIN_A0);
  Serial.print("[CẢM BIẾN] Giá trị cảm biến MQ2 (Analog - A0): ");
  Serial.println(mq2_value_a0);
  
  // Gửi dữ liệu khí gas lên Blynk (V3)
  Blynk.virtualWrite(V3, mq2_value_a0);
  
  // Kiểm tra ngưỡng cảnh báo
  checkThresholds(t, mq2_value_a0, h);
  
  Serial.println("[CẢM BIẾN] --------------------------------");
}

// Hàm kiểm tra ngưỡng cảnh báo
void checkThresholds(float temperature, int gasValue, float humidity) {
  // Kiểm tra nếu nhiệt độ, khí gas, hoặc độ ẩm vượt ngưỡng
  if (temperature > TEMP_THRESHOLD || gasValue > GAS_THRESHOLD || humidity > HUMIDITY_THRESHOLD) {
    // Bật cảnh báo
    alarmState = true;
    Blynk.virtualWrite(V4, 1);  // Bật đèn cảnh báo trên Blynk
    
    // In thông báo cảnh báo cụ thể
    if (temperature > TEMP_THRESHOLD) {
      Serial.println("[CẢNH BÁO] Nhiệt độ vượt ngưỡng! ĐÈN CẢNH BÁO BẬT");
    }
    if (gasValue > GAS_THRESHOLD) {
      Serial.println("[CẢNH BÁO] Khí gas vượt ngưỡng! ĐÈN CẢNH BÁO BẬT");
    }
    if (humidity > HUMIDITY_THRESHOLD) {
      Serial.println("[CẢNH BÁO] Độ ẩm vượt ngưỡng! ĐÈN CẢNH BÁO BẬT");
    }
  } else {
    // Tắt cảnh báo
    alarmState = false;
    Blynk.virtualWrite(V4, 0);  // Tắt đèn cảnh báo trên Blynk
  }
}

// Hàm gửi dữ liệu lên Blynk
void sendDataToBlynk() {
  // Đọc dữ liệu từ cảm biến DHT22
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  // Đọc dữ liệu từ cảm biến MQ2
  int mq2_value_a0 = analogRead(MQ2_PIN_A0);
  
  // Kiểm tra xem có lỗi đọc dữ liệu DHT22 không
  if (!isnan(h) && !isnan(t)) {
    // Gửi dữ liệu nhiệt độ lên Blynk (V0)
    Blynk.virtualWrite(V0, t);
    
    // Gửi dữ liệu độ ẩm lên Blynk (V5)
    Blynk.virtualWrite(V5, h);
  }
  
  // Gửi dữ liệu khí gas lên Blynk (V3)
  Blynk.virtualWrite(V3, mq2_value_a0);
  
  // Kiểm tra ngưỡng cảnh báo
  checkThresholds(t, mq2_value_a0, h);
  
  // Gửi trạng thái kết nối lên V2 (LED báo kết nối)
  Blynk.virtualWrite(V2, 1);
}
