#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Servo.h>
#include <avr/sleep.h>
#include <avr/power.h>

// Define task handles
TaskHandle_t TaskSenseHandle;
TaskHandle_t TaskMovementHandle;
TaskHandle_t TaskPowerManagementHandle;

// Create servo object
Servo myservo;

// Global variables
const int trig = 6;
const int echo = 5;
int tien1 = 10;
int tien2 = 11;
int lui1 = 12;
int lui2 = 13;
int dongcoservo = 9;
int gioihan = 25; // Khoảng cách nhận biết vật
int khoangcach = 0;
int khoangcachtrai = 0;
int khoangcachphai = 0;
int maxspeed = 30;

// Biến để theo dõi trạng thái hoạt động
volatile bool isObstacleDetected = false;
volatile bool isMoving = false;
volatile bool isLowPowerMode = false;
volatile uint32_t lastActivityTime = 0;
const uint32_t INACTIVITY_TIMEOUT = 30000; // 30 giây không hoạt động sẽ vào chế độ tiết kiệm điện

// Mutex for shared resources
SemaphoreHandle_t xServoMutex;
SemaphoreHandle_t xDistanceMutex;
SemaphoreHandle_t xStateMutex;

// Function prototypes
void dokhoangcach();
void dithang();
void disangtrai();
void disangphai();
void dilui();
void resetdongco();
void quaycbsangphai();
void quaycbsangtrai();
void resetservo();
void enterLowPowerMode();
void exitLowPowerMode();
void updateActivityStatus();

// Task declarations
void TaskSense(void *pvParameters);
void TaskMovement(void *pvParameters);
void TaskPowerManagement(void *pvParameters);

void setup() {
  // Initialize servo
  myservo.attach(dongcoservo);
  
  // Initialize pins
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(tien1, OUTPUT);
  pinMode(tien2, OUTPUT);
  pinMode(lui1, OUTPUT);
  pinMode(lui2, OUTPUT);
  
  // Initialize outputs to LOW
  digitalWrite(tien1, LOW);
  digitalWrite(tien2, LOW);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
  
  // Center the servo
  myservo.write(90);
  delay(500);
  
  // Create mutexes
  xServoMutex = xSemaphoreCreateMutex();
  xDistanceMutex = xSemaphoreCreateMutex();
  xStateMutex = xSemaphoreCreateMutex();
  
  // Khởi tạo thời gian hoạt động
  lastActivityTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
  
  // Create tasks
  xTaskCreate(
    TaskSense,  // Task function
    "Sense",    // Task name
    128,        // Stack size
    NULL,       // Parameters
    2,          // Priority (higher number = higher priority)
    &TaskSenseHandle);
    
  xTaskCreate(
    TaskMovement,
    "Movement",
    128,
    NULL,
    1,
    &TaskMovementHandle);
  
  xTaskCreate(
    TaskPowerManagement,
    "PowerMgmt",
    128,
    NULL,
    3, // Ưu tiên cao nhất để có thể tạm dừng/tiếp tục các task khác
    &TaskPowerManagementHandle);
    
  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty. Things are done in Tasks.
}

// Task quản lý năng lượng
void TaskPowerManagement(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5000 / portTICK_PERIOD_MS; // Kiểm tra mỗi 5 giây
  
  // Initialize the xLastWakeTime variable with the current time
  xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    if (xSemaphoreTake(xStateMutex, (TickType_t)10) == pdTRUE) {
      uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
      
      if (!isLowPowerMode && (currentTime - lastActivityTime > INACTIVITY_TIMEOUT)) {
        // Không có hoạt động trong thời gian dài -> chuyển sang chế độ tiết kiệm năng lượng
        isLowPowerMode = true;
        
        // Tạm dừng các task không cần thiết
        vTaskSuspend(TaskMovementHandle);
        
        // Giảm tần suất đọc cảm biến
        // (Chỉ để đọc để phát hiện khi có vật cản xuất hiện)
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        // Tiết kiệm năng lượng
        enterLowPowerMode();
      } 
      else if (isLowPowerMode && isObstacleDetected) {
        // Có vật cản được phát hiện khi đang ở chế độ tiết kiệm -> trở lại chế độ hoạt động
        isLowPowerMode = false;
        lastActivityTime = currentTime;
        
        // Tiếp tục các task đã tạm dừng
        vTaskResume(TaskMovementHandle);
        
        // Trở lại chế độ hoạt động đầy đủ
        exitLowPowerMode();
      }
      
      xSemaphoreGive(xStateMutex);
    }
    
    // Đợi đến chu kỳ tiếp theo, tiết kiệm năng lượng khi đang chờ
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task to handle distance sensing
void TaskSense(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t xLastWakeTime;
  const TickType_t xNormalFrequency = 100 / portTICK_PERIOD_MS; // 100ms trong chế độ bình thường
  const TickType_t xLowPowerFrequency = 1000 / portTICK_PERIOD_MS; // 1s trong chế độ tiết kiệm
  
  // Initialize the xLastWakeTime variable with the current time
  xLastWakeTime = xTaskGetTickCount();
  
  for (;;) { // Infinite loop
    TickType_t currentFrequency;
    
    // Xác định tần suất dựa trên chế độ năng lượng
    if (xSemaphoreTake(xStateMutex, (TickType_t)10) == pdTRUE) {
      currentFrequency = isLowPowerMode ? xLowPowerFrequency : xNormalFrequency;
      xSemaphoreGive(xStateMutex);
    } else {
      currentFrequency = xNormalFrequency; // Mặc định nếu không thể lấy mutex
    }
    
    // Đo khoảng cách và cập nhật biến trạng thái
    if (xSemaphoreTake(xDistanceMutex, (TickType_t)10) == pdTRUE) {
      dokhoangcach();
      
      // Cập nhật biến phát hiện vật cản
      if (xSemaphoreTake(xStateMutex, (TickType_t)10) == pdTRUE) {
        isObstacleDetected = (khoangcach <= gioihan && khoangcach > 0);
        
        // Nếu phát hiện vật cản, cập nhật thời gian hoạt động
        if (isObstacleDetected) {
          lastActivityTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        
        xSemaphoreGive(xStateMutex);
      }
      
      xSemaphoreGive(xDistanceMutex);
    }
    
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, currentFrequency);
  }
}

// Task to handle robot movement
void TaskMovement(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 200 / portTICK_PERIOD_MS; // 200ms
  
  // Initialize the xLastWakeTime variable with the current time
  xLastWakeTime = xTaskGetTickCount();
  
  for (;;) { // Infinite loop
    // Take mutex to read shared distance variable
    if (xSemaphoreTake(xDistanceMutex, (TickType_t)10) == pdTRUE) {
      int currentDistance = khoangcach;
      xSemaphoreGive(xDistanceMutex);
      
      // Cập nhật trạng thái di chuyển
      bool wasMoving = false;
      if (xSemaphoreTake(xStateMutex, (TickType_t)10) == pdTRUE) {
        wasMoving = isMoving;
        xSemaphoreGive(xStateMutex);
      }
      
      if (currentDistance > gioihan || currentDistance == 0) {
        dithang();
        
        if (xSemaphoreTake(xStateMutex, (TickType_t)10) == pdTRUE) {
          isMoving = true;
          // Cập nhật thời gian hoạt động nếu trạng thái di chuyển thay đổi
          if (!wasMoving) {
            lastActivityTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
          }
          xSemaphoreGive(xStateMutex);
        }
      } else {
        resetdongco();
        
        if (xSemaphoreTake(xStateMutex, (TickType_t)10) == pdTRUE) {
          isMoving = false;
          xSemaphoreGive(xStateMutex);
        }
        
        // Cập nhật thời gian hoạt động khi có tác động
        lastActivityTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Check left and right distances
        if (xSemaphoreTake(xServoMutex, (TickType_t)100) == pdTRUE) {
          quaycbsangtrai();
          if (xSemaphoreTake(xDistanceMutex, (TickType_t)10) == pdTRUE) {
            khoangcachtrai = khoangcach;
            xSemaphoreGive(xDistanceMutex);
          }
          
          quaycbsangphai();
          if (xSemaphoreTake(xDistanceMutex, (TickType_t)10) == pdTRUE) {
            khoangcachphai = khoangcach;
            xSemaphoreGive(xDistanceMutex);
          }
          
          resetservo();
          xSemaphoreGive(xServoMutex);
        }
        
        // Decide movement direction based on distances
        if (khoangcachphai < 30 && khoangcachtrai < 30) {
          dilui();
        } else {
          if (khoangcachphai >= khoangcachtrai) {
            disangphai();
          } else {
            disangtrai();
          }
        }
      }
    }
    
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Các chức năng tiết kiệm năng lượng
void enterLowPowerMode() {
  // Tắt các chức năng không cần thiết
  resetdongco(); // Tắt động cơ
  power_adc_disable(); // Tắt ADC nếu không cần
  
  // Có thể thêm các lệnh tắt các ngoại vi khác tùy thuộc vào phần cứng
  
  // Lưu ý: Không tắt các chức năng mà FreeRTOS cần để hoạt động
  // Không gọi sleep_mode() vì sẽ làm FreeRTOS ngừng hoạt động
}

void exitLowPowerMode() {
  // Bật lại các chức năng đã tắt
  power_adc_enable(); // Bật ADC
  
  // Có thể thêm các lệnh bật lại các ngoại vi khác
}

void updateActivityStatus() {
  lastActivityTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// Functions for distance measurement and movement
void dokhoangcach() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  // Đo độ rộng xung HIGH ở chân echo.
  unsigned long thoigian = pulseIn(echo, HIGH);
  khoangcach = thoigian / 2 / 29.412;
}

void dithang() {
  digitalWrite(tien1, HIGH);
  digitalWrite(tien2, HIGH);
}

void disangphai() {
  resetdongco();
  digitalWrite(lui1, HIGH);
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Using FreeRTOS delay instead of Arduino delay
  digitalWrite(lui1, LOW);
}

void disangtrai() {
  resetdongco();
  digitalWrite(lui2, HIGH);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  digitalWrite(lui2, LOW);
}

void dilui() {
  resetdongco();
  digitalWrite(lui1, HIGH);
  digitalWrite(lui2, HIGH);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
}

void resetdongco() {
  digitalWrite(tien1, LOW);
  digitalWrite(tien2, LOW);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
}

void quaycbsangtrai() {
  myservo.write(180);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  dokhoangcach();
  myservo.write(90);
}

void quaycbsangphai() {
  myservo.write(0);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  dokhoangcach();
  myservo.write(90);
}

void resetservo() {
  myservo.write(90);
}