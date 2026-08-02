#include <Bluepad32.h>
#include <ESP32Servo.h>

#define LED_PIN 22
#define SERVO1_PIN 12
#define SERVO2_PIN 14

#define LED_ON LOW
#define LED_OFF HIGH

// === КАЛИБРОВОЧНЫЕ ЗНАЧЕНИЯ (вставьте свои) ===
#define CAL_CENTER_X 0
#define CAL_CENTER_Y 0
#define CAL_MIN_X -428
#define CAL_MAX_X 440
#define CAL_MIN_Y -436
#define CAL_MAX_Y 452
// =============================================

// Создаем объекты серв
Servo servo1;
Servo servo2;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Текущие углы серв
int currentAngle1 = 90;
int currentAngle2 = 90;

// Мертвая зона для центра (чтобы избежать дрейфа)
const int DEAD_ZONE = 10;

// Функция для преобразования значения оси в угол с учетом калибровки
int mapAxisToAngle(int axisValue, int center, int minVal, int maxVal) {
    // Применяем мертвую зону
    if (abs(axisValue - center) < DEAD_ZONE) {
        return 90; // Центр
    }
    
    // Нормализуем значение относительно центра
    int normalized;
    if (axisValue < center) {
        // Левая/верхняя половина
        normalized = map(axisValue, minVal, center, 0, 90);
    } else {
        // Правая/нижняя половина
        normalized = map(axisValue, center, maxVal, 90, 180);
    }
    
    // Ограничиваем
    if (normalized < 0) normalized = 0;
    if (normalized > 180) normalized = 180;
    
    return normalized;
}

// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", 
                         ctl->getModelName().c_str(), 
                         properties.vendor_id,
                         properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            
            digitalWrite(LED_PIN, LED_ON); // 
            
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            
            bool anyControllerConnected = false;
            for (int j = 0; j < BP32_MAX_GAMEPADS; j++) {
                if (myControllers[j] != nullptr) {
                    anyControllerConnected = true;
                    break;
                }
            }
            
            if (!anyControllerConnected) {
                digitalWrite(LED_PIN, LED_OFF);
            }
            
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    int rawX = ctl->axisX();
    int rawY = ctl->axisY();
    
    // Показываем сырые и откалиброванные значения
    int calX = mapAxisToAngle(rawX, CAL_CENTER_X, CAL_MIN_X, CAL_MAX_X);
    int calY = mapAxisToAngle(rawY, CAL_CENTER_Y, CAL_MIN_Y, CAL_MAX_Y);
    
    Serial.printf(
        "idx=%d, buttons: 0x%04x, raw: %4d, %4d, cal: %3d°, %3d°\n",
        ctl->index(),
        ctl->buttons(),
        rawX, rawY,
        calX, calY
    );
}

void processGamepad(ControllerPtr ctl) {
    // Получаем сырые значения с джойстика
    int axisX = ctl->axisX();
    int axisY = ctl->axisY();
    
    // Преобразуем с учетом калибровки
    int angle1 = mapAxisToAngle(axisX, CAL_CENTER_X, CAL_MIN_X, CAL_MAX_X);
    int angle2 = mapAxisToAngle(axisY, CAL_CENTER_Y, CAL_MIN_Y, CAL_MAX_Y);
    
    // Обновляем сервы только если угол изменился
    if (angle1 != currentAngle1) {
        servo1.write(angle1);
        currentAngle1 = angle1;
    }
    
    if (angle2 != currentAngle2) {
        servo2.write(angle2);
        currentAngle2 = angle2;
    }

    // Кнопки для дополнительных функций
    if (ctl->a()) {
        // Центрируем сервы при нажатии A
        servo1.write(90);
        servo2.write(90);
        currentAngle1 = 90;
        currentAngle2 = 90;
        Serial.println("Servos centered!");
    }

    if (ctl->b()) {
        // Демонстрация вращения серв
        static int angle = 0;
        angle = (angle + 2) % 180;
        servo1.write(angle);
        servo2.write(180 - angle);
        currentAngle1 = angle;
        currentAngle2 = 180 - angle;
        Serial.printf("Servo demo: %d°\n", angle);
    }

    if (ctl->x()) {
        // Виброотклик
        ctl->playDualRumble(0, 250, 0x80, 0x40);
    }

    // Выводим данные в Serial (для отладки)
    dumpGamepad(ctl);
}

void processMouse(ControllerPtr ctl) {
    // Можно добавить управление сервами через мышь
    int deltaX = ctl->deltaX();
    int deltaY = ctl->deltaY();
    
    // Пример: используем мышь для управления
    if (abs(deltaX) > 5) {
        int angle = constrain(currentAngle1 + deltaX / 10, 0, 180);
        servo1.write(angle);
        currentAngle1 = angle;
    }
    
    if (abs(deltaY) > 5) {
        int angle = constrain(currentAngle2 + deltaY / 10, 0, 180);
        servo2.write(angle);
        currentAngle2 = angle;
    }
}

void processKeyboard(ControllerPtr ctl) {
    if (!ctl->isAnyKeyPressed())
        return;

    // Управление через клавиатуру
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        int angle = constrain(currentAngle1 - 5, 0, 180);
        servo1.write(angle);
        currentAngle1 = angle;
    }
    
    if (ctl->isKeyPressed(Keyboard_RightArrow)) {
        int angle = constrain(currentAngle1 + 5, 0, 180);
        servo1.write(angle);
        currentAngle1 = angle;
    }
    
    if (ctl->isKeyPressed(Keyboard_UpArrow)) {
        int angle = constrain(currentAngle2 - 5, 0, 180);
        servo2.write(angle);
        currentAngle2 = angle;
    }
    
    if (ctl->isKeyPressed(Keyboard_DownArrow)) {
        int angle = constrain(currentAngle2 + 5, 0, 180);
        servo2.write(angle);
        currentAngle2 = angle;
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    esp_log_level_set("*", ESP_LOG_NONE);
    
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Настройка пина LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    // Подключение сервомашинок
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo1.attach(SERVO1_PIN, 1000, 2000);
    servo2.attach(SERVO2_PIN, 1000, 2000);
    
    // Начальное положение - центр
    servo1.write(90);
    servo2.write(90);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    delay(50);
}
