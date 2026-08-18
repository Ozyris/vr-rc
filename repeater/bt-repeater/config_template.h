// config.h

// === ОТЛАДКА ===
#define DEBUG

// === ТИП КОНТРОЛЛЕРА ===
// Раскомментируйте нужный тип
#define VRBOX      // VRBOX контроллер (газ кнопками)
//#define GAMEPAD    // Стандартный геймпад (газ на правом стике)

// === КАЛИБРОВКА СТИКОВ ===
#define CAL_CENTER_X 0
#define CAL_CENTER_Y 0
#define CAL_MIN_X -428
#define CAL_MAX_X 440
#define CAL_MIN_Y -436
#define CAL_MAX_Y 452

// === НАСТРОЙКИ ГАЗА ДЛЯ VRBOX ===
#define THROTTLE_STEP 50      // Шаг изменения газа (мкс)
#define THROTTLE_MIN 1000     // Минимальный газ
#define THROTTLE_MAX 2000     // Максимальный газ
#define THROTTLE_DEFAULT 1000 // Газ по умолчанию (0%)

// === ВЫБОР МИКШЕРА ===
#include "mixer_default.h"     // Микшер по умолчанию (1:1)
// #include "mixer_difthrst.h"      // Дифференциальная тяга
// #include "mixer_car.h"          // Микшер для машины
// #include "mixer_boat.h"         // Микшер для лодки
// #include "mixer_copter.h"       // Микшер для коптера