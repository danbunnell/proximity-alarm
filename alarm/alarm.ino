#include "ble_config.h"

/*
 * Proximity Alarm
 * 
 * @author Dan Bunnell (bunnell.dan@gmail.com)
 * 
 * Manages a Red-Bear Duo controlled sensor array and alarm that accepts motion-tracking info from 
 * a Bluetooth client device to understand location and sound an alarm when subject within proximity. 
 *
 * @attribution Based on previous code by Jon Froehlich, Liang He, Bjoern Hartmann, Chris Dziemborowicz and the RedBear Team
 */

#if defined(ARDUINO) 
SYSTEM_MODE(SEMI_AUTOMATIC); 
#endif

#define RECEIVE_MAX_LEN        5
#define SEND_MAX_LEN           3
#define SEND_INTERVAL_MS       200
#define SERVO_MOVE_INTERVAL_MS 200
#define BLE_SHORT_NAME_LEN 8 // must be in the range of [0x01, 0x09]

#define BLE_SHORT_NAME 'z', 'L', 'a', 'M', '3', '9', 'a' // length must be: BLE_SHORT_NAME_LEN - 1

#define CMD_SEND_DISTANCE 0x01

#define CMD_CLIENT_NOTIFY_SUBJECT_X_POSITION 0x01

// Define the pins on the Duo board
#define SERVO_OUT_PIN      D2
#define SONIC_TRIG_OUT_PIN D0
#define SONIC_ECHO_IN_PIN  D1
#define LED_OUT_PIN        D3
#define PIEZO_OUT_PIN      A0  // Does not work on Dx pins
#define BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN D7

#define MAX_SERVO_ANGLE  180
#define MIN_SERVO_ANGLE  0

#define DISTANCE_WINDOW_SIZE 3

#define PROXIMITY_THRESHOLD_CM 50

Servo servo;

// Device connected and disconnected callbacks
void deviceConnectedCallback(BLEStatus_t status, uint16_t handle);
void deviceDisconnectedCallback(uint16_t handle);

// UUID is used to find the device by other BLE-abled devices
static uint8_t service1_uuid[16]    = { 0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_tx_uuid[16] = { 0x71,0x3d,0x00,0x03,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_rx_uuid[16] = { 0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };

// Define the receive and send handlers
static uint16_t receive_handle = 0x0000; // recieve
static uint16_t send_handle = 0x0000; // send

static uint8_t receive_data[RECEIVE_MAX_LEN] = { 0x01 };
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size); // function declaration for receiving data callback
static uint8_t send_data[SEND_MAX_LEN] = { 0x00 };

// Define the configuration data
static uint8_t adv_data[] = {
  0x02,
  BLE_GAP_AD_TYPE_FLAGS,
  BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE, 
  
  BLE_SHORT_NAME_LEN,
  BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
  BLE_SHORT_NAME, 
  
  0x11,
  BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
  0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71 
};

int subject_x_position = 90;
static int subject_distance[DISTANCE_WINDOW_SIZE];
int next_distance_index = 0;
boolean is_alarm_on = false;

static btstack_timer_source_t send_characteristic;
static void send_data_callback(btstack_timer_source_t *ts);

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.print("Running setup()... ");

  // Initialize ble_stack.
  ble.init();
  
  // Register BLE callback functions
  ble.onConnectedCallback(bleConnectedCallback);
  ble.onDisconnectedCallback(bleDisconnectedCallback);

  //lots of standard initialization hidden in here - see ble_config.cpp
  configureBLE(); 
  
  // Set BLE advertising data
  ble.setAdvertisementData(sizeof(adv_data), adv_data);
  
  // Register BLE callback functions
  ble.onDataWriteCallback(bleReceiveDataCallback);

  // Add user defined service and characteristics
  ble.addService(service1_uuid);
  receive_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY|ATT_PROPERTY_WRITE|ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, receive_data, RECEIVE_MAX_LEN);
  send_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, send_data, SEND_MAX_LEN);

  // BLE peripheral starts advertising now.
  ble.startAdvertising();

  // Setup pins
  pinMode(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, OUTPUT);
  pinMode(SONIC_TRIG_OUT_PIN, OUTPUT);
  pinMode(SONIC_ECHO_IN_PIN, INPUT);
  pinMode(LED_OUT_PIN, OUTPUT);
  pinMode(PIEZO_OUT_PIN, OUTPUT);
  
  servo.attach(SERVO_OUT_PIN);
  servo.write( (int)((MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) / 2.0) );

  send_characteristic.process = &send_data_callback;
  ble.setTimer(&send_characteristic, SEND_INTERVAL_MS); 
  ble.addTimer(&send_characteristic);

  Serial.println("complete.");
}

void loop() 
{
  servo.write(map(subject_x_position, 0, 255, 60, 120));
  Serial.print("P: ");
  Serial.println(subject_x_position, DEC);
  delay(SERVO_MOVE_INTERVAL_MS);
}

/**
 * @brief Connect handle.
 *
 * @param[in]  status   BLE_STATUS_CONNECTION_ERROR or BLE_STATUS_OK.
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleConnectedCallback(BLEStatus_t status, uint16_t handle) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("BLE device connected!");
      digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, HIGH);
      break;
    default: break;
  }
}

/**
 * @brief Disconnect handle.
 *
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleDisconnectedCallback(uint16_t handle) {
  Serial.println("BLE device disconnected.");
  digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, LOW);
}

/**
 * @brief Callback for receiving data from Android (or whatever device you're connected to).
 *
 * @param[in]  value_handle  
 * @param[in]  *buffer       The buffer pointer of writting data.
 * @param[in]  size          The length of writting data.   
 *
 * @retval 
 */
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {

  if (receive_handle == value_handle) {
    memcpy(receive_data, buffer, RECEIVE_MAX_LEN);
    
    // process the data. 
    uint8_t command = receive_data[0];
    
    switch(command) {
      case CMD_CLIENT_NOTIFY_SUBJECT_X_POSITION:
        subject_x_position = receive_data[4];
        break;
      default:
        Serial.print("Received unknown command: ");
        Serial.println(command, HEX);
    }
    
    if (receive_data[0] == 0x01) { //receive the face data 
      // CSE590 Student TODO
      // Write code here that processes the FaceTrackerBLE data from Android
      // and properly angles the servo + ultrasonic sensor towards the face
      // Example servo code here: https://github.com/jonfroehlich/CSE590Sp2018/tree/master/L06-Arduino/RedBearDuoServoSweep   
    }
  }
  return 0;
}

/**
 * @brief Timer task for sending status change to client.
 * @param[in]  *ts   
 * 
 * @retval None
 * 
 * Send the data from either analog read or digital read back to 
 * the connected BLE device (e.g., Android)
 */
static void send_data_callback(btstack_timer_source_t *ts) {
  // CSE590 Student TODO
  // Write code that uses the ultrasonic sensor and transmits this to Android
  // Example ultrasonic code here: https://github.com/jonfroehlich/CSE590Sp2018/tree/master/L06-Arduino/RedBearDuoUltrasonicRangeFinder
  // Also need to check if distance measurement < threshold and sound alarm
  save_distance(read_ultrasonic_sensor());
  int moving_avg_distance = get_smoothed_subject_distance();
  
  send_distance(moving_avg_distance);

  Serial.print("D: ");
  Serial.println(moving_avg_distance, DEC);

  if (moving_avg_distance < PROXIMITY_THRESHOLD_CM && !is_alarm_on) {
    enable_alarm();
  } else if (moving_avg_distance >= PROXIMITY_THRESHOLD_CM && is_alarm_on) {
    disable_alarm();
  }
  
  // Restart timer.
  ble.setTimer(ts, 200);
  ble.addTimer(ts);
}

/**
 * @brief Sends the distance we measured for subject to client.
 * @param[in] distance The subject's distance
 * 
 * @retval None
 */
static void send_distance(int distance) {
  send_data[0] = CMD_SEND_DISTANCE;
  send_data[1] = distance >> 8;
  send_data[2] = distance;
  ble.sendNotify(send_handle, send_data, SEND_MAX_LEN);
}
/**
 * @brief  Reads a distance value from the ultrasonic sensor.
 * 
 * @retval The distance read
 */
static int read_ultrasonic_sensor() {
  // Clear trigger
  digitalWrite(SONIC_TRIG_OUT_PIN, LOW);
  delayMicroseconds(2);
  
  // Pulse trigger
  digitalWrite(SONIC_TRIG_OUT_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONIC_TRIG_OUT_PIN, LOW);
  
  // Read echo, or sound travel time
  float duration = pulseIn(SONIC_ECHO_IN_PIN, HIGH);
  
  // Calculating the distance sound traveled one-way in cm (Speed of sound = 0.034 cm/micros)
  return duration*0.034/2;
}

/**
 * @brief Gets the smoothed subject distance.
 * 
 * @retval Distance
 */
static int get_smoothed_subject_distance() {
  int sum = 0;

  for(int i = 0; i < DISTANCE_WINDOW_SIZE; i++) {
    sum += subject_distance[i];
  }
  
  return sum / DISTANCE_WINDOW_SIZE;
}

/**
 * @brief Saves the distance into moving average window.
 * @param[in] distance The distance to save
 * 
 * @retval None
 */
static void save_distance(int distance) {
  subject_distance[next_distance_index++ % DISTANCE_WINDOW_SIZE] = distance;
}

/**
 * @brief Enables the alarm.
 * 
 * @retval None
 */
static void enable_alarm() {
  Serial.println("ALARM ON");
  digitalWrite(LED_OUT_PIN, HIGH);
  tone(PIEZO_OUT_PIN, 1200);
  is_alarm_on = true;
}

/**
 * @brief Disables the alarm.
 * 
 * @retval None
 */
static void disable_alarm() {
  Serial.println("ALARM OFF");
  digitalWrite(LED_OUT_PIN, LOW);
  noTone(PIEZO_OUT_PIN);
  is_alarm_on = false;
}

