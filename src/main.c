#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

// Must match sensor packet structure EXACTLY
typedef struct __attribute__((packed)) {
    uint8_t node_id;      // 1 byte
    uint32_t timestamp;   // 4 bytes
    int16_t accel_x;      // 2 bytes
    int16_t accel_y;      // 2 bytes
    int16_t accel_z;      // 2 bytes
    int16_t gyro_x;       // 2 bytes
    int16_t gyro_y;       // 2 bytes
    int16_t gyro_z;       // 2 bytes
} sensor_packet_t;
// Total: 15 bytes

static const char *TAG = "BLE_CENTRAL";
// Sensor node address and connection handle
static uint8_t sensor_addr[6];
static bool sensor_found = false;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
// IMU service handles
static uint16_t imu_service_start_handle = 0;
static uint16_t imu_service_end_handle = 0;

// Characteristic handles
static uint16_t packet_handle = 0;

// Sensor data storage


static void ble_app_scan(void);
static void ble_app_scan(void);  


static int ble_gattc_disc_svc_cb(uint16_t conn_handle,const struct ble_gatt_error *error,
                                  const struct ble_gatt_svc *service,void *arg);

 static int ble_gattc_disc_chr_cb(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  const struct ble_gatt_chr *chr,
                                  void *arg);   
                                  
static int ble_gattc_read_cb(uint16_t conn_handle,
                              const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr,
                              void *arg);

static void sensor_read_task(void *param)
{
    ESP_LOGI(TAG, "Sensor read task started");
    
    while (1) {
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE && packet_handle != 0) {
            // Read single 15-byte packet
            ble_gattc_read(conn_handle, packet_handle, ble_gattc_read_cb, NULL);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Repeat every second
    }
}

static char *addr_str(const uint8_t *addr)
{
    static char buf[18];
    sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
            addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    return buf;
}

// Callback when a device is discovered during scanning
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connection established!");
            conn_handle = event->connect.conn_handle;
            
            // Discover all services and characteristics
            ESP_LOGI(TAG, "Discovering services...");
            ble_gattc_disc_all_svcs(conn_handle, ble_gattc_disc_svc_cb, NULL);

        } else {
            ESP_LOGI(TAG, "Connection failed, status=%d", event->connect.status);
            sensor_found = false;
            ble_app_scan();  // Restart scanning
        }
        break;
    
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected, reason=%d", event->disconnect.reason);
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            sensor_found = false;
            ble_app_scan();  // Restart scanning
            break;
    case BLE_GAP_EVENT_DISC:
        // A device was discovered
        ESP_LOGI(TAG, "Device discovered:");
        ESP_LOGI(TAG, "  Address: %s", addr_str(event->disc.addr.val));
        
        // Check if this is our sensor node
        if (event->disc.length_data > 0) {
            // Print the device name if available
            struct ble_hs_adv_fields fields;
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            
            if (fields.name != NULL) {
                char name[32] = {0};
                memcpy(name, fields.name, fields.name_len);
                ESP_LOGI(TAG, "  Name: %s", name);
                
              if (strncmp(name, "ESP32_Sensor", 12) == 0 && !sensor_found) {
                ESP_LOGI(TAG, "Found our sensor node! Connecting...");
                
                // Store the address
                memcpy(sensor_addr, event->disc.addr.val, 6);
                sensor_found = true;
                
                // Stop scanning
                ble_gap_disc_cancel();
                
                // Connect to the sensor
                struct ble_gap_conn_params conn_params = {0};
                conn_params.scan_itvl = 0x0010;
                conn_params.scan_window = 0x0010;
                conn_params.itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN;
                conn_params.itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX;
                conn_params.latency = 0;
                conn_params.supervision_timeout = 0x0100;
                conn_params.min_ce_len = 0;
                conn_params.max_ce_len = 0;
                
                ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000, 
                                &conn_params, ble_gap_event, NULL);
            }
            }
        }
        break;
        
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Scan complete, restarting scan...");
        ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, NULL, ble_gap_event, NULL);
        break;
    }
    
    return 0;
}

static void ble_app_scan(void)
{
    struct ble_gap_disc_params disc_params = {0};
    disc_params.filter_duplicates = 1;
    disc_params.passive = 0;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;
    
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
    ESP_LOGI(TAG, "Started scanning for devices...");
}

static int ble_gattc_disc_svc_cb(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  const struct ble_gatt_svc *service,
                                  void *arg)
{
    if (error->status != 0 && error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "Service discovery error: %d", error->status);
        return 0;
    }
    
    if (error->status == BLE_HS_EDONE) {
    ESP_LOGI(TAG, "Service discovery complete");
    
    if (imu_service_start_handle != 0) {
        ESP_LOGI(TAG, "Discovering characteristics in IMU service...");
        ble_gattc_disc_all_chrs(conn_handle, imu_service_start_handle,
                                imu_service_end_handle, ble_gattc_disc_chr_cb, NULL);
    }
    return 0;
}
    
    // A service was found
    if (service != NULL) {
    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&service->uuid.u, uuid_str);
    ESP_LOGI(TAG, "Found service: %s", uuid_str);
    
    // Check if this is our IMU service
    if (ble_uuid_cmp(&service->uuid.u, BLE_UUID16_DECLARE(0x1234)) == 0) {
        ESP_LOGI(TAG, "  This is our IMU service!");
        imu_service_start_handle = service->start_handle;
        imu_service_end_handle = service->end_handle;
    }
}
    
    return 0;
}

static int ble_gattc_disc_chr_cb(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  const struct ble_gatt_chr *chr,
                                  void *arg)
{
    if (error->status != 0 && error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "Characteristic discovery error: %d", error->status);
        return 0;
    }
    
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Characteristic discovery complete");



                // Start the reading task
    xTaskCreate(sensor_read_task, "sensor_read", 4096, NULL, 5, NULL);

        return 0;
    }
    
if (chr != NULL) {
    // Look for our packet characteristic (UUID 0x2A00)
    if (ble_uuid_cmp(&chr->uuid.u, BLE_UUID16_DECLARE(0x2A00)) == 0) {
        packet_handle = chr->val_handle;
        ESP_LOGI(TAG, "Found packet characteristic, handle=%d", packet_handle);
    }
}
    
    return 0;
}

static int ble_gattc_read_cb(uint16_t conn_handle,
                              const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr,
                              void *arg)
{
if (error->status != 0) {
        ESP_LOGE(TAG, "Read error: %d", error->status);
        return 0;
    }
    
    // Verify we received exactly 15 bytes
    if (attr->om->om_len == sizeof(sensor_packet_t)) {
        sensor_packet_t packet;
        
        // Copy binary data into struct
        memcpy(&packet, attr->om->om_data, sizeof(sensor_packet_t));
        
        // Display formatted output
        printf("\n=== NODE %02X @ %lu ms ===\n", packet.node_id, (unsigned long)packet.timestamp);
        printf("Accel: X=%6d  Y=%6d  Z=%6d\n", 
               packet.accel_x, packet.accel_y, packet.accel_z);
        printf("Gyro:  X=%6d  Y=%6d  Z=%6d\n", 
               packet.gyro_x, packet.gyro_y, packet.gyro_z);
        printf("==========================\n");
    } else {
        ESP_LOGE(TAG, "Unexpected packet size: %d bytes (expected %d)", 
                 attr->om->om_len, sizeof(sensor_packet_t));
    }
    
    return 0;
}

static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack synced");
    ble_app_scan();
}

static void host_task(void *param)
{
    nimble_port_run();
}

void app_main(void)
{

        // Suppress all ESP-IDF component logs (NimBLE, etc)
        // Suppress all ESP-IDF component logs (NimBLE, etc)
    esp_log_level_set("*", ESP_LOG_ERROR);
    
    // Keep only our app logs at INFO level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    nvs_flash_init();
    
    ESP_LOGI(TAG, "BLE Central starting...");
    
    nimble_port_init();
    
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    
    nimble_port_freertos_init(host_task);
}