#ifndef BLE_LE_H
#define BLE_LE_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/direction.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#define BLE_MAX_BEACONS 4

typedef void (*ble_aoa_report_cb_t)(int beacon_idx, double angle);

typedef struct
{
    struct bt_le_per_adv_sync_param sync_create_param;
    struct bt_le_per_adv_sync *adv_sync;
    bt_addr_le_t beacon_addrs[BLE_MAX_BEACONS];
    int num_beacons;
    bool per_adv_found;
    bool scan_enabled;
    bool sync_wait;
    bool sync_terminated;
    uint8_t per_sid;
    uint16_t sync_create_timeout;
    int beacon_idx;
    struct k_sem sem_per_adv;
    struct k_sem sem_per_sync;
    struct k_sem sem_per_sync_lost;
    ble_aoa_report_cb_t aoa_cb;
    bt_addr_le_t per_addr;

} ble_ctx_t;

void ble_init(ble_ctx_t *ctx);
bool ble_scan_enable(ble_ctx_t *ctx);
void ble_scan_disable(ble_ctx_t *ctx);
void ble_create_sync(ble_ctx_t *ctx);
void ble_enable_cte_rx(ble_ctx_t *ctx);
void ble_delete_sync(ble_ctx_t *ctx);

void ble_wait_for_adv(ble_ctx_t *ctx);
void ble_wait_for_sync(ble_ctx_t *ctx);
void ble_wait_for_sync_lost(ble_ctx_t *ctx);

#endif