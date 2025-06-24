#include "ble.h"
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include "aoa_calc.h"

#define NAME_LEN 30
#define SYNC_CREATE_TIMEOUT_INTERVAL_NUM 10
#define ADV_DATA_HEX_STR_LEN_MAX (BT_GAP_ADV_MAX_EXT_ADV_DATA_LEN * 2 + 1)

#if defined(CONFIG_BT_DF_CTE_RX_AOA)
const static uint8_t ant_patterns[] = {0x5, 0x4, 0x7, 0x6, 0x1, 0x0, 0x3, 0x2, // Antenna switching pattern
                                       0xD, 0xC, 0xF, 0xE, 0x9, 0x8, 0xB, 0xA};
#endif

static ble_ctx_t *ble_ctx_ptr = NULL;

static uint16_t sync_create_timeout_get(uint16_t interval)
{
    uint32_t interval_us;
    uint32_t timeout;
    interval_us = BT_GAP_PER_ADV_INTERVAL_TO_US(interval);
    timeout = BT_GAP_US_TO_PER_ADV_SYNC_TIMEOUT(interval_us) * SYNC_CREATE_TIMEOUT_INTERVAL_NUM;
    timeout = CLAMP(timeout, BT_GAP_PER_ADV_MIN_TIMEOUT, BT_GAP_PER_ADV_MAX_TIMEOUT);
    return (uint16_t)timeout;
}

static bool data_cb(struct bt_data *data, void *user_data)
{
    char *name = user_data;
    uint8_t len;

    switch (data->type)
    {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
        len = MIN(data->data_len, NAME_LEN - 1);
        memcpy(name, data->data, len);
        name[len] = '\0';
        return false;
    default:
        return true;
    }
}

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
    ble_ctx_t *ctx = ble_ctx_ptr;
    char le_addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
    printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, Interval 0x%04x (%u ms), PHY %u\n",
           bt_le_per_adv_sync_get_index(sync), le_addr, info->interval, info->interval * 5 / 4,
           info->phy);
    k_sem_give(&ctx->sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info)
{
    ble_ctx_t *ctx = ble_ctx_ptr;
    char le_addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
    printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
           bt_le_per_adv_sync_get_index(sync), le_addr);

    if (ctx->sync_wait)
    {
        ctx->sync_terminated = true;
        k_sem_give(&ctx->sem_per_sync);
    }
    else
    {
        k_sem_give(&ctx->sem_per_sync_lost);
    }
}

static void recv_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf)
{
    static char data_str[ADV_DATA_HEX_STR_LEN_MAX];
    char le_addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
    bin2hex(buf->data, buf->len, data_str, sizeof(data_str));
    printk("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, RSSI %i, data length %u, data: %s\n",
           bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power, info->rssi,
           buf->len, data_str);
}

static void cte_recv_cb(struct bt_le_per_adv_sync *sync, const struct bt_df_per_adv_sync_iq_samples_report *report)
{
    ble_ctx_t *ctx = ble_ctx_ptr;
    rot3d_t angle;
    // You must provide your own AoA calculation function!
    extern bool calculate_aoa(const struct bt_df_per_adv_sync_iq_samples_report *report, rot3d_t *angle_deg);
    if (!calculate_aoa(report, &angle))
    {
        printk("AoA calculation failed\n");
        return;
    }

    struct bt_le_per_adv_sync_info info;
    bt_le_per_adv_sync_get_info(sync, &info);

    ctx->aoa_cb(angle);

    ble_delete_sync(ctx);
}

static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
    ble_ctx_t *ctx = ble_ctx_ptr;
    char le_addr[BT_ADDR_LE_STR_LEN];
    char name[NAME_LEN];
    (void)memset(name, 0, sizeof(name));
    bt_data_parse(buf, data_cb, name);
    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

    if (!ctx->per_adv_found && info->interval != 0)
    {
        ctx->sync_create_timeout = sync_create_timeout_get(info->interval);
        ctx->per_adv_found = true;
        ctx->per_sid = info->sid;
        bt_addr_le_copy(&ctx->per_addr, info->addr);
        k_sem_give(&ctx->sem_per_adv);
    }
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
    .synced = sync_cb,
    .term = term_cb,
    .recv = recv_cb,
    .cte_report_cb = cte_recv_cb,
};

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
};

void ble_init(ble_ctx_t *ctx)
{
    ble_ctx_ptr = ctx;
    k_sem_init(&ctx->sem_per_adv, 0, 1);
    k_sem_init(&ctx->sem_per_sync, 0, 1);
    k_sem_init(&ctx->sem_per_sync_lost, 0, 1);
    ctx->scan_enabled = false;
    ctx->per_adv_found = false;
    ctx->sync_wait = false;
    ctx->sync_terminated = false;
    bt_le_scan_cb_register(&scan_callbacks);
    bt_le_per_adv_sync_cb_register(&sync_callbacks);
}

bool ble_scan_enable(ble_ctx_t *ctx)
{
    struct bt_le_scan_param param = {
        .type = BT_LE_SCAN_TYPE_ACTIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW,
        .timeout = 0U,
    };
    int err;

    if (!ctx->scan_enabled)
    {
        printk("Start scanning...");
        err = bt_le_scan_start(&param, NULL);
        if (err != 0)
        {
            printk("failed (err %d)\n", err);
            return false;
        }
        printk("success\n");
        ctx->scan_enabled = true;
    }
    return true;
}

void ble_scan_disable(ble_ctx_t *ctx)
{
    int err;
    printk("Scan disable...");
    err = bt_le_scan_stop();
    if (err != 0)
    {
        printk("failed (err %d)\n", err);
        return;
    }
    printk("Success.\n");
    ctx->scan_enabled = false;
}

void ble_create_sync(ble_ctx_t *ctx)
{
    int err;
    printk("Creating Periodic Advertising Sync...");
    printk(" with address %s, SID %u, timeout %u ms...",
           bt_addr_le_str(&ctx->per_addr), ctx->per_sid, ctx->sync_create_timeout);
    bt_addr_le_copy(&ctx->sync_create_param.addr, &ctx->per_addr);
    ctx->sync_create_param.options = BT_LE_PER_ADV_SYNC_OPT_SYNC_ONLY_CONST_TONE_EXT;
    ctx->sync_create_param.sid = ctx->per_sid;
    ctx->sync_create_param.skip = 0;
    ctx->sync_create_param.timeout = ctx->sync_create_timeout;
    err = bt_le_per_adv_sync_create(&ctx->sync_create_param, &ctx->adv_sync);
    if (err != 0)
    {
        printk("failed (err %d)\n", err);
        return;
    }
    printk("success.\n");
}

void ble_enable_cte_rx(ble_ctx_t *ctx)
{
    int err;
    const struct bt_df_per_adv_sync_cte_rx_param cte_rx_params = {
        .max_cte_count = 5,
#if defined(CONFIG_BT_DF_CTE_RX_AOA)
        .cte_types = BT_DF_CTE_TYPE_ALL,
        .slot_durations = 0x2,
        .num_ant_ids = ARRAY_SIZE(ant_patterns),
        .ant_ids = ant_patterns,
#else
        .cte_types = BT_DF_CTE_TYPE_AOD_1US | BT_DF_CTE_TYPE_AOD_2US,
#endif
    };
    printk("Enable receiving of CTE...\n");
    err = bt_df_per_adv_sync_cte_rx_enable(ctx->adv_sync, &cte_rx_params);
    if (err != 0)
    {
        printk("failed (err %d)\n", err);
        return;
    }
    printk("success. CTE receive enabled.\n");
}

void ble_delete_sync(ble_ctx_t *ctx)
{
    int err;
    printk("Deleting Periodic Advertising Sync...");
    err = bt_le_per_adv_sync_delete(ctx->adv_sync);
    if (err != 0)
    {
        printk("failed (err %d)\n", err);
        return;
    }
    printk("success\n");
}

void ble_wait_for_adv(ble_ctx_t *ctx)
{
    k_sem_take(&ctx->sem_per_adv, K_FOREVER);
}

void ble_wait_for_sync(ble_ctx_t *ctx)
{
    k_sem_take(&ctx->sem_per_sync, K_FOREVER);
}

void ble_wait_for_sync_lost(ble_ctx_t *ctx)
{
    k_sem_take(&ctx->sem_per_sync_lost, K_FOREVER);
}