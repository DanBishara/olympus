/*
* File: BLE.cpp
* Author: Daniel Bishara
* Date: March 1, 2026
* Description: define class methods for the BLE class
*/

#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>

#include "BLE.h"
#include "stepCounter.h"

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

LOG_MODULE_REGISTER( BLEManager, CONFIG_LOG_DEFAULT_LEVEL );

static struct k_work_delayable bleWorkItem;

#define TEST_SERVICE_UUID_VAL  BT_UUID_128_ENCODE(0x88776655, 0x4433, 0x2211, 0xFFEE, 0xDDCCABAA0000)
#define TEST_CHAR_UUID_VAL     BT_UUID_128_ENCODE(0x88776655, 0x4433, 0x2211, 0xFFEE, 0xDDCCACAA0000)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, TEST_SERVICE_UUID_VAL),
};

static struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_NAME,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    NULL
);

/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/*
 * Custom service/characteristic.
 * UUIDs generated arbitrarily for example purposes.
 */
static struct bt_uuid_128 test_service_uuid = BT_UUID_INIT_128(TEST_SERVICE_UUID_VAL);
static struct bt_uuid_128 test_char_uuid    = BT_UUID_INIT_128(TEST_CHAR_UUID_VAL);

static uint32_t test_data = 0;
static bool test_notify_enabled;

static ssize_t read_test(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len,
                          uint16_t offset)
{
    const uint32_t *data = (const uint32_t *)attr->user_data;
    LOG_INF( "Read request for test data: %u", *data );
	return bt_gatt_attr_read(conn, attr, buf, len, offset, data,
	                         sizeof(*data));
}

static void test_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                  uint16_t value)
{
	test_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE( test_svc,
	BT_GATT_PRIMARY_SERVICE(&test_service_uuid),
	BT_GATT_CHARACTERISTIC(&test_char_uuid.uuid,
	                       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
	                       BT_GATT_PERM_READ,
	                       read_test, NULL, &test_data),
	BT_GATT_CCC(test_ccc_cfg_changed,
	            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    );

/*
 * Step Counter Service
 * Service UUID:        BB-AB base
 * Characteristic UUID: BB-AC base
 */
static struct bt_uuid_128 step_counter_svc_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x88776655, 0x4433, 0x2211, 0xFFEE, 0xDDCCABBB0000)
);

static struct bt_uuid_128 step_count_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x88776655, 0x4433, 0x2211, 0xFFEE, 0xDDCCACBB0000)
);

static ssize_t read_step_count( struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len,
                                 uint16_t offset )
{
    uint32_t steps = StepCounter::Instance().getStepCount();
    LOG_INF( "Read request for step count: %u", steps );
    return bt_gatt_attr_read( conn, attr, buf, len, offset, &steps, sizeof(steps) );
}

BT_GATT_SERVICE_DEFINE( step_counter_svc,
    BT_GATT_PRIMARY_SERVICE( &step_counter_svc_uuid ),
    BT_GATT_CHARACTERISTIC( &step_count_char_uuid.uuid,
                            BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ,
                            read_step_count, NULL, NULL ),
);

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
    } else {
        LOG_INF("Connected!");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Disconnected (reason %u)", reason);
    test_data++;
    // Need to restart advertising when a client disconnects so that it can accept new connections
    k_work_schedule(&bleWorkItem, K_MSEC(50));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

void bleWorkHandler( struct k_work *work )
{
    LOG_INF("BLE work handler executed");
    // Should change to add adbertising data and scan response data, but for now just restart advertising with the same data
    int err = bt_le_adv_start(&adv_param, NULL, 0,
			      NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}
}

// From beacom example, not sure if needed but will keep for now
void bt_ready(int err)
{
	char addr_s[BT_ADDR_LE_STR_LEN];
	bt_addr_le_t addr = {0};
	size_t count = 1;

	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return;
	}

	LOG_INF("Bluetooth initialized\n");

	/* Start advertising */
	err = bt_le_adv_start(&adv_param, NULL, 0,
			      NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}

	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

	LOG_INF("Beacon started, advertising as %s\n", addr_s);
}

ErrCode_t BLEManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;

    zephyrCode = bt_enable(bt_ready);
    if( zephyrCode )
    {
        LOG_ERR( "Bluetooth init failed (err %d)", zephyrCode );
        goto exit;
    }

    k_work_init_delayable( &bleWorkItem, bleWorkHandler );

    errCode = ErrCode_Success;
exit:
    return errCode;
}