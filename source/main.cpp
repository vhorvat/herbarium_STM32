/* mbed Microcontroller Library
 * Copyright (c) 2017 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdint>
#include <cstdio>
#include <stdio.h>

#include "platform/Callback.h"
#include "events/EventQueue.h"
#include "platform/NonCopyable.h"

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattClient.h"
#include "ble/GapAdvertisingParams.h"
#include "ble/GapAdvertisingData.h"
#include "ble/GattServer.h"
#include "BLEProcess.h"
#include "XNucleoIKS01A3.h"

using mbed::callback;


static XNucleoIKS01A3 *mems_expansion_board = XNucleoIKS01A3::instance(D14, D15, D4, D5, A3, D6, A4);
static HTS221Sensor  *ht_sensor = mems_expansion_board->ht_sensor;
static LPS22HHSensor *pt_sensor = mems_expansion_board->pt_sensor;

float temperature = 0.0f;
float humidity = 0.0f;
float air_pressure = 0.0f;

uint8_t index = 0;
uint8_t row = -1;
uint8_t historic_row = 0;
uint8_t historic[48][17];
uint8_t sensors[17] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
//uint8_t historic[30][50] = {};

AnalogIn soil_m(A1);
AnalogIn MQ9(A2);
AnalogIn MQ131(A3);
AnalogIn MQ135(A4);
AnalogIn VOC(A5);

/**
 * A Clock service that demonstrate the GattServer features.
 *
 * The clock service host three characteristics that model the current hour,
 * minute and second of the clock. The value of the second characteristic is
 * incremented automatically by the system.
 *
 * A client can subscribe to updates of the clock characteristics and get
 * notified when one of the value is changed. Clients can also change value of
 * the second, minute and hour characteristric.
 */


class ClockService {
    typedef ClockService Self;

public:
    ClockService() :
        _hour_char("485f4145-52b9-4644-af1f-7a6b9322490f", 0),
        _historic_char("0a924ca7-87cd-4699-a3bd-abdcd9cf126a", 0),
        _current_char("8dd6a1b7-bc75-4741-8a26-264af75807de", 0),
        _clock_service(
            /* uuid */ "51311102-030e-485f-b122-f8f381aa84ed",
            /* characteristics */ _clock_characteristics,
            /* numCharacteristics */ sizeof(_clock_characteristics) /
                                     sizeof(_clock_characteristics[0])
        ),
        _server(NULL),
        _event_queue(NULL)
    {
        // update internal pointers (value, descriptors and characteristics array)
        _clock_characteristics[0] = &_hour_char;
        _clock_characteristics[1] = &_historic_char;
        _clock_characteristics[2] = &_current_char;
        
        // setup authorization handlers
        _hour_char.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        _historic_char.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        _current_char.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        
    }



    void start(BLE &ble_interface, events::EventQueue &event_queue)
    {
         if (_event_queue) {
            return;
        }

        _server = &ble_interface.gattServer();
        _event_queue = &event_queue;

        // register the service
        printf("Adding demo service\r\n");
        ble_error_t err = _server->addService(_clock_service);

        if (err) {
            printf("Error %u during demo service registration.\r\n", err);
            return;
        }

        // read write handler
        _server->onDataSent(as_cb(&Self::when_data_sent));
        _server->onDataWritten(as_cb(&Self::when_data_written));
        _server->onDataRead(as_cb(&Self::when_data_read));

        // updates subscribtion handlers
        _server->onUpdatesEnabled(as_cb(&Self::when_update_enabled));
        _server->onUpdatesDisabled(as_cb(&Self::when_update_disabled));
        _server->onConfirmationReceived(as_cb(&Self::when_confirmation_received));

        // print the handles
        printf("clock service registered\r\n");
        printf("service handle: %u\r\n", _clock_service.getHandle());
        printf("\thour characteristic value handle %u\r\n", _hour_char.getValueHandle());
        printf("\tminute characteristic value handle %u\r\n", _historic_char.getValueHandle());
        printf("\tsecond characteristic value handle %u\r\n", _current_char.getValueHandle());

        _event_queue->call_every(1000 /* ms */, callback(this, &Self::getSensors));
        _event_queue->call_every(2000 /* ms */, callback(this, &Self::store_sensors));
       _event_queue->call_every(100 /* ms */, callback(this, &Self::send_historic));
    }

private:

    /**
     * Handler called when a notification or an indication has been sent.
     */
    void when_data_sent(unsigned count)
    {
        printf("sent %u updates\r\n", count);
    }

    /**
     * Handler called after an attribute has been written.
     */
    void when_data_written(const GattWriteCallbackParams *e)
    {
        printf("data written:\r\n");
        printf("\tconnection handle: %u\r\n", e->connHandle);
        printf("\tattribute handle: %u", e->handle);
        if (e->handle == _hour_char.getValueHandle()) {
            printf(" (hour characteristic)\r\n");
        } else if (e->handle == _historic_char.getValueHandle()) {
            printf(" (minute characteristic)\r\n");
        } else if (e->handle == _current_char.getValueHandle()) {
            printf(" (second characteristic)\r\n");
        } else {
            printf("\r\n");
        }
        printf("\twrite operation: %u\r\n", e->writeOp);
        printf("\toffset: %u\r\n", e->offset);
        printf("\tlength: %u\r\n", e->len);
        printf("\t data: ");

        for (size_t i = 0; i < e->len; ++i) {
            printf("%02X", e->data[i]);
        }

        printf("\r\n");
    }

    /**
     * Handler called after an attribute has been read.
     */
    void when_data_read(const GattReadCallbackParams *e)
    {
        printf("data read:\r\n");
        printf("\tconnection handle: %u\r\n", e->connHandle);
        printf("\tattribute handle: %u", e->handle);
        if (e->handle == _hour_char.getValueHandle()) {
            printf(" (hour characteristic)\r\n");
        } else if (e->handle == _historic_char.getValueHandle()) {
            printf(" (minute characteristic)\r\n");
        } else if (e->handle == _current_char.getValueHandle()) {
            printf(" (second characteristic)\r\n");
        } else {
            printf("\r\n");
        }
    }

    /**
     * Handler called after a client has subscribed to notification or indication.
     *
     * @param handle Handle of the characteristic value affected by the change.
     */
    void when_update_enabled(GattAttribute::Handle_t handle)
    {
        printf("update enabled on handle %d\r\n", handle);
    }

    /**
     * Handler called after a client has cancelled his subscription from
     * notification or indication.
     *
     * @param handle Handle of the characteristic value affected by the change.
     */
    void when_update_disabled(GattAttribute::Handle_t handle)
    {
        printf("update disabled on handle %d\r\n", handle);
    }

    /**
     * Handler called when an indication confirmation has been received.
     *
     * @param handle Handle of the characteristic value that has emitted the
     * indication.
     */
    void when_confirmation_received(GattAttribute::Handle_t handle)
    {
        printf("confirmation received on handle %d\r\n", handle);
    }

    /**
     * Handler called when a write request is received.
     *
     * This handler verify that the value submitted by the client is valid before
     * authorizing the operation.
     */
    void authorize_client_write(GattWriteAuthCallbackParams *e)
    {
        printf("characteristic %u write authorization\r\n", e->handle);

        if (e->offset != 0) {
            printf("Error invalid offset\r\n");
            e->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_INVALID_OFFSET;
            return;
        }

        if (e->len != 1) {
            printf("Error invalid len\r\n");
            e->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_INVALID_ATT_VAL_LENGTH;
            return;
        }

        if ((e->data[0] >= 60) ||
            ((e->data[0] >= 24) && (e->handle == _hour_char.getValueHandle()))) {
            printf("Error invalid data\r\n");
            e->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_WRITE_NOT_PERMITTED;
            return;
        }

        e->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
    }

    /**
     * Increment the second counter.
     */
    void getSensors(void)
    {
        printf("Hello!");
        // 0 index
        sensors[0] = (uint8_t)(row);

        // 1 temperatura zraka
        ht_sensor->get_temperature(&temperature);
        sensors[1] = (uint8_t)(temperature);

        // 2 vlaga zraka
        ht_sensor->get_humidity(&humidity);
        sensors[2] = (uint8_t)(humidity);

        // 3 dim
        sensors[3] = (uint8_t)(MQ135.read()*100);

        // 4 svijetlost
        sensors[4] = 1;

        // 5 vlaga tla
        sensors[5] = (uint8_t)(soil_m.read()*100);

        // 6 voc
        sensors[6] = 1;

        // 7 sumpor
        sensors[7] = (uint8_t)(MQ135.read()*100);

        // 8 air_pressure
        pt_sensor->get_pressure(&air_pressure);
        sensors[8] = 

        // 9 benzene
        sensors[9] = (uint8_t)(MQ135.read()*100);

        // 10 pm25
        sensors[10] = 1;
        
        // 11 monoxide
        sensors[11] = (uint8_t)(MQ9.read()*100);

        // 12 methane
        sensors[12] =(uint8_t)(MQ9.read()*100);

        // 13 lpg
        sensors[13] = (uint8_t)(MQ9.read()*100);

        // 14 nh3
        sensors[14] = (uint8_t)(MQ135.read()*100);

        // 15 dioxide
        sensors[15] = (uint8_t)(MQ135.read()*100);

        // 16 ozone
        sensors[16] = (uint8_t)(MQ131.read()*100);

        
        ble_error_t err = _current_char.set_sensors(*_server, sensors[0]);
        printf("Moist: %.2f\n", (soil_m.read()*240));
        
        if (err) {
            printf("write of the second value returned error %u\r\n", err);
            return;
        }

    }

    /**
     * Increment the minute counter.
     */
    void store_sensors(void)
    {
        row++;
        sensors[0] = row;
        for(int i=0; i<17; i++){
            historic[row][i] = sensors[i];
        }
        if(row>=47){
            row=0;
        }
        //printf("\nSensors stored, row=%d, %d\n\n", historic[row%30][0], row);
        
    }

    /**
     * Increment the hour counter.
     */
    void send_historic(void)
    {
    
        //printf("\nSent: %d\n", historic_row);
        ble_error_t err = _historic_char.set_sensors(*_server, historic[historic_row][0]);
        historic_row++;

        if(historic_row >=47){
            historic_row =0;
        }
        
    }
    

private:
    /**
     * Helper that construct an event handler from a member function of this
     * instance.
     */
    template<typename Arg>
    FunctionPointerWithContext<Arg> as_cb(void (Self::*member)(Arg))
    {
        return makeFunctionPointer(this, member);
    }

    /**
     * Read, Write, Notify, Indicate  Characteristic declaration helper.
     *
     * @tparam T type of data held by the characteristic.
     */
    template<typename T>
    class ReadOnlyArrayGattCharacteristic : public GattCharacteristic {
    public:
        /**
         * Construct a characteristic that can be read or written and emit
         * notification or indication.
         *
         * @param[in] uuid The UUID of the characteristic.
         * @param[in] initial_value Initial value contained by the characteristic.
         */
        ReadOnlyArrayGattCharacteristic(const UUID & uuid, const T& initial_value) :
            GattCharacteristic(
                /* UUID */ uuid,
                /* Initial value */ &_value,
                /* Value size */ sizeof(sensors),
                /* Value capacity */ sizeof(sensors),
                /* Properties */ GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
                                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
                                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY |
                                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE,
                /* Descriptors */ NULL,
                /* Num descriptors */ 0,
                /* variable len */ false
            ),
            _value(initial_value) {
        }
        

        /**
         * Get the value of this characteristic.
         *
         * @param[in] server GattServer instance that contain the characteristic
         * value.
         * @param[in] dst Variable that will receive the characteristic value.
         *
         * @return BLE_ERROR_NONE in case of success or an appropriate error code.
         */
        ble_error_t get(GattServer &server, T& dst) const
        {
            uint16_t value_length = sizeof(dst);
            return server.read(getValueHandle(), &dst, &value_length);
        }

        /**
         * Assign a new value to this characteristic.
         *
         * @param[in] server GattServer instance that will receive the new value.
         * @param[in] value The new value to set.
         * @param[in] local_only Flag that determine if the change should be kept
         * locally or forwarded to subscribed clients.
         */
        ble_error_t set(
            GattServer &server, uint8_t &value, bool local_only = false
        ) const {
            return server.write(getValueHandle(), &value, sizeof(value), local_only);
        }

         ble_error_t set_sensors(
            GattServer &server, uint8_t &value, bool local_only = false
        ) const {
            return server.write(getValueHandle(), sensors, sizeof(sensors), local_only);
        }

    private:
        uint8_t _value;
    };

    ReadOnlyArrayGattCharacteristic<uint8_t> _hour_char;
    ReadOnlyArrayGattCharacteristic<uint8_t> _historic_char;
    ReadOnlyArrayGattCharacteristic<uint8_t> _current_char;
    

    // list of the characteristics of the clock service
    GattCharacteristic* _clock_characteristics[3];

    // demo service
    GattService _clock_service;

    GattServer* _server;
    events::EventQueue *_event_queue;
};

int main() {

    ht_sensor->enable();
    BLE &ble_interface = BLE::Instance();
    events::EventQueue event_queue;
    ClockService demo_service;
    BLEProcess ble_process(event_queue, ble_interface);

    ble_process.on_init(callback(&demo_service, &ClockService::start));

    // bind the event queue to the ble interface, initialize the interface
    // and start advertising
    ble_process.start();

    // Process the event queue.
    event_queue.dispatch_forever();

    return 0;
}
