#include <Arduino.h>

// The Arduino standard GPIO routines are not enough,
// must use some from the Espressif SDK as well
extern "C" {
#include "gpio.h"
}

#include "MHI-AC-Ctrl.h"

//                       sb0   sb1   sb2   db0   db1   db2   db3   db4   db5   db6   db7   db8   db9  db10  db11  db12  db13  db14  chkH  chkL
byte tx_SPIframe[20] = {0xA9, 0x00, 0x07, 0x50, 0x10, 0x2e, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x05, 0xf5};

uint updateMQTTStatus_opdata = opdataCnt;

void MHIAcCtrl::evalOpMode(char mqtt_msg[], byte db10) {
    switch (db10 & 0x1f) {  // 0x10=Auto, 0x11=Dry, 0x12=Cold, 0x13=Fan, 0x14=heat
        case 0x10:
            if ((db10 & 0x30) == 0x30)
                strcpy(mqtt_msg, "Stop"); // for error operating data
            else
                strcpy(mqtt_msg, "Auto"); // for operating data
            break;
        case 0x11:
            strcpy(mqtt_msg, "Dry");
            break;
        case 0x12:
            strcpy(mqtt_msg, "Cool");
            break;
        case 0x13:
            strcpy(mqtt_msg, "Fan");
            break;
        case 0x14:
            strcpy_P(mqtt_msg, PSTR("Heat"));
            break;
    }
}

void MHIAcCtrl::update_sync(bool sync_new) {
    if (sync_new != sync) {
        sync = sync_new;
        Serial.printf("MHI sync=%i\r\n", sync);
        if (sync)
            m_cb->cbiMhiEventHandlerFunction(PSTR("synced"), PSTR("1"));
        else
            m_cb->cbiMhiEventHandlerFunction(PSTR("synced"), PSTR("0"));
    }
}

uint16_t MHIAcCtrl::calc_tx_checksum() {
    uint16_t checksum = 0;
    for (int i = 0; i < CBH; i++)
        checksum += tx_SPIframe[i];
    return checksum;
}

void MHIAcCtrl::loop() {
    SCKMillis = millis();
    while (millis() - SCKMillis < 5) { // wait for 5ms stable high signal
        delay(0);
        if (!digitalRead(SCK)) {
            SCKMillis = millis();
            if (SCKMillis - lastDatapacketMillis > 200) {
                update_sync(false);
                Serial.println("Error: No SPI Clock (SCK) detected!");
            }
        }
    }

    if (frame++ > 19) { // setup tx frame
        doubleframe++;  // toggle between 0 and 1
        tx_SPIframe[DB14] = (doubleframe % 2) << 2;
        frame = 1;
        if (doubleframe % 2) {
            if (erropdataCnt == 0) {
                tx_SPIframe[DB0] = 0x00;
                tx_SPIframe[DB1] = 0x00;
                tx_SPIframe[DB2] = 0x00;
                tx_SPIframe[DB6] = opdata[opdataNo][0];
                tx_SPIframe[DB9] = opdata[opdataNo][1];
                tx_SPIframe[DB10] = opdata[opdataNo][2];
                tx_SPIframe[DB11] = opdata[opdataNo][3];
                tx_SPIframe[DB12] = opdata[opdataNo][4];
                opdataNo = (opdataNo + 1) % opdataCnt;
            } else { // error operating data available
                tx_SPIframe[DB6] = 0x80;
                tx_SPIframe[DB9] = 0xff;
                tx_SPIframe[DB10] = 0xff;
                tx_SPIframe[DB11] = 0xff;
                tx_SPIframe[DB12] = 0xff;
                erropdataCnt--;
            }
            if (set_Power) {
                tx_SPIframe[DB0] = new_Power;
                set_Power = false;
            }

            if (set_Mode) {
                tx_SPIframe[DB0] = new_Mode;
                set_Mode = false;
            }

            if (set_Tsetpoint) {
                tx_SPIframe[DB2] = 2 * new_Tsetpoint | 0b10000000;
                set_Tsetpoint = false;
            }

            if (set_Fan) {
                if (new_Fan == 4) {
                    tx_SPIframe[DB1] = 0b1010;
                    tx_SPIframe[DB6] |= 0b00010000;
                } else {
                    tx_SPIframe[DB1] = (1 << 3) | (new_Fan - 1);
                }
                set_Fan = false;
            }

            if (set_Vanes) {
                if (new_Vanes == 5) { //5: swing
                    tx_SPIframe[DB0] = 0b11000000;
                } else {
                    tx_SPIframe[DB0] = 0b10000000; // disable swing
                    tx_SPIframe[DB1] = (1 << 7) | ((new_Vanes - 1) << 4);
                }
                set_Vanes = false;
            }

            if (request_erropData) {
                tx_SPIframe[DB6] = 0x80;
                tx_SPIframe[DB9] = 0x45;
                request_erropData = false;
            }
        }
    }

    uint16_t tx_checksum = this->calc_tx_checksum();
    tx_SPIframe[CBH] = highByte(tx_checksum);
    tx_SPIframe[CBL] = lowByte(tx_checksum);

    new_datapacket_received = false;
    uint16_t rx_checksum = 0;
    for (uint8_t byte_cnt = 0; byte_cnt < 20; byte_cnt++) { // read and write a data packet of 20 bytes
        payload_byte = 0;
        byte bit_mask = 1;
        for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) { // read and write 1 byte
            while (digitalRead(SCK)) {} // wait for falling edge
            if ((tx_SPIframe[byte_cnt] & bit_mask) > 0)
                digitalWrite(MISO, 1);
            else
                digitalWrite(MISO, 0);
            while (!digitalRead(SCK)) {} // wait for rising edge
            if (digitalRead(MOSI))
                payload_byte += bit_mask;
            bit_mask = bit_mask << 1;
        }

        if (rx_SPIframe[byte_cnt] != payload_byte) {
            new_datapacket_received = true;
            rx_SPIframe[byte_cnt] = payload_byte;
        }

        if (byte_cnt < 18)
            rx_checksum += payload_byte;
    }
    if (((rx_SPIframe[SB0] & 0xfe) == 0x6c) & (rx_SPIframe[SB1] == 0x80) & (rx_SPIframe[SB2] == 0x04) &
        ((rx_SPIframe[CBH] << 8 | rx_SPIframe[CBL]) == rx_checksum)) {
        valid_datapacket_received = true;
    } else {
        update_sync(false);
        Serial.printf("Wrong MOSI signature: 0x%.2X 0x%.2X 0x%.2X or checksum received!\r\n", rx_SPIframe[SB0],
                      rx_SPIframe[SB1], rx_SPIframe[SB2]);
        for (uint8_t i = 0; i < 19; i++) {
            Serial.printf("%.2X ", rx_SPIframe[i]);
        }
        Serial.printf("\r\n");
    }

    if (valid_datapacket_received) { // valid MOSI frame received
        packet_cnt++;
        repetitionNo++;
        valid_datapacket_received = false;
        if (millis() - lastDatapacketMillis < 60)
            update_sync(true);
        lastDatapacketMillis = millis();

        if (new_datapacket_received) {
            new_datapacket_received = false;            
            Serial.println("Valid datapacket received!"); // added by Dan
            bool updateMQTTStatus = false;
            if (updateMQTTStatus | ((rx_SPIframe[DB0] & 0x01) != power_old)) { // Power
                power_old = rx_SPIframe[DB0] & 0x01;
                if (power_old == 0) {
                    m_cb->cbiMhiEventHandlerFunction(PSTR("Power"), PSTR("Off"));
                    Serial.println("Power: OFF");
                }
                else {
                    m_cb->cbiMhiEventHandlerFunction(PSTR("Power"), PSTR("On"));
                    Serial.println("Power: ON");
                }
            }

            if (updateMQTTStatus | ((rx_SPIframe[DB0] & 0x1c) != mode_old)) { // Mode
                mode_old = rx_SPIframe[DB0] & 0x1c;
                switch (mode_old) {
                    case 0x00:
                        m_cb->cbiMhiEventHandlerFunction(PSTR("Mode"), PSTR("Auto"));
                        break;
                    case 0x04:
                        m_cb->cbiMhiEventHandlerFunction(PSTR("Mode"), PSTR("Dry"));
                        break;
                    case 0x08:
                        m_cb->cbiMhiEventHandlerFunction(PSTR("Mode"), PSTR("Cool"));
                        break;
                    case 0x0c:
                        m_cb->cbiMhiEventHandlerFunction(PSTR("Mode"), PSTR("Fan"));
                        break;
                    case 0x10:
                        m_cb->cbiMhiEventHandlerFunction(PSTR("Mode"), PSTR("Heat"));
                        break;
                    default:
                        m_cb->cbiMhiEventHandlerFunction(PSTR("Mode"), PSTR("invalid"));
                        break;
                }
            }

            uint fantmp;
            if ((rx_SPIframe[DB6] & 0x40) != 0) // Fan status
                fantmp = 4;
            else
                fantmp = (rx_SPIframe[DB1] & 0x03) + 1;
            if (updateMQTTStatus | (fantmp != fan_old)) {
                fan_old = fantmp;
                itoa(fan_old, strtmp, 10);
                m_cb->cbiMhiEventHandlerFunction(PSTR("Fan"), strtmp);
            }

            // Only updated when Vanes command via wired RC
            uint vanestmp = (rx_SPIframe[DB0] & 0xc0) + ((rx_SPIframe[DB1] & 0xB0) >> 4);
            if (updateMQTTStatus | (vanestmp != vanes_old)) {
                if ((vanestmp & 0x88) == 0) // last vanes update was via IR-RC, so status is not known
                    m_cb->cbiMhiEventHandlerFunction(PSTR("Vanes"), PSTR("?"));
                else if ((vanestmp & 0x40) != 0) // Vanes status swing
                    m_cb->cbiMhiEventHandlerFunction(PSTR("Vanes"), PSTR("Swing"));
                else {
                    switch (vanestmp & 0x03) {
                        case 0x00:
                            m_cb->cbiMhiEventHandlerFunction(PSTR("Vanes"), PSTR("1")); // up
                            break;
                        case 0x01:
                            m_cb->cbiMhiEventHandlerFunction(PSTR("Vanes"), PSTR("2"));
                            break;
                        case 0x02:
                            m_cb->cbiMhiEventHandlerFunction(PSTR("Vanes"), PSTR("3"));
                            break;
                        case 0x03:
                            m_cb->cbiMhiEventHandlerFunction(PSTR("Vanes"), PSTR("4")); // down
                            break;
                    }
                }
                vanes_old = vanestmp;
            }

            int8_t troom_diff = rx_SPIframe[DB3] - troom_old; // avoid using other functions inside the brackets of abs, see https://www.arduino.cc/reference/en/language/functions/math/abs/
            if (updateMQTTStatus | (abs(troom_diff) > 1 & lastDatapacketMillis - lastTroomMillis >
                                                          30000)) { // Room temperature delta > 0.25°C
                lastTroomMillis = lastDatapacketMillis;
                troom_old = rx_SPIframe[DB3];
                float troom = (float) (troom_old - 61) / 4.0;
                dtostrf(troom, 0, 2, strtmp);
                m_cb->cbiMhiEventHandlerFunction(PSTR("Troom"), strtmp);
            }

            if (updateMQTTStatus | ((rx_SPIframe[DB2] & 0x7f) >> 1 != tsetpoint_old)) { // Temperature setpoint
                tsetpoint_old = (rx_SPIframe[DB2] & 0x7f) >> 1;
                itoa(tsetpoint_old, strtmp, 10);
                m_cb->cbiMhiEventHandlerFunction(PSTR("Tsetpoint"), strtmp);
            }

            if (updateMQTTStatus | (rx_SPIframe[DB4] != errorcode_old)) { // error code
                errorcode_old = rx_SPIframe[DB4];
                itoa(errorcode_old, strtmp, 10);
                m_cb->cbiMhiEventHandlerFunction(PSTR("Errorcode"), strtmp);
            }

            // section below added by Dan to print RX contents to serial for dev testing
            //for(int i = 0; i < sizeof(rx_SPIframe); i++)
            //{
            //  Serial.println(rx_SPIframe[i]);
            //}
            
            // Operating Data
            switch (rx_SPIframe[DB9]) {
                case 0x02: // 1 MODE
                    if ((rx_SPIframe[DB6] & 0x80) != 0) {
                        if ((rx_SPIframe[DB10] | 0x30) == 0x30) {
                            evalOpMode(strtmp, rx_SPIframe[DB10]);
                            m_cb->cbiMhiEventHandlerFunction(PSTR("ERR-Mode"), strtmp);
                        } else if ((rx_SPIframe[DB10] != op_mode_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_mode_old = rx_SPIframe[DB10];
                            evalOpMode(strtmp, rx_SPIframe[DB10]);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("Mode"), strtmp))
                                op_mode_old = 0xff;
                        }
                    }
                    break;
                case 0x05: // 1 SET-TEMP
                    if ((rx_SPIframe[DB10] == 0x13) & ((rx_SPIframe[DB6] & 0x80) != 0)) {
                        if ((rx_SPIframe[DB11] != op_settemp_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_settemp_old = rx_SPIframe[DB11];
                            itoa(rx_SPIframe[DB11] >> 1, strtmp, 10);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("SET-TEMP"), strtmp))
                                op_settemp_old = 0xff;
                        }
                    } else if ((rx_SPIframe[DB10] == 0x33) & ((rx_SPIframe[DB6] & 0x80) != 0)) { // last error data
                        itoa(rx_SPIframe[DB11] >> 1, strtmp, 10);
                        m_cb->cbiMhiEventHandlerFunction(PSTR("ERR-SET-TEMP"), strtmp);
                    }
                    break;
                case 0x80: // 3 RETURN-AIR or 21 OUTDOOR
                    if ((rx_SPIframe[DB10] == 0x20) & ((rx_SPIframe[DB6] & 0x80) != 0)) { // 3 RETURN-AIR
                        if ((rx_SPIframe[DB11] != op_return_air_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_return_air_old = rx_SPIframe[DB11];
                            dtostrf(rx_SPIframe[DB11] * 0.25f - 15, 0, 2, strtmp);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("RETURN-AIR"), strtmp))
                                op_return_air_old = 0xff;
                        }
                    } else if ((rx_SPIframe[DB10] == 0x30) &
                               ((rx_SPIframe[DB6] & 0x80) != 0)) { // 3 RETURN-AIR last error data
                        dtostrf(rx_SPIframe[DB11] * 0.25f - 15, 0, 2, strtmp);
                        m_cb->cbiMhiEventHandlerFunction(PSTR("ERR-RETURN-AIR"), strtmp);
                    } else if ((rx_SPIframe[DB10] == 0x10) & ((rx_SPIframe[DB6] & 0x80) == 0)) { // 21 OUTDOOR
                        if ((rx_SPIframe[DB11] != op_outdoor_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_outdoor_old = rx_SPIframe[DB11];
                            dtostrf((rx_SPIframe[DB11] - 94) * 0.25f, 0, 2, strtmp);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("OUTDOOR"), strtmp))
                                op_outdoor_old = 0xff;
                        }
                    }
                    break;
                case 0x1f: // 8 IU-FANSPEED or 34 OU-FANSPEED
                    if ((rx_SPIframe[DB6] & 0x80) != 0) {
                        if ((rx_SPIframe[DB10] & 0x30) == 0x10) { // 8 IU-FANSPEED
                            if ((rx_SPIframe[DB10] != op_iu_fanspeed_old) | (updateMQTTStatus_opdata > 0)) {
                                if (updateMQTTStatus_opdata > 0)
                                    updateMQTTStatus_opdata--;
                                op_iu_fanspeed_old = rx_SPIframe[DB10];
                                itoa(rx_SPIframe[DB10] & 0x0f, strtmp, 10);
                                if (!m_cb->cbiMhiEventHandlerFunction(PSTR("IU-FANSPEED"), strtmp))
                                    op_iu_fanspeed_old = 0xff;
                            }
                        } else if ((rx_SPIframe[DB10] & 0x30) == 0x30) { // 8 IU-FANSPEED last error data
                            itoa(rx_SPIframe[DB10] & 0x0f, strtmp, 10);
                            m_cb->cbiMhiEventHandlerFunction(PSTR("ERR-IU-FANSPEED"), strtmp);
                        }
                    } else { // 34 OU-FANSPEED
                        if ((rx_SPIframe[DB10] != op_ou_fanspeed_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_ou_fanspeed_old = rx_SPIframe[DB10];
                            itoa(op_ou_fanspeed_old & 0x0f, strtmp, 10);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("OU-FANSPEED"), strtmp))
                                op_ou_fanspeed_old = 0xff;
                        }
                    }
                    break;
                case 0x1e: // 12 TOTAL-IU-RUN or 37 TOTAL-COMP-RUN
                    if ((rx_SPIframe[DB10] == 0x10) & ((rx_SPIframe[DB6] & 0x80) != 0)) { // 12 TOTAL-IU-RUN
                        if ((rx_SPIframe[DB11] != op_total_iu_run_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_total_iu_run_old = rx_SPIframe[DB11];
                            itoa(rx_SPIframe[DB11] * 100, strtmp, 10);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("TOTAL-IU-RUN"), strtmp))
                                op_total_iu_run_old = 0xff;
                        }
                    } else if ((rx_SPIframe[DB10] == 0x30) &
                               ((rx_SPIframe[DB6] & 0x80) != 0)) { // 12 TOTAL-IU-RUN last error data
                        itoa(rx_SPIframe[DB11] * 100, strtmp, 10);
                        m_cb->cbiMhiEventHandlerFunction(PSTR("TOTAL-IU-RUN"), strtmp);
                    } else if ((rx_SPIframe[DB10] == 0x11) & ((rx_SPIframe[DB6] & 0x80) == 0)) { // 37 TOTAL-COMP-RUN
                        if ((rx_SPIframe[DB11] != op_total_comp_run_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_total_comp_run_old = rx_SPIframe[DB11];
                            itoa(rx_SPIframe[DB11] * 100, strtmp, 10);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("TOTAL-COMP-RUN"), strtmp))
                                op_total_comp_run_old = 0xff;
                        }
                    }
                    break;
                case 0x11: // 24 COMP
                    if ((rx_SPIframe[DB6] & 0x80) == 0) {
                        if (((rx_SPIframe[DB11] << 8 | rx_SPIframe[DB10]) != op_comp_old) |
                            (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_comp_old = rx_SPIframe[DB11] << 8 | rx_SPIframe[DB10];
                            dtostrf((rx_SPIframe[DB10] - 0x10) * 25.6f + 0.1f * rx_SPIframe[DB11], 0, 2, strtmp);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("COMP"), strtmp))
                                op_comp_old = 0xffff;
                        }
                    }
                    break;
                case 0x90: // 29 CT
                    if ((rx_SPIframe[DB10] == 0x10) & ((rx_SPIframe[DB6] & 0x80) == 0)) {
                        if ((rx_SPIframe[DB11] != op_protection_no_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_protection_no_old = rx_SPIframe[DB11];
                            itoa(rx_SPIframe[DB11], strtmp, 10);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("CT"), strtmp))
                                op_protection_no_old = 0xff;
                        }
                    }
                    break;
                case 0xb1: // 32 TDSH
                    if ((rx_SPIframe[DB10] == 0x10) & ((rx_SPIframe[DB6] & 0x80) == 0)) {
                        if ((rx_SPIframe[DB11] != op_tdsh_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_tdsh_old = rx_SPIframe[DB11];
                            itoa(rx_SPIframe[DB11] / 2, strtmp, 10);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("TDSH"), strtmp))
                                op_tdsh_old = 0xff;
                        }
                    }
                    break;
                case 0x7c: // 33 PROTECTION-No
                    if ((rx_SPIframe[DB10] == 0x10) & ((rx_SPIframe[DB6] & 0x80) == 0)) {
                        if ((rx_SPIframe[DB11] != op_ct_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_ct_old = rx_SPIframe[DB11];
                            itoa(rx_SPIframe[DB11], strtmp, 10);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("PROTECTION-No"), strtmp))
                                op_ct_old = 0xff;
                        }
                    }
                    break;
                case 0x0c: // 36 DEFROST
                    if ((rx_SPIframe[DB6] & 0x80) == 0) {
                        if ((rx_SPIframe[DB10] != op_defrost_old) | (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_defrost_old = rx_SPIframe[DB10];
                            if (rx_SPIframe[DB10] = 0x10)
                                strcpy(strtmp, "Off");
                            else  //0x11
                                strcpy(strtmp, "On");
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("DEFROST"), strtmp))
                                op_defrost_old = 0x00;
                        }
                    }
                    break;
                case 0x13: // 38 OU-EEV
                    if ((rx_SPIframe[DB10] == 0x10) & ((rx_SPIframe[DB6] & 0x80) == 0)) { // 38 OU-EEV
                        if (((rx_SPIframe[DB12] << 8 | rx_SPIframe[DB11]) != op_ou_eev_old) |
                            (updateMQTTStatus_opdata > 0)) {
                            if (updateMQTTStatus_opdata > 0)
                                updateMQTTStatus_opdata--;
                            op_ou_eev_old = rx_SPIframe[DB12] << 8 | rx_SPIframe[DB11];
                            itoa(op_ou_eev_old, strtmp, 10);
                            if (!m_cb->cbiMhiEventHandlerFunction(PSTR("OU-EEV"), strtmp))
                                op_ou_eev_old = 0xffff;
                        }
                    }
                    break;
                    // related to last error operating data
                case 0x45: // last error number or count of following error operating data
                    if ((rx_SPIframe[DB10] == 0x11) & ((rx_SPIframe[DB6] & 0x80) != 0)) { // last error number
                        itoa(rx_SPIframe[DB11], strtmp, 10);
                        m_cb->cbiMhiEventHandlerFunction(PSTR("Errorcode"), strtmp);
                    } else if ((rx_SPIframe[DB10] == 0x12) &
                               ((rx_SPIframe[DB6] & 0x80) != 0)) { // count of following error operating data
                        erropdataCnt = rx_SPIframe[DB11] + 4;
                    }
                    break;
            }


            updateMQTTStatus = false;
            repetitionNo = 0;
        } // if(new_datapacket_received)
    } // if(valid_datapacket_received)

}

bool MHIAcCtrl::powerOn() {
    new_Power = rx_SPIframe[DB0] | 0b11;
    set_Power = true;
    new_Power = 0b11;
    return true;
}

bool MHIAcCtrl::powerOff() {
    new_Power = rx_SPIframe[DB0] | 0b11;
    set_Power = true;
    new_Power = 0b10;
    return true;
}

bool MHIAcCtrl::setMode(MhiMode mode) {
    switch (mode) {
        case MhiMode::Auto  :
            new_Mode = 0b00100000;
            break;
        case MhiMode::Dry:;
            new_Mode = 0b00100100;
            break;
        case MhiMode::Cool :;
            new_Mode = 0b00101000;
            break;
        case MhiMode::Fan :;
            new_Mode = 0b00101100;
            break;
        case MhiMode::Heat :;
            new_Mode = 0b00110000;
            break;
        default:
            return false;
    }
    set_Mode = true;
    return true;
}

bool MHIAcCtrl::tSetpoint(byte _tSetPoint) {
    new_Tsetpoint = _tSetPoint;
    if ((new_Tsetpoint >= 18) & (new_Tsetpoint <= 30)) {
        set_Tsetpoint = true;
        return true;
    }
    return false;
}

bool MHIAcCtrl::setFanspeed(MhiFanSpeed fanSpeed) {
    new_Fan = static_cast<char>(fanSpeed);
    if ((new_Fan >= 1) & (new_Fan <= 4)) {
        set_Fan = true;
        return true;
    }
    return false;
}

bool MHIAcCtrl::setVanes(MhiVanes vanes) {
    new_Vanes = static_cast<char>(vanes);
    if ((new_Vanes >= 1) & (new_Vanes <= 5)) {
        set_Vanes = true;
        return true;
    }
    return false;
}

bool MHIAcCtrl::getErrOpData() {
    request_erropData = true;
    return true;
}
