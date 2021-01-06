#ifndef MHI_AC_Ctrl_h
#define MHI_AC_Ctrl_h

enum class MhiMode
{
    Auto = 0,
    Dry = 1,
    Cool = 2,
    Fan = 3,
    Heat = 4
};

enum class MhiFanSpeed
{
    Low = 1,
    Med = 2,
    High = 3,
    Auto = 4
};

enum class MhiVanes
{
    Down = 1,
    DownMiddle = 2,
    UpMiddle = 3,
    Up = 4,
    Swing = 5,
};

// comment out the data you are not interested, but at least one row must be used.
static const byte opdata[][5]{
        {0xc0, 0x02, 0xff, 0xff, 0xff},  //  1 "MODE"
        {0xc0, 0x05, 0xff, 0xff, 0xff},  //  2 "SET-TEMP"
        {0xc0, 0x80, 0xff, 0xff, 0xff},  //  3 "RETURN-AIR" [°C]
        {0xc0, 0x1f, 0xff, 0xff, 0xff},  //  8 "IU-FANSPEED"
        {0xc0, 0x1e, 0xff, 0xff, 0xff},  // 12 "TOTAL-IU-RUN" [h]
        {0x40, 0x80, 0xff, 0xff, 0xff},  // 21 "OUTDOOR" [°C]
        {0x40, 0x11, 0xff, 0xff, 0xff},  // 24 "COMP" [Hz]
        {0x40, 0x90, 0xff, 0xff, 0xff},  // 29 "CT" [A]
        {0x40, 0xb1, 0xff, 0xff, 0xff},  // 32 "TDSH" [°C]
        {0x40, 0x7c, 0xff, 0xff, 0xff},  // 33 "PROTECTION-No"
        {0x40, 0x1f, 0xff, 0xff, 0xff},  // 34 "OU-FANSPEED"
        {0x40, 0x0c, 0xff, 0xff, 0xff},  // 36 "DEFROST"
        {0x40, 0x1e, 0xff, 0xff, 0xff},  // 37 "TOTAL-COMP-RUN" [h]
        {0x40, 0x13, 0xff, 0xff, 0xff},  // 38 "OU-EEV" [Puls]
};

// The following lines are for program internal purposes and shouldn't be touched
const int opdataCnt = sizeof(opdata) / sizeof(byte) / 5;

// constants for the frame
#define SB0 0
#define SB1 SB0 + 1
#define SB2 SB0 + 2
#define DB0 SB2 + 1
#define DB1 SB2 + 2
#define DB2 SB2 + 3
#define DB3 SB2 + 4
#define DB4 SB2 + 5
#define DB6 SB2 + 7
#define DB9 SB2 + 10
#define DB10 SB2 + 11
#define DB11 SB2 + 12
#define DB12 SB2 + 13
#define DB14 SB2 + 15
#define CBH 18
#define CBL 19

class CallbackInterface
{
public:
    virtual bool cbiMhiEventHandlerFunction(const char* key, const char* value) = 0;
};

class MHIAcCtrl {
public:
    MHIAcCtrl(CallbackInterface *cb) {
            m_cb = cb;
    };
    void loop();
    bool powerOn();
    bool powerOff();
    bool setMode(MhiMode mode);
    bool tSetpoint(byte _tSetPoint);
    bool setFanspeed(MhiFanSpeed fanSpeed);
    bool setVanes(MhiVanes vanes);
    bool getErrOpData();

private:
    // interfaces:
    void evalOpMode(char mqtt_msg[], byte db10);
    void update_sync(bool sync_new);
    uint16_t calc_tx_checksum();

    // Member variables
    CallbackInterface *m_cb;
    char strtmp[10];
    // old status:
    uint8_t fan_old = 0xff;
    uint8_t vanes_old = 0xff;
    uint8_t power_old = 0xff;
    uint8_t mode_old = 0xff;
    uint8_t errorcode_old = 0xff;
    byte troom_old = 0xff;
    byte tsetpoint_old = 0xff;
    // old operating data
    byte op_settemp_old = 0xff;
    byte op_mode_old = 0xff;
    uint16_t op_ou_eev_old = 0xffff;
    byte op_total_iu_run_old = 0;
    byte op_iu_fanspeed_old = 0xff;
    byte op_ou_fanspeed_old = 0xff;
    byte op_outdoor_old = 0xff;
    byte op_return_air_old = 0xff;
    byte op_total_comp_run_old = 0;
    byte op_protection_no_old = 0xff;
    byte op_defrost_old = 0x00;
    uint16_t op_comp_old = 0xffff;
    byte op_tdsh_old = 0xff;
    byte op_ct_old = 0xff;

    unsigned long lastDatapacketMillis = 0;
    unsigned long lastTroomMillis = 0;
    unsigned long SCKMillis;
    bool sync = 0;
    uint8_t doubleframe = 1;
    uint8_t frame = 1;
    uint8_t erropdataCnt = 0;
    uint8_t opdataNo = 0;
    bool set_Power = false;
    byte new_Power;
    bool set_Mode = false;
    byte new_Mode;
    bool set_Tsetpoint = false;
    byte new_Tsetpoint;
    bool set_Fan = false;
    byte new_Fan;
    bool set_Vanes = false;
    byte new_Vanes;
    bool request_erropData = false;
    bool new_datapacket_received = false;
    uint16_t packet_cnt = 0;
    uint8_t repetitionNo = 0;
    byte payload_byte;  // received MOSI byte
    byte rx_SPIframe[20];
    bool valid_datapacket_received = false;
};



#endif  // MHI_AC_Ctrl_h
