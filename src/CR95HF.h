#include "SPI.h"

// CR95HF Commands Definition
#define    IDN                    0x01
#define    ProtocolSelect         0x02
#define    SendRecv               0x04
#define    Idle                   0x07
#define    RdReg                  0x08
#define    WrReg                  0x09
#define    BaudRate               0x0A
#define    ECHO                   0x55

class CR95HF {

  public:
    CR95HF(int cs, int irq_in = -1, int ssi_o = -1, int ssi_i = -1) : CS(cs), IRQ_IN(irq_in), SSI_0(ssi_o), SSI_1(ssi_i) {}
    void begin();
    char* readSerial();
    String getID();

  private:
    void writeCmd(unsigned short cmd, unsigned short dataLen);
    void readCmd();
    char EchoResponse();
    void Calibration();
    void GetNFCTag();
    void GetTagID();
    void AutoFDet();
    void IndexMod_Gain();
    void Select_ISO_IEC_18092_Protocol();
    void Select_ISO_IEC_14443_A_Protocol();



  private:

    const SPISettings SETTINGS = SPISettings(1000000, MSBFIRST, SPI_MODE0);
    unsigned short sdata[18];
    unsigned short rdata[18];
    unsigned short res = 0, dataNum = 0;
    unsigned short j = 0, tmp = 0;
    int x_pos = 0;
    int x_pos_old = 0;
    char CR95HF_ID[13];
    String ID;
    String ID_old;
    char flag = 0;
    unsigned char NFC_flag = 0;
    unsigned char TAG_flag = 1;
    const unsigned int SSI_0; // 3
    const unsigned int SSI_1; //5
    const unsigned int IRQ_IN; //2
    const unsigned int CS;  //4
};
