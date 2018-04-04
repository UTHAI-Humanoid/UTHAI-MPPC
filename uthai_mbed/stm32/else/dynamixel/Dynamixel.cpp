#include <mbed.h>
#include <Dynamixel.h>

uint8_t Instruction_Packet[15];     // Array to hold instruction packet data
uint8_t Status_Packet[8];           // Array to hold returned status packet data
uint8_t Status_Return_Value = READ; // Status package return states (NON, READ, ALL)

Timer timer;
//-------------------------------------------------------------------------------------------------------------------------------
//            Private Methods
//-------------------------------------------------------------------------------------------------------------------------------
void Dynamixel::transmitInstructionPacket(void)
{
    if (dynamixelSerial->readable())
        while (dynamixelSerial->readable())
            dynamixelSerial->getc();

    dynamixelDi->write(1);
    dynamixelSerial->putc(HEADER);
    dynamixelSerial->putc(HEADER);
    dynamixelSerial->putc(Instruction_Packet[0]);
    dynamixelSerial->putc(Instruction_Packet[1]);
    for (uint8_t i = 0; i < Instruction_Packet[1]; i++)
    {
        dynamixelSerial->putc(Instruction_Packet[2 + i]);
    }
    wait_us((Instruction_Packet[1] + 4) * 5);
    dynamixelDi->write(0);
}

unsigned int Dynamixel::readStatusPacket(void)
{
    wait_us(250);
    uint8_t Counter = 0;
    uint8_t InBuff[20];
    uint8_t i = 0, j = 0, RxState = 0;
    Status_Packet[0] = 0;
    Status_Packet[1] = 0;
    Status_Packet[2] = 0;
    Status_Packet[3] = 0;
    timer.start();
    int bytes = 2;
    int timeout = 0;
    int plen = 0;
    // while (timer.read_ms() < 3000)
    // {
    //     if (dynamixelSerial->readable())
    //     {
    //         InBuff[plen] = dynamixelSerial->getc();
    //         plen++;
    //     }
    // }
    while ((timeout < ((6 + bytes) * 10000)) && (plen < (6 + bytes)))
    {

        if (dynamixelSerial->readable())
        {
            InBuff[plen] = dynamixelSerial->getc();
            plen++;
            timeout = 0;
        }

        // wait for the bit period
        wait_us(1);
        timeout++;
    }
    timer.stop();
    for (int i = 0; i < plen; i++)
    {
        Status_Packet[i] = InBuff[i];
        printf("0x%X,", Status_Packet[i]);
    }
    printf("\r\n");
    return 0x01;
}
//-------------------------------------------------------------------------------------------------------------------------------
// Public Methods
//-------------------------------------------------------------------------------------------------------------------------------
Dynamixel::Dynamixel(PinName tx, PinName rx, int baud, PinName di)
{
    dynamixelSerial = new Serial(tx, rx);
    dynamixelSerial->baud(baud);
    dynamixelDi = new DigitalOut(di);
    dynamixelDi->write(0);
}

Dynamixel::~Dynamixel(void)
{
    if (dynamixelSerial != NULL)
        delete dynamixelSerial;
}

unsigned int Dynamixel::ping(uint8_t ID)
{
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = PING_LENGTH;
    Instruction_Packet[2] = COMMAND_PING;
    Instruction_Packet[3] = ~(ID + PING_LENGTH + COMMAND_PING);
    transmitInstructionPacket();
    // if ((ID == 0xFE) || (Status_Return_Value != ALL))
    //     return 0x00;
    // else
    // {
    readStatusPacket();
    return 0x01;
    // if (Status_Packet[2] == 0)
    // return (Status_Packet[0]);
    // else
    // return (Status_Packet[2] | 0xF000);
    // }
}
unsigned int Dynamixel::getTemperature(uint8_t ID)
{
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = READ_TEMP_LENGTH;
    Instruction_Packet[2] = COMMAND_READ_DATA;
    Instruction_Packet[3] = RAM_PRESENT_TEMPERATURE;
    Instruction_Packet[4] = READ_ONE_BYTE_LENGTH;
    Instruction_Packet[5] = ~(ID + READ_TEMP_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_TEMPERATURE + READ_ONE_BYTE_LENGTH);

    transmitInstructionPacket();
    return readStatusPacket();
    // return Instruction_Packet[5];
    /*
    if (Status_Packet[2] == 0)
    { // If there is no status packet error return value
        // return Status_Packet[3];
        return 0;
    }
    else
    {
        return 1;
        // return (Status_Packet[2] | 0xF000); // If there is a error Returns error value
    }*/
}
void Dynamixel::setPosition(uint8_t ID, unsigned int Position, unsigned int Speed = 0)
{
    uint8_t L_Position = (uint8_t)(Position & 0xFF);
    uint8_t H_Position = (uint8_t)(Position >> 8);
    uint8_t L_Speed = (uint8_t)(Speed & 0xFF);
    uint8_t H_Speed = (uint8_t)(Speed >> 8);
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = SERVO_GOAL_LENGTH;
    Instruction_Packet[2] = COMMAND_WRITE_DATA;
    Instruction_Packet[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet[4] = L_Position;
    Instruction_Packet[5] = H_Position;
    Instruction_Packet[6] = L_Speed;
    Instruction_Packet[7] = H_Speed;
    Instruction_Packet[8] = ~(ID + SERVO_GOAL_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_POSITION_L + L_Position + H_Position + L_Speed + H_Speed);
    transmitInstructionPacket();
}
