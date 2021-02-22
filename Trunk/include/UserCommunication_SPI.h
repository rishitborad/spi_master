static void _UserCommSpiDMADoneCallback();

uint8_t InitCommunication_UserSPI( void );

uint8_t GetVar_NbrOfData( void );

void _InitNSSInterrupt( FunctionalState enable );

/* User SPI Data Structure */
typedef struct {
    uint8_t firstByteFlag;
    uint8_t index;

    uint8_t invalidCommand;  // this will go high if a command if invalid;
                             //   prevents the function from outputting data
    uint8_t writeError;  // this will go high if an error occurs (error? response?)

    uint8_t writeDataFlag;
    uint8_t registerAddr;

    // Set the number of bytes to send based on the selected register
    uint8_t NumberOfBytes;

    // Create an array of bytes that is sufficiently long to hold any register
    //   length (only 'NumberOfBytes' will be output to the master, after that,
    //   0xFF will be output)
    uint8_t TxBuffer[ 50 ];

    uint8_t invalidRegister;
} UserSpiStruct;

extern UserSpiStruct gUserSpi;
