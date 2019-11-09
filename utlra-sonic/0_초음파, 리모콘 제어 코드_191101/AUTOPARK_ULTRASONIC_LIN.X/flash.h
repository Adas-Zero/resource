#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS  64
#define FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS 512

#define FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS  (FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS*2)
#define FLASH_WRITE_ROW_SIZE_IN_PC_UNITS (FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS*2)
#define FLASH_HAS_ECC  1

#define INSTRUCTION_IN_PC_UNITS (2) /* 1 instruction (24 bits) occupies 2 addresses */
#define FLASH_PAGE_ROUND_UP(x) ( (((uint32_t)(x)) + FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS-1)  & (~(FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS-1)) ) 
#define FLASH_MEM_ADDRESS   (0x200LU)
#define MAX_FIRMWARE_SIZE   (512LU*70LU) // 70 pages of instructions (70*512*3=105KB)
#define FLASH_APP_CONFIG_ADDRESS FLASH_PAGE_ROUND_UP(FLASH_MEM_ADDRESS + INSTRUCTION_IN_PC_UNITS*MAX_FIRMWARE_SIZE)

#define FLASH_UNLOCK_KEY 0x00AA0055

#define PAGE_MASK (~((FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS*2) - 1)) 
void     FLASH_Unlock(uint32_t  key);
void     FLASH_Lock(void);

bool     FLASH_ErasePage(uint32_t address);    

uint16_t FLASH_ReadWord16(uint32_t address);
uint32_t FLASH_ReadWord24(uint32_t address);

bool     FLASH_WriteDoubleWord16(uint32_t flashAddress, uint16_t Data0, uint16_t Data1);
bool     FLASH_WriteDoubleWord24(uint32_t address, uint32_t Data0, uint32_t Data1  );

/* Program the flash one row at a time. */

/* FLASH_WriteRow24: Writes a single row of data from the location given in *data to
 *                   the flash location in address.  Since the flash is only 24 bits wide
 *                   all data in the upper 8 bits of the source will be lost .  
 *                   The address in *data must be row aligned.
 *                   returns true if successful */

bool     FLASH_WriteRow24(uint32_t flashAddress, uint32_t *data);

/* FLASH_WriteRow16: Writes a single row of data from the location in given in *data to
 *                   to the flash location in address. Each 16 bit source data 
 *                   word is stored in the lower 16 bits of each flash entry and the 
 *                   upper 8 bits of the flash is not programmed. 
 *                   The address in *data must be row aligned.
 *                   returns true if successful */
bool     FLASH_WriteRow16(uint32_t address, uint16_t *data);


uint16_t FLASH_GetErasePageOffset(uint32_t address);
uint32_t FLASH_GetErasePageAddress(uint32_t address);

// Allocate and reserve a page of flash for this test to use.  The compiler/linker will reserve this for data and not place any code here.
static __prog__  uint8_t flashTestPage[FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS] __attribute__((space(prog),aligned(FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS)));

// We have detected a flash hardware error of some type.
static void FlashError()
{
    while (1) 
    { }
}

static void MiscompareError()
{
    while (1) 
    { }
}

static void WordWriteExample()
{
    uint32_t flash_storage_address;
    bool result;
    uint32_t write_data[4];
    uint32_t read_data[4];

    // Get flash page aligned address of flash reserved above for this test.
    flash_storage_address = FLASH_GetErasePageAddress((uint32_t)&flashTestPage[0]);

    FLASH_Unlock(FLASH_UNLOCK_KEY);

    result = FLASH_ErasePage(flash_storage_address);
    if (result == false)
    {
        FlashError();
    }
    
    // Fill first 4 flash words with data
    // For this product we must write two adjacent words at a one time.
    write_data[0] = 0x00112233;
    write_data[1] = 0x00445566;
    write_data[2] = 0x00AABBCC;
    write_data[3] = 0x00DDEEFF;

    // For this product we must write two adjacent words at a one time.
    result  = FLASH_WriteDoubleWord24(flash_storage_address,   write_data[0], write_data[1]);
    result &= FLASH_WriteDoubleWord24(flash_storage_address + 4, 
                                      write_data[2], write_data[3]);

    if (result == false)
    {
        FlashError();
    }

    // Clear Key for NVM Commands so accidental call to flash routines will not corrupt flash
    FLASH_Lock();
    
    // read the flash words to verify the data
    read_data[0] = FLASH_ReadWord24(flash_storage_address);
    read_data[1] = FLASH_ReadWord24(flash_storage_address + 2);
    read_data[2] = FLASH_ReadWord24(flash_storage_address + 4 );
    read_data[3] = FLASH_ReadWord24(flash_storage_address + 6 );

    // Stop if the read data does not match the write data;
    if ( (write_data[0] != read_data[0]) ||
         (write_data[1] != read_data[1]) ||
         (write_data[2] != read_data[2]) ||
         (write_data[3] != read_data[3]) )
    {
        MiscompareError();    
    }
}

static void PageWritexample()
{
    uint32_t flash_storage_address,flashOffset;
    bool result;
    uint32_t readData;
    // Get flash page aligned address of flash reserved above for this test.
    flash_storage_address = FLASH_GetErasePageAddress((uint32_t)&flashTestPage[0]);
    FLASH_Unlock(FLASH_UNLOCK_KEY);
    
    // ------------------------------------------
    // Fill a page of memory with data.  
    // ------------------------------------------
    
    // Erase the page of flash at this address
    result = FLASH_ErasePage(flash_storage_address);
    if (result == false)
    {
        FlashError();
    }
  
    // Program flash with a data pattern.  For the data pattern we will use the index 
    // into the flash as the data.
    for (flashOffset= 0; flashOffset< FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS; flashOffset += 4)
    {
        result = FLASH_WriteDoubleWord24(flash_storage_address+flashOffset, flashOffset, flashOffset+2);
        if (result == false)
        {
            FlashError();
        }   
    }

    // Clear Key for NVM Commands so accidental call to flash routines will not corrupt flash
    FLASH_Lock();

    
    // Verify the flash data is correct.  If it's not branch to error loop.
    // The data in the flash is the offset into the flash page.
    for (flashOffset= 0; flashOffset< FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS; flashOffset += 2)
    {
        readData = FLASH_ReadWord24(flash_storage_address+flashOffset);
        if (readData != flashOffset )
        {
            MiscompareError();
        }   
    }

}


void FlashDemo()
{
    WordWriteExample();
    PageWritexample();
}


void ReadAppConfig(uint8_t* buf, uint32_t size)
{
    const uint32_t flash_storage_address = FLASH_APP_CONFIG_ADDRESS;
    uint32_t flashOffset;
    uint32_t read_data;
    int32_t remain_size = size;
    const uint32_t read_size = 3; //24 bits
    uint32_t copy_size = read_size;
    while(read_size == copy_size) {
        read_data = FLASH_ReadWord24(flash_storage_address+flashOffset);
        //printf("%08lx\r\n", read_data);
        copy_size = (remain_size < read_size)? remain_size : read_size;
        memcpy(buf,&read_data, copy_size);
        buf += read_size;
        remain_size -= read_size;
        flashOffset += 2;
    }
}
bool WriteAppConfig(uint8_t* const buffer, uint32_t size)
{
    uint32_t flash_storage_address, flashOffset;
    uint16_t idx = 0;
    uint32_t word1, word2;
    bool result;
    const uint32_t size_in_pc_units = ((size+5)/6)*INSTRUCTION_IN_PC_UNITS*2; //round up to 6 since we write 6 bytes each time

    flash_storage_address = FLASH_GetErasePageAddress(FLASH_APP_CONFIG_ADDRESS);
    FLASH_Unlock(FLASH_UNLOCK_KEY);

    result = FLASH_ErasePage(flash_storage_address);

    if (result == false) {
        FLASH_Lock();
        //LED_RED();
        return false;
    }

    for (flashOffset= 0; flashOffset < size_in_pc_units; flashOffset += 4) {
        word1 = ((uint32_t)buffer[idx+0]) <<  0 |
                ((uint32_t)buffer[idx+1]) <<  8 |
                ((uint32_t)buffer[idx+2]) << 16;
        idx+=3;

        word2 = ((uint32_t)buffer[idx+0]) <<  0 | 
                ((uint32_t)buffer[idx+1]) <<  8 |
                ((uint32_t)buffer[idx+2]) << 16;
        idx+=3;

        result = FLASH_WriteDoubleWord24(flash_storage_address+flashOffset, word1, word2);
        if (result == false) {
            FLASH_Lock();
            //LED_RED();
            return false;
        }
    }

    FLASH_Lock();
    
    //TODO(Toan): verifying
    
    return true;
}