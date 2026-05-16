#include "common.h"
#include "my_flash.h"
#include "treadmills.h"


uint32_t          u32Addr    = 0x3f00;
uint8_t           u8TestData = 0x5a;

storage_structunion storage_data={0};

ee_param_t ee_param;


void flash_init(void)
{
	Flash_Init(6);

}


void flash_read(void)
{
		uint16_t flash_i=0;
		u32Addr=0x3f00;
    for (flash_i=0; flash_i<sizeof(storage_structParam); flash_i++)
    {
			storage_data.data8[flash_i]	= *((volatile uint8_t*)u32Addr);
			u32Addr++;			
    }
}


void flash_write(void)
{
  uint16_t flash_i=0;
	u32Addr=0x3f00;
	Flash_SectorErase(u32Addr);
  for (flash_i=0; flash_i<sizeof(storage_structParam); flash_i++)
  {
    Flash_WriteByte(u32Addr, storage_data.data8[flash_i]);
    u32Addr++;		
  }
}

void save_param(void)
{
 //**********2023.2.3保存设置的参数*********           
  ee_param.beep_status = treadmills.param.beep_status;
	storage_data.mstruct.Encoder_1=ee_param.beep_status;


//  flash_write();  //写保存上面的数据
}
void read_beep_status(void)
{
//  flash_read(); 
	ee_param.beep_status=storage_data.mstruct.Encoder_1;
  treadmills.param.beep_status = ee_param.beep_status;
}

