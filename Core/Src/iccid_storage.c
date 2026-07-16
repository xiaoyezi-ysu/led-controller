#include "iccid_storage.h"
#include <string.h>

void ICCID_Load(char *iccid)
{
  const uint8_t *p = (const uint8_t*)ICCID_FLASH_ADDR;
  if (p[0] == '8' && p[1] == '9')
  {
    strcpy(iccid, (const char*)p);
  }
  else
  {
    iccid[0] = '\0';
  }
}

void ICCID_Save(const char *iccid)
{
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef fe = {0};
  fe.TypeErase = FLASH_TYPEERASE_PAGES;
  fe.PageAddress = ICCID_FLASH_ADDR;
  fe.NbPages = 1;
  uint32_t page_err;
  HAL_FLASHEx_Erase(&fe, &page_err);
  for (int i = 0; i < 32; i += 2)
  {
    uint16_t w = (uint16_t)(uint8_t)iccid[i] | ((uint16_t)(uint8_t)iccid[i+1] << 8);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ICCID_FLASH_ADDR + i, w);
  }
  HAL_FLASH_Lock();
}
