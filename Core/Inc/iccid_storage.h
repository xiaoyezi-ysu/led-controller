#ifndef ICCID_STORAGE_H
#define ICCID_STORAGE_H

#include "main.h"

#define ICCID_FLASH_ADDR  0x0800FC00

void ICCID_Load(char *iccid);
void ICCID_Save(const char *iccid);

#endif
