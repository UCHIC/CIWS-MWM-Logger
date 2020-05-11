#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <avr/io.h>
#include "state.h"

#define addr_siteNum100         0
#define addr_siteNum10          1
#define addr_siteNum1           2
#define addr_meterSize          3
#define addr_logID100           4
#define addr_logID10            5
#define addr_logID1             6
#define addr_fileNum1000        7
#define addr_fileNum100         8
#define addr_fileNum10          9
#define addr_fileNum1          10
#define addr_factor1           11
#define addr_factorTenths      12
#define addr_factorHundredths  13
#define addr_factorThousandths 14

#define addr_chs0 15
#define addr_chs1 16
#define addr_chs2 17
#define addr_chs3 18

bool configurationExists(void);
uint8_t readConfiguration(uint8_t segment);
void writeConfiguration(uint8_t segment, char data);

#endif
