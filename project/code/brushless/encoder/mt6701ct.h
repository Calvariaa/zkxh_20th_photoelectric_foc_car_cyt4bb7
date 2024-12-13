#ifndef _MT6701CT_H
#define _MT6701CT_H

#include "brushless/foc.h"
#include "zf_common_typedef.h"

//================================================���� MT6701CT �ڲ���ַ================================================
#define MT6701CT_DEV_ADDR (0x69) // SA0�ӵأ�0x68 SA0������0x69 ģ��Ĭ������
#define MT6701CT_SPI_W (0x00)
#define MT6701CT_SPI_R (0x80)

// --- Volatile registers
#define MT6701CT_NOP 0x0000   // �޲���ָ��
#define MT6701CT_ERRFL 0x0001 //
#define MT6701CT_PROG 0x0003  // ��оƬ���б�̵ļĴ�����ַ
#define MT6701CT_DIAAGC 0x3FFC
#define MT6701CT_MAG 0x3FFD      //
#define MT6701CT_ANGLEUNC 0x3FFE //
#define MT6701CT_ANGLECOM 0x3FFF // ��ž��ж�̬�Ƕ������Ƕ���Ϣ�ļĴ�����ַ

uint16_t mt6701ct_get_magnet_val();
void mt6701ct_init();

#endif /* _MT6701CT_H */