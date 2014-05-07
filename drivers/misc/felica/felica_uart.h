/*
 * MSE, ml-motofelica@nttd-mse.com
 * 04/27/2012 First Release of FeliCa device driver
 * 05/29/2012 Update FeliCa driver
 * 06/05/2012 Fix of pointing out checkpatch.pl
 *
 * @file felica_uart.h
 * @brief FeliCa driver header file
 *
 * @date 2012/04/27
 *
 *
 * Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved.
 */
#ifndef __DV_FELICA_UART_H__
#define __DV_FELICA_UART_H__

/* ========================================================================== */
/* define                                                                     */
/* ========================================================================== */
#define FELICA_COMM_MINOR_NO_COMM	(0)
#define FELICA_DEV_NUM				(1)

#define TTY_FILE_PATH				"/dev/ttyHSL1"
#define UART_TTY_READBUFF_SIZE		4095
#define UART_TTY_WRITEBUFF_SIZE		260

/* ========================================================================== */
/* extern                                                                     */
/* ========================================================================== */
extern dev_t dev_id_uart;

#endif /* __DV_FELICA_UART_H__ */
