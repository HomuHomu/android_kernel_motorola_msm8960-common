/*
 * MSE, ml-motofelica@nttd-mse.com
 * 04/27/2012 First Release of FeliCa device driver
 * 05/29/2012 Update FeliCa driver
 * 06/05/2012 Fix of pointing out checkpatch.pl
 *
 * @file felica_rws.h
 * @brief Local header file of FeliCa RWS driver
 *
 * @date 2012/04/27
 *
 *
 * Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved.
 */

#ifndef __DV_FELICA_RWS_H__
#define __DV_FELICA_RWS_H__

/* ========================================================================== */
/* define                                                                     */
/* ========================================================================== */
#define FELICA_OUT_RWS_SIZE			(1)
#define FELICA_OUT_RWS_AVAILABLE	(0)

#define FELICA_RWS_MINOR_NO_RWS		(0)
#define FELICA_DEV_NUM				(1)

/* ========================================================================== */
/* extern                                                                     */
/* ========================================================================== */
extern dev_t dev_id_rws;

#endif /* __DV_FELICA_RWS_H__ */
