/*
 * MSE, ml-motofelica@nttd-mse.com
 * 04/27/2012 First Release of FeliCa device driver
 * 05/22/2012 Release of CEN, INT, RFS function
 * 05/29/2012 Update FeliCa driver
 *
 * @file felica.h
 * @brief Local header file of FeliCa driver
 *
 * @date 2012/04/27
 *
 *
 * Copyright(C) 2012 NTT DATA MSE CORPORATION. All right reserved.
 */

#ifndef __DV_FELICA_H__
#define __DV_FELICA_H__

/*
 * include extra header
 */
#include <linux/sched.h>					/* jiffies    */

/*
 * The FeliCa terminal condition to acquire and to set.
 */
#define FELICA_OUT_L		(0x00)	/* condition L */
#define FELICA_OUT_H		(0x01)	/* condition H */
#define FELICA_OUT_RFS_L	(0x01)	/* RFS condition L */
#define FELICA_OUT_RFS_H	(0x00)	/* RFS condition H */

#define FELICA_OUT_SIZE		(1)	/* size of to acquire and to set */

/*
 * The Felica interrupt condition to acquire.
 */
#define FELICA_EDGE_L		(0x00)	/* changed condition from H to L */
#define FELICA_EDGE_H		(0x01)	/* changed condition from L to H */
#define FELICA_EDGE_NON		(0xFF)	/* non changed */

#define FELICA_EDGE_OUT_SIZE (2) /* size of to acquire and to set for EDGE */

/*
 * BIT defined
 */
#define FELICA_POS_BIT_0			(0x00000001U)
#define FELICA_POS_BIT_1			(0x00000002U)
#define FELICA_POS_BIT_2			(0x00000004U)
#define FELICA_POS_BIT_3			(0x00000008U)
#define FELICA_POS_BIT_4			(0x00000010U)
#define FELICA_POS_BIT_5			(0x00000020U)
#define FELICA_POS_BIT_6			(0x00000040U)
#define FELICA_POS_BIT_7			(0x00000080U)

/*
 * Lock Controller BU80003GUL REG defined
 */
#define FELICA_I2C_SLAVE_ADDR			0x2B

#define FELICA_LOCKCONT_EEPCNT_ADDR		0x00
#define FELICA_LOCKCONT_OD_ADDR			0x01
#define FELICA_LOCKCONT_LOCK_ADDR		0x02

#define FELICA_LOCKCONT_BUSY_BIT		FELICA_POS_BIT_7
#define FELICA_LOCKCONT_SET2EEP_BIT		FELICA_POS_BIT_7
#define FELICA_LOCKCONT_LOCKEN_BIT		FELICA_POS_BIT_0

/*
 * Identification information of FeliCa device drivers
 */
#define FELICA_DEV_NAME			 "felica"     /* Device Name        */
#define FELICA_DEV_NAME_CTRL_PON "felica_pon" /* PON Terminal Ctrl  */
#define FELICA_DEV_NAME_CTRL_CEN "felica_cen" /* CEN Terminal Ctrl  */
#define FELICA_DEV_NAME_CTRL_RFS "felica_rfs" /* RFS Terminal Ctrl  */
#define FELICA_DEV_NAME_CTRL_ITR "felica_int" /* ITR Terminal Ctrl  */
#define FELICA_DEV_NAME_COMM	 "felica"     /* Commnunication     */
#define FELICA_DEV_NAME_RWS		 "felica_rws" /* RWS                */

#define FELICA_DEV_NUM				(1)
#define FELICA_DEV_ITR_MAX_OPEN_NUM	(1)

/*
 * Minor number
 */
#define FELICA_CTRL_MINOR_NO_PON	(0)		/* PON Terminal Ctrl  */
#define FELICA_CTRL_MINOR_NO_CEN	(1)		/* CEN Terminal Ctrl  */
#define FELICA_CTRL_MINOR_NO_RFS	(2)		/* RFS Terminal Ctrl  */
#define FELICA_CTRL_MINOR_NO_ITR	(3)		/* ITR Terminal Ctrl  */
#define FELICA_COMM_MINOR_NO_COMM	(0)		/* Commnunication     */
#define FELICA_RWS_MINOR_NO_RWS		(0)		/* RWS                */

/*
 * Interrupts device name
 */
#define FELICA_CTRL_ITR_STR_INT "felica_ctrl_int"	/* INT interrupt */

/*
 * GPIO paramater
 */
#define FELICA_GPIO_PORT_LDO_EN		(51)	/* correspondence for hsv */
#define FELICA_GPIO_PORT_LOCKCONT	(98)	/* correspondence for hsv */
#define FELICA_GPIO_PORT_PON		(68)	/* correspondence for hsv */
#define FELICA_GPIO_PORT_RFS		(15)	/* correspondence for hsv */
#define FELICA_GPIO_PORT_INT		(106)	/* correspondence for hsv */

#define FELICA_GPIO_STR_LDO_EN	 "felica_ldo_en"   /* LDO_EN label name */
#define FELICA_GPIO_STR_LOCKCONT "felica_lockcont" /* LOCKCONT label name */
#define FELICA_GPIO_STR_PON		 "felica_pon"      /* PON label name */
#define FELICA_GPIO_STR_RFS		 "felica_rfs"      /* RFS label name */
#define FELICA_GPIO_STR_INT		 "felica_int"      /* INT label name */

/*
 * Timer
 */
#define FELICA_TIMER_CEN_TERMINAL_WAIT	((20 * HZ) / 1000 + 1)
#define FELICA_TIMER_PON_TERMINAL_WAIT	((30 * HZ) / 1000 + 1)

/*
 * The FeliCa terminal condition to acquire and to set for Internal.
 */
#define DFD_OUT_L	(0x00)		/* condition L */
#define DFD_OUT_H	(0x01)		/* condition H */

/*
 * Cleanup parameters
 */
enum {
	DFD_CLUP_NONE = 0,
	DFD_CLUP_UNREG_CDEV_PON,
	DFD_CLUP_CDEV_DEL_PON,
	DFD_CLUP_UNREG_CDEV_CEN,
	DFD_CLUP_CDEV_DEL_CEN,
	DFD_CLUP_UNREG_CDEV_RFS,
	DFD_CLUP_CDEV_DEL_RFS,
	DFD_CLUP_UNREG_CDEV_ITR,
	DFD_CLUP_CDEV_DEL_ITR,
	DFD_CLUP_CLASS_DESTORY,
	DFD_CLUP_DEV_DESTORY_PON,
	DFD_CLUP_DEV_DESTORY_CEN,
	DFD_CLUP_DEV_DESTORY_RFS,
	DFD_CLUP_DEV_DESTORY_ITR,
	DFD_CLUP_DEV_DESTORY_COMM,
	DFD_CLUP_DEV_DESTORY_RWS,
	DFD_CLUP_DEV_DESTORY_CFG,
	DFD_CLUP_ALL
};

/*
 * Wait Timer
 */
struct FELICA_TIMER {
	struct timer_list Timer;
	wait_queue_head_t wait;
};

/*
 * Device control data structure(PON terminal)
 */
struct FELICA_CTRLDEV_PON_INFO {
	/* Timeout control information */
	struct FELICA_TIMER PON_Timer;		/* PON terminal control timer */
};

/*
 * Device control data structure(CEN terminal)
 */
struct FELICA_CTRLDEV_CEN_INFO {
	/* Timeout control information */
	struct FELICA_TIMER PON_Timer;		/* PON terminal control timer */
	struct FELICA_TIMER CEN_Timer;		/* CEN terminal control timer */
};

/*
 * Device control data structure(ITR terminal)
 */
struct FELICA_CTRLDEV_ITR_INFO {
	/* ITR result information */
	unsigned char INT_Flag;				/* INT interrupts */
	unsigned char Reserve_Flag;			/* interrupt reserve */
	unsigned char reserve1[2];			/* reserved       */

	/* wait queue */
	wait_queue_head_t RcvWaitQ;
};


/*
 * Device control data structure
 */
struct FELICA_DEV_INFO {
	unsigned int w_cnt;		/* write open counter  */
	unsigned int r_cnt;		/* read open counter   */
	void *Info;				/* interal saving data */
};


struct FELICA_ITR_INFO {
	unsigned int irq;
	irq_handler_t handler;
	unsigned long flags;
	const char *name;
	void *dev;
};

#endif /* __DV_FELICA_GPIO_H__ */
