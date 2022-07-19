/*
 * @brief : kernel log printk
 * @author: guangjieMVP
 * @date : 2021-05-xx
 * @version : v1.0.0
 * @copyright(c) 2020 : OptoMedic Technologies Co.,Ltd. All rights reserved
 */
#ifndef _SSLOG_H_
#define _SSLOG_H_

#include <linux/string.h>

#define __FILENAME__ (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1) : __FILE__)

#define SLOG_PRINT(X, args...) printk("[%s]-%d:" X, __FILENAME__, __LINE__, ##args)
#define ERR_PRINT(Frm, ...) SLOG_PRINT("Err:" Frm, __VA_ARGS__)
#define DEBUG_PRINT(fmt, args...)   printk("[%s]-%d:"fmt, __FUNCTION__, __LINE__, ##args)       

#endif
