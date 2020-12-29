/**
 * @file I2C.c
 *
 * I2C Related Function
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/
#ifdef __cplusplus
extern "C" {
#endif ///< for __cplusplus

#include "UpiDef.h"

/**
 * @brief  I2CInit()
 *
 *  Init the I2C module
 *
 * @return NULL
 *
 */
GVOID I2CInit(GVOID)
{

}


/**
 * @brief  I2CExeCmd()
 *
 *  Execute I2C Command
 *
 * @return NULL
 *
 */
GBOOL I2CExeCmd(GI2CCmdType *pI2CCmd)
{
  GBOOL bRet = GTRUE;
  GBYTE bRetry = 0;

  while(1)
  {
  ///--------------------------------///
  /// Execute I2C Command
  ///--------------------------------///
		
    
    ///-------------------------------------------------------------------------------///
    /// 1. Mutex Lock
    ///-------------------------------------------------------------------------------/// 
    UpiMutexLock((GVOID *)&UpiSbsMutex);
       
    bRet = _I2cExeCmd(pI2CCmd);
	  	
    ///-------------------------------------------------------------------------------///
    /// 3. Mutex UnLock
    ///-------------------------------------------------------------------------------/// 
    UpiMutexUnLock((GVOID *)&UpiSbsMutex);     
	  	
    if(bRet == GTRUE)
    {
      break;
    }

    ///--------------------------------///
    /// Retry
    ///--------------------------------///
    bRetry++;
    if(bRetry == I2C_MAX_RETRY_CNT)
    {
      UpiGauge.wErrorStatus|=GAUGE_ERR_I2C_FAIL;
      UpiGauge.wI2cErrCnt++;
      return bRet;
     }
  }
	
  UpiGauge.wI2cErrCnt = 0;
  UpiGauge.wErrorStatus&=~GAUGE_ERR_I2C_FAIL;
  return bRet;
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

