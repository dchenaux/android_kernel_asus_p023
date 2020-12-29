/**
 * @file ChargeCtl.c
 *
 *  Charge Control
 *
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
 * @brief CHGCheckFullChargedIn
 *
 *  Check Full Charged
 *
 * @return NULL
 *
 */
GVOID CHGCheckFCClear(GVOID)
{
	///------------------------------///
	/// 1. Check if FC is set
	///------------------------------///	
	if(!(SBS->wBS & FC_BS))
	{
		return;
	}

	///------------------------------///
	/// 2. Leave when under charging
	///------------------------------///		
	if(GGIsDischarging() == GFALSE)
	{
		return;
	}

	///------------------------------///
	/// 3. If RSOC < FC_CLEAR_RSOC, 
	///    then clear FC
	///------------------------------///		
	if(SBS->wBS & FC_BS)
	{
		if(SBS->wRSOC < FC_CLEAR_RSOC)
		{
			SBS->wBS &=~ (FC_BS | TCA_BS);
		}
	}
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

