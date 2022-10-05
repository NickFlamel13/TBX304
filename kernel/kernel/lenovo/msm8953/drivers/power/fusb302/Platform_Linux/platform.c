#include <linux/printk.h>                                                       // pr_err, printk, etc
#include "fusb30x_global.h"                                                     // Chip structure
#include "platform_helpers.h"                                                   // Implementation details
#include "../core/platform.h"
#include "../core/TypeC.h"

extern struct qpnp_typec_chip *tusb_typec_chip;
extern  int set_property_on_battery(struct qpnp_typec_chip *chip, enum power_supply_property prop);

/*******************************************************************************
* Function:        platform_set/get_vbus_lvl_enable
* Input:           VBUS_LVL - requested voltage
*                  Boolean - enable this voltage level
*                  Boolean - turn off other supported voltages
* Return:          Boolean - on or off
* Description:     Provide access to the VBUS control pins.
******************************************************************************/
void platform_set_vbus_lvl_enable(VBUS_LVL level, FSC_BOOL blnEnable, FSC_BOOL blnDisableOthers)
{
    FSC_U32 i;
    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
        printk("fusb302 func=%s,line=%d,blnEnable=%d,level=%d,blnDisableOthers=%d\n",__func__,__LINE__,blnEnable,level,blnDisableOthers);
        case VBUS_LVL_5V:
            // Enable/Disable the 5V Source
            fusb_GPIO_Set_VBus5v(blnEnable == TRUE ? true : false);
            //Don't use ID to pull up/down, use the separate function that control VBUS boost.
        break;
        // case VBUS_LVL_12V:  //add by longcheer_liml
            // Enable/Disable the 12V Source
            // fusb_GPIO_Set_VBusOther(blnEnable == TRUE ? true : false);
        //  break;
        default:
            // Otherwise, do nothing.
        break;
    }


    // Turn off other levels, if requested
    if (blnDisableOthers || ((level == VBUS_LVL_ALL) && (blnEnable == FALSE)))
    {
        i = 0;

        do {
            // Skip the current level
            if( i == level ) continue;

            // Turn off the other level(s)
            platform_set_vbus_lvl_enable( i, FALSE, FALSE );
        } while (++i < VBUS_LVL_COUNT);
    }

    return;
}

FSC_BOOL platform_get_vbus_lvl_enable(VBUS_LVL level)
{
    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    printk("fusb302 func=%s,line=%d,level=%d\n",__func__,__LINE__,level);
    case VBUS_LVL_5V:
        // Return the state of the 5V VBUS Source.
        return !(fusb_GPIO_Get_VBus5v() ? TRUE : FALSE);

   // case VBUS_LVL_12V: //add by longcheer_liml
        // Return the state of the 12V VBUS Source.
   //     return fusb_GPIO_Get_VBusOther() ? TRUE : FALSE;

    default:
        // Otherwise, return FALSE.
        return FALSE;
    }
}

/*******************************************************************************
* Function:        platform_set_vbus_discharge
* Input:           Boolean
* Return:          None
* Description:     Enable/Disable Vbus Discharge Path
******************************************************************************/
void platform_set_vbus_discharge(FSC_BOOL blnEnable)
{
    // TODO - Implement if required for platform
}

/*******************************************************************************
* Function:        platform_get_device_irq_state
* Input:           None
* Return:          Boolean.  TRUE = Interrupt Active
* Description:     Get the state of the INT_N pin.  INT_N is active low.  This
*                  function handles that by returning TRUE if the pin is
*                  pulled low indicating an active interrupt signal.
******************************************************************************/
FSC_BOOL platform_get_device_irq_state(void)
{
    return fusb_GPIO_Get_IntN() ? TRUE : FALSE;
}

/*******************************************************************************
* Function:        platform_i2c_write
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to transmit
*                  PacketSize - Maximum size of each transmitted packet
*                  IncSize - Number of bytes to send before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer of char data to transmit
* Return:          Error state
* Description:     Write a char buffer to the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_write(FSC_U8 SlaveAddress,
                        FSC_U8 RegAddrLength,
                        FSC_U8 DataLength,
                        FSC_U8 PacketSize,
                        FSC_U8 IncSize,
                        FSC_U32 RegisterAddress,
                        FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    
  //  printk("fusb302 func=%s,line=%d\n",__func__,__LINE__);
    if (Data == NULL)
    {
        printk(" fusb302%s - Error: Write data buffer is NULL!\n", __func__);
        ret = TRUE;
    }
    else if (fusb_I2C_WriteData((FSC_U8)RegisterAddress, DataLength, Data))
    {
        ret = FALSE;
    }
    else  // I2C Write failure
    {
        ret = TRUE;       // Write data block to the device
    }
    return ret;
}

/*******************************************************************************
* Function:        platform_i2c_read
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to attempt to read
*                  PacketSize - Maximum size of each received packet
*                  IncSize - Number of bytes to recv before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer for received char data
* Return:          Error state.
* Description:     Read char data from the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_read(FSC_U8 SlaveAddress,
                       FSC_U8 RegAddrLength,
                       FSC_U8 DataLength,
                       FSC_U8 PacketSize,
                       FSC_U8 IncSize,
                       FSC_U32 RegisterAddress,
                       FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    FSC_S32 i = 0;
    FSC_U8 temp = 0;
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        printk(" fusb302FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        return TRUE;
    }

    if (Data == NULL)
    {
        printk(" fusb302%s - Error: Read data buffer is NULL!\n", __func__);
        ret = TRUE;
    }
    else if (DataLength > 1 && chip->use_i2c_blocks)    // Do block reads if able and necessary
    {
        if (!fusb_I2C_ReadBlockData(RegisterAddress, DataLength, Data))
        {
            ret = TRUE;
        }
        else
        {
            ret = FALSE;
        }
    }
    else
    {
        for (i = 0; i < DataLength; i++)
        {
            if (fusb_I2C_ReadData((FSC_U8)RegisterAddress + i, &temp))
            {
                Data[i] = temp;
                ret = FALSE;
            }
            else
            {
                ret = TRUE;
                break;
            }
        }
    }

    return ret;
}

/*****************************************************************************
* Function:        platform_enable_timer
* Input:           enable - TRUE to enable platform timer, FALSE to disable
* Return:          None
* Description:     Enables or disables platform timer
******************************************************************************/
void platform_enable_timer(FSC_BOOL enable)
{

}

/*****************************************************************************
* Function:        platform_delay_10us
* Input:           delayCount - Number of 10us delays to wait
* Return:          None
* Description:     Perform a software delay in intervals of 10us.
******************************************************************************/
void platform_delay_10us(FSC_U32 delayCount)
{
    fusb_Delay10us(delayCount);
}

/*******************************************************************************
* Function:        platform_notify_cc_orientation
* Input:           orientation - Orientation of CC (NONE, CC1, CC2)
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current CC orientation. Called in SetStateAttached... and
*                  SetStateUnattached functions.
******************************************************************************/
extern int typec_direction; //add by longcheer_liml_2017_04_17
void platform_notify_cc_orientation(CC_ORIENTATION orientation)
{
	// Optional: Notify platform of CC orientation
	typec_direction = orientation;//add by longcheer_liml_2017_04_17

}

/*******************************************************************************
* Function:        platform_notify_pd_contract
* Input:           contract - TRUE: Contract, FALSE: No Contract
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current PD contract status. Called in PDPolicy.
*******************************************************************************/
extern FSC_U8 GetUSBPDStatusOverview(void);
extern void GetUSBPDStatus(FSC_U8 abytData[]);

void platform_notify_pd_contract(FSC_BOOL contract)
{
	FSC_U8 pdstatus = 0;
	FSC_U8 aby_data[64] = {0};
	doDataObject_t  contract_obj = {0};
	sopMainHeader_t received_caps_header = {0};
	doDataObject_t  received_caps_data[7] = {{0}};
	struct qpnp_typec_chip *chip = tusb_typec_chip;

    int i = 0, j = 0;
    int intIndex = 5;
    //unsigned int voltage_mv = 0;
    
	
    if (contract)  // PD contract has been established.
    {
        pdstatus = GetUSBPDStatusOverview();

        if (!(pdstatus&0x04)) 
        {
            //PAD is sink
            //charging
            //Get voltage and Current
            GetUSBPDStatus(aby_data);
		
		    //Get USBPDContract information;
            for (i=0;i<4;i++)
            {
                contract_obj.byte[i] = aby_data[intIndex++];
            }
		
		    //Get Received capabilities
            received_caps_header.byte[0] = aby_data[intIndex++];
            received_caps_header.byte[1] = aby_data[intIndex++];
            for (i=0;i<7;i++)                                                       // Loop through each data object
            {
                for (j=0;j<4;j++)                                                   // Loop through each byte of the data object
		            received_caps_data[i].byte[j] =  aby_data[intIndex++];
				
                         //PDO from source
			    printk("FUSB [%d], maxCurrent=%dmA, voltage=%dmV\n", i, received_caps_data[i].FPDOSupply.MaxCurrent*10, 
				received_caps_data[i].FPDOSupply.Voltage*50);
            } 
		    chip->current_ma = contract_obj.FVRDO.OpCurrent*10;
		    //voltage_mv = received_caps_data[contract_obj.FVRDO.ObjectPosition-1].FPDOSupply.Voltage*50;
printk("fusb302 func=%s,line=%d,PD chip->current_ma=%d\n",__func__,__LINE__,chip->current_ma);
			// TO DO
	        // Set the charging current with current_ma and voltage_mv
	        set_property_on_battery(chip,POWER_SUPPLY_PROP_CURRENT_CAPABILITY);
       
        }		
    }
}

/*******************************************************************************
* Function:        platform_notify_unsupported_accessory
* Input:           None
* Return:          None
* Description:     A callback used by the core to report entry to the
*                  Unsupported Accessory state. The platform may implement
*                  USB Billboard.
*******************************************************************************/
void platform_notify_unsupported_accessory(void)
{
    // Optional: Implement USB Billboard
}

/*******************************************************************************
* Function:        platform_set_data_role
* Input:           PolicyIsDFP - Current data role
* Return:          None
* Description:     A callback used by the core to report the new data role after
*                  a data role swap.
*******************************************************************************/
void platform_set_data_role(FSC_BOOL PolicyIsDFP)
{
	int rc;
	struct qpnp_typec_chip *chip = tusb_typec_chip;
printk("fusb302 func=%s,line=%d,PolicyIsDFP=%d\n",__func__,__LINE__,PolicyIsDFP);	
	if (PolicyIsDFP)
    {
        // DFP/HOST
        printk("%s: host dfp\n", __func__); 
        chip->typec_state = POWER_SUPPLY_TYPE_DFP;
        chip->type_c_psy.type = POWER_SUPPLY_TYPE_DFP;
        chip->current_ma = 0;
        rc = set_property_on_battery(chip,POWER_SUPPLY_PROP_TYPEC_MODE);
        if (rc)
            printk("failed to set TYPEC MODE on battery psy rc=%d\n",rc);
 
	}
	else
	{
	     //Device
        printk("%s: TYPEC_ATTACHED_SNK devices\n", __func__);
        chip->current_ma = 2000;
        chip->typec_state = POWER_SUPPLY_TYPE_UFP;
        chip->type_c_psy.type = POWER_SUPPLY_TYPE_UFP;
        rc = set_property_on_battery(chip, POWER_SUPPLY_PROP_TYPEC_MODE);
        if (rc)
            pr_err("failed to set TYPEC MODE on battery psy rc=%d\n", rc);  
	}
		
}

/*******************************************************************************
* Function:        platform_notify_bist
* Input:           bistEnabled - TRUE when BIST enabled, FALSE when disabled
* Return:          None
* Description:     A callback that may be used to limit current sinking during
*                  BIST
*******************************************************************************/
void platform_notify_bist(FSC_BOOL bistEnabled)
{
    // Do something
}

void platform_set_timer(TIMER *timer, FSC_U16 timeout)
{
    timer->start_time = get_system_time();
    timer->timeout = timeout;
}

FSC_BOOL platform_check_timer(TIMER *timer)
{
    return (((FSC_U16)(get_system_time() - timer->start_time) > timer->timeout) ? TRUE: FALSE);
}

FSC_U16 platform_get_system_time()
{
    return get_system_time();
}
