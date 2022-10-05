/* 
 * File:   fusb30x_driver.c
 * Company: Fairchild Semiconductor
 *
 * Created on September 2, 2015, 10:22 AM
 */

/* Standard Linux includes */
#include <linux/init.h>                                                         // __init, __initdata, etc
#include <linux/module.h>                                                       // Needed to be a module
#include <linux/kernel.h>                                                       // Needed to be a kernel module
#include <linux/i2c.h>                                                          // I2C functionality
#include <linux/slab.h>                                                         // devm_kzalloc
#include <linux/types.h>                                                        // Kernel datatypes
#include <linux/errno.h>                                                        // EINVAL, ERANGE, etc
#include <linux/of_device.h>                                                    // Device tree functionality

/* Driver-specific includes */
#include "fusb30x_global.h"                                                     // Driver-specific structures/types
#include "platform_helpers.h"                                                   // I2C R/W, GPIO, misc, etc

#ifdef FSC_DEBUG
#include "../core/core.h"                                                       // GetDeviceTypeCStatus
#endif // FSC_DEBUG

#include "fusb30x_driver.h"

/******************************************************************************
* Driver functions
******************************************************************************/
static int __init fusb30x_init(void)
{
    printk("fusb302 FUSB  %s - Start driver initialization...\n", __func__);

	return i2c_add_driver(&fusb30x_driver);
}

static void __exit fusb30x_exit(void)
{
	i2c_del_driver(&fusb30x_driver);
    printk("fusb302 FUSB  %s - Driver deleted...\n", __func__);
}

static int fusb302_i2c_resume(struct device* dev)
{
    struct fusb30x_chip *chip;
        struct i2c_client *client = to_i2c_client(dev);

        if (client) {
            chip = i2c_get_clientdata(client);
                if (chip)
                up(&chip->suspend_lock);
        }
     return 0;
}

static int fusb302_i2c_suspend(struct device* dev)
{
    struct fusb30x_chip* chip;
        struct i2c_client* client =  to_i2c_client(dev);

        if (client) {
             chip = i2c_get_clientdata(client);
                 if (chip)
                    down(&chip->suspend_lock);
        }
        return 0;
}

#define PINCTRL_STATE_ACTIVE "fusb_active"
#define PINCTRL_STATE_SUSPEND "fusb_suspend"
static int fusb302_pinctrl_init(void)
{
    struct pinctrl *fusb302_pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct pinctrl_state *pinctrl_state_suspend;
    struct fusb30x_chip* chip = fusb30x_GetChip();

	fusb302_pinctrl = devm_pinctrl_get(&chip->client->dev);
	if (!fusb302_pinctrl) {
		 pr_err("FUSB  Target does not use pinctrl\n");
		return -EINVAL;
	}

	pinctrl_state_active = pinctrl_lookup_state(fusb302_pinctrl, PINCTRL_STATE_ACTIVE);
	if (!pinctrl_state_active) {
		 pr_err("FUSB  Can not lookup %s pinstate\n", PINCTRL_STATE_ACTIVE);
		return -EINVAL;
	}

	pinctrl_state_suspend = pinctrl_lookup_state(fusb302_pinctrl, PINCTRL_STATE_SUSPEND);
	if (!pinctrl_state_suspend) {
		pr_err("FUSB Can not lookup %s pinstate\n", PINCTRL_STATE_SUSPEND);
		return -EINVAL;
	}

	pinctrl_select_state(fusb302_pinctrl, pinctrl_state_active);
	return 0;
}

static int fusb30x_probe (struct i2c_client* client,
                          const struct i2c_device_id* id)
{
    int ret = 0;
    struct fusb30x_chip* chip; 
    struct i2c_adapter* adapter;

    if (!client)
    {
        pr_err("FUSB  %s - Error: Client structure is NULL!\n", __func__);
        return -EINVAL;
    }
    dev_info(&client->dev, "%s\n", __func__);

    /* Make sure probe was called on a compatible device */
	if (!of_match_device(fusb30x_dt_match, &client->dev))
	{
		printk("fusb302 FUSB  %s - Error: Device tree mismatch!\n", __func__);
		return -EINVAL;
	}
    printk("fusb302 FUSB  %s - Device tree matched!\n", __func__);

    /* Allocate space for our chip structure (devm_* is managed by the device) */
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
	{
		printk("fusb302 FUSB  %s - Error: Unable to allocate memory for g_chip!\n", __func__);
		return -ENOMEM;
	}
    chip->client = client;                                                      // Assign our client handle to our chip
    fusb30x_SetChip(chip);                                                      // Set our global chip's address to the newly allocated memory
    printk("fusb302 FUSB  %s - Chip structure is set! Chip: %p ... g_chip: %p\n", __func__, chip, fusb30x_GetChip());

    /* Initialize the chip lock */
    mutex_init(&chip->lock);

    /* Initialize the chip's data members */
    fusb_InitChipData();
    printk("fusb302 FUSB  %s - Chip struct data initialized!\n", __func__);

    /* Verify that the system has our required I2C/SMBUS functionality (see <linux/i2c.h> for definitions) */
    adapter = to_i2c_adapter(client->dev.parent);
    if (i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_BLOCK_REQUIRED_FUNC))
    {
        chip->use_i2c_blocks = true;
    }
    else
    {
        // If the platform doesn't support block reads, try with block writes and single reads (works with eg. RPi)
        // NOTE: It is likely that this may result in non-standard behavior, but will often be 'close enough' to work for most things
        dev_warn(&client->dev, "FUSB  %s - Warning: I2C/SMBus block read/write functionality not supported, checking single-read mode...\n", __func__);
        if (!i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_REQUIRED_FUNC))
        {
            printk("fusb302 FUSB  %s - Error: Required I2C/SMBus functionality not supported!\n", __func__);
            printk("fusb302 FUSB  %s - I2C Supported Functionality Mask: 0x%x\n", __func__, i2c_get_functionality(adapter));
            return -EIO;
        }
    }
    printk("fusb302 FUSB  %s - I2C Functionality check passed! Block reads: %s\n", __func__, chip->use_i2c_blocks ? "YES" : "NO");

    /* Assign our struct as the client's driverdata */
    i2c_set_clientdata(client, chip);
    printk("fusb302 FUSB  %s - I2C client data set!\n", __func__);

    /* Verify that our device exists and that it's what we expect */
    if (!fusb_IsDeviceValid())
    {
        printk("fusb302 FUSB  %s - Error: Unable to communicate with device!\n", __func__);
        return -EIO;
    }
    printk("fusb302 FUSB  %s - Device check passed!\n", __func__);

    /* reset fusb302*/
    fusb_reset();

    /* Initialize semaphore*/
    sema_init(&chip->suspend_lock, 1);

    fusb302_pinctrl_init();// LONGCHEER P3592 add by longcheer_liml
    
    /* Initialize the platform's GPIO pins and IRQ */
    ret = fusb_InitializeGPIO();
    if (ret)
    {
        printk("fusb302 FUSB  %s - Error: Unable to initialize GPIO!\n", __func__);
        return ret;
    }
    printk("fusb302 FUSB  %s - GPIO initialized!\n", __func__);

#ifdef FSC_DEBUG
    /* Initialize debug sysfs file accessors */
    fusb_Sysfs_Init();
    printk("fusb302 FUSB  %s - Sysfs device file created!\n", __func__);
#endif // FSC_DEBUG

#ifdef FSC_INTERRUPT_TRIGGERED
    /* Enable interrupts after successful core/GPIO initialization */
    ret = fusb_EnableInterrupts();
    if (ret)
    {
        printk("fusb302 FUSB  %s - Error: Unable to enable interrupts! Error code: %d\n", __func__, ret);
        return -EIO;
    }

    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now)
    *  Interrupt must be enabled before starting 302 initialization */
    fusb_InitializeCore();
    printk("fusb302 FUSB  %s - Core is initialized!\n", __func__);
#else
    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now) */
    fusb_InitializeCore();
    printk("fusb302 FUSB  %s - Core is initialized!\n", __func__);

    /* Init our workers, but don't start them yet */
    fusb_InitializeWorkers();
    /* Start worker threads after successful initialization */
    fusb_ScheduleWork();
    printk("fusb302 FUSB  %s - Workers initialized and scheduled!\n", __func__);
#endif  // ifdef FSC_POLLING elif FSC_INTERRUPT_TRIGGERED

    dev_info(&client->dev, "FUSB  %s - FUSB30X Driver loaded successfully!\n", __func__);
	return ret;
}

static int fusb30x_remove(struct i2c_client* client)
{
    printk("fusb302 FUSB  %s - Removing fusb30x device!\n", __func__);

#ifndef FSC_INTERRUPT_TRIGGERED // Polling mode by default
    fusb_StopThreads();
#endif  // !FSC_INTERRUPT_TRIGGERED

    fusb_GPIO_Cleanup();
    printk("fusb302 FUSB  %s - FUSB30x device removed from driver...\n", __func__);
    return 0;
}

static void fusb30x_shutdown(struct i2c_client *client)
{
    fusb_reset();
    fusb_GPIO_Set_VBus5v(0);
        printk("fusb302 FUSB	%s - fusb302 shutdown\n", __func__);
}


/*******************************************************************************
 * Driver macros
 ******************************************************************************/
module_init(fusb30x_init);                                                      // Defines the module's entrance function
module_exit(fusb30x_exit);                                                      // Defines the module's exit function

MODULE_LICENSE("GPL");                                                          // Exposed on call to modinfo
MODULE_DESCRIPTION("Fairchild FUSB30x Driver");                                 // Exposed on call to modinfo
MODULE_AUTHOR("Fairchild");                        								// Exposed on call to modinfo
