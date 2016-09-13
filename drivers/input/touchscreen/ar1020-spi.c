/*
 * Microchip SPI Touchscreen Driver
 *
 * Copyright (c) 2011 Microchip Technology, Inc.
 * 
 * http://www.microchip.com/mtouch
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>	
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
//#include <linux/types.h>
//#include <linux/gpio.h>

//#include <linux/irq.h>
//#include <linux/kobject.h>
//#include <linux/string.h>
//#include <linux/sysfs.h>
//#include <linux/init.h>

//#include <linux/slab.h>

/* The maximum packet byte length */
#define MCHIP_MAX_LENGTH 5

/* The private data structure that is referenced within the SPI bus driver */
struct ar1020_spi_priv {
	struct spi_device *client;
	struct input_dev *input;
	struct work_struct work;
	int irq;
	int testCount;
};

/* These are all the sysfs variables used to store and retrieve information
   from a user-level application */
static char sendBuffer[100];
static char receiveBuffer[100];
static int commandMode=0;
static int commandDataPending=0;
static int minX=0;
static int maxX=4095;
static int minY=0;
static int maxY=4095;
static int swapAxes=0;
static int invertX=0;
static int invertY=0;
static int lastPUCoordX=0;
static int lastPUCoordY=0;


/* Since the reference to private data is stored within the SPI
   bus driver, we will store another reference within this driver
   so the sysfs related function may also access this data */
struct ar1020_spi_priv *privRef=NULL;

/**********************************************************************
Function:
	commandDataPending_show()

Description:
	Display value of "commandDataPending" variable to application that is
	requesting it's value.	
**********************************************************************/
static ssize_t commandDataPending_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d", commandDataPending);
}

/******************************************************************************
Function:
	commandDataPending_store()

Description:
	Save value to "commandDataPending" variable from application that is
	requesting this.	
******************************************************************************/
static ssize_t commandDataPending_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%d", &commandDataPending);
	return count;
}

static struct kobj_attribute commandDataPending_attribute =
	__ATTR(commandDataPending, 0660, commandDataPending_show, commandDataPending_store);

/******************************************************************************
Function:
	commandMode_show()

Description:
	Display value of "commandMode" variable to application that is
	requesting it's value.	
******************************************************************************/
static ssize_t commandMode_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d", commandMode);
}

/******************************************************************************
Function:
	commandMode_store()

Description:
	Save value to "commandMode" variable from application that is
	requesting this.	
******************************************************************************/
static ssize_t commandMode_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%d", &commandMode);
	return count;

}

static struct kobj_attribute commandMode_attribute =
	__ATTR(commandMode, 0660, commandMode_show, commandMode_store);

/******************************************************************************
Function:
	receiveBuffer_show()

Description:
	Display value of "receiveBuffer" variable to application that is
	requesting it's value.	
******************************************************************************/
static ssize_t receiveBuffer_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	/* since we have now read the receiveBuffer, receive data is no longer pending */
	commandDataPending=0;
	return sprintf(buf, "%s", receiveBuffer);
}

/******************************************************************************
Function:
	receiveBuffer_store()

Description:
	Save value to "receiveBuffer" variable from application that is
	requesting this.	
******************************************************************************/
static ssize_t receiveBuffer_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	return snprintf(receiveBuffer,sizeof(receiveBuffer),"%s",buf);
}

static struct kobj_attribute receiveBuffer_attribute =
	__ATTR(receiveBuffer, 0660, receiveBuffer_show, receiveBuffer_store);

/******************************************************************************
Function:
	sendBuffer_show()

Description:
	Display value of "sendBuffer" variable to application that is
	requesting it's value.	
******************************************************************************/
static ssize_t sendBuffer_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%s", sendBuffer);
}

/******************************************************************************
Function:
	sendBuffer_store()

Description:
	Save value to "sendBuffer" variable from application that is
	requesting this.	
******************************************************************************/
static ssize_t sendBuffer_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	int commandByte[8];
	int numCommandBytes;
	int i;

	commandDataPending=0;

	/* disallow commands to be sent until command mode is enabled */
	if (0==commandMode)
	{
		printk("AR1020 SPI: Warning: command bytes will be ignored until commandMode is enabled\n");
		strcpy(sendBuffer,"");
		return count;
	}

	numCommandBytes=sscanf(buf,"%x %x %x %x %x %x %x %x",&commandByte[0],&commandByte[1],&commandByte[2],&commandByte[3],&commandByte[4],&commandByte[5],&commandByte[6],&commandByte[7]);

	printk(KERN_DEBUG "AR1020 SPI: Processed %d bytes.\n",numCommandBytes); 

	/* Verify command string to send to controller is valid */
	if (numCommandBytes<3) 
	{
		printk("AR1020 I2C: Insufficient command bytes to process.\n");
	}
	else if (commandByte[0]!=0x55)
	{
		printk("AR1020 I2C: Invalid header byte (0x55 expected).\n");
	}	
	else if (commandByte[1] != (numCommandBytes-2))
	{
		printk("AR1020 I2C: Number of command bytes specified not valid for current string.\n");
	}	

	strcpy(sendBuffer,"");
	printk(KERN_DEBUG "AR1020 SPI: sending command bytes: "); 
	for (i=0;i<numCommandBytes;i++)
	{
	   printk(KERN_DEBUG "0x%02x ",commandByte[i]);
	   spi_write(privRef->client,(unsigned char *)&commandByte[i],1);
	}
	printk(KERN_DEBUG "\n");

	return snprintf(sendBuffer,sizeof(sendBuffer),"%s",buf);
}

static struct kobj_attribute sendBuffer_attribute =
	__ATTR(sendBuffer, 0660, sendBuffer_show, sendBuffer_store);

/******************************************************************************
Function:
	calibrationSettings_show()

Description:
	Display value of "calibrationSettings" variable to application that is
	requesting it's value.	The handling of the calibration settings has
	been grouped together.
******************************************************************************/
static ssize_t calibrationSettings_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	int calibrationSetting=0;

	if (strcmp(attr->attr.name, "minX") == 0)
		calibrationSetting = minX;
	else if (strcmp(attr->attr.name, "maxX") == 0)
		calibrationSetting = maxX;
	else if (strcmp(attr->attr.name, "minY") == 0)
		calibrationSetting = minY;
	else if (strcmp(attr->attr.name, "maxY") == 0)
		calibrationSetting = maxY;
	else if (strcmp(attr->attr.name, "swapAxes") == 0)
		calibrationSetting = swapAxes;
	else if (strcmp(attr->attr.name, "invertX") == 0)
		calibrationSetting = invertX;
	else if (strcmp(attr->attr.name, "invertY") == 0)
		calibrationSetting = invertY;
	else if (strcmp(attr->attr.name, "lastPUCoordX") == 0)
		calibrationSetting = lastPUCoordX;
	else if (strcmp(attr->attr.name, "lastPUCoordY") == 0)
		calibrationSetting = lastPUCoordY;

	return sprintf(buf, "%d\n", calibrationSetting);
}

/******************************************************************************
Function:
	calibrationSettings_store()

Description:
	Save calibration setting to corresponding variable from application 
	that is requesting this.	
******************************************************************************/
static ssize_t calibrationSettings_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	int calibrationSetting;

	sscanf(buf, "%d", &calibrationSetting);

	if (strcmp(attr->attr.name, "minX") == 0)
		minX = calibrationSetting;
	else if (strcmp(attr->attr.name, "maxX") == 0)
		maxX = calibrationSetting;
	else if (strcmp(attr->attr.name, "minY") == 0)
		minY = calibrationSetting;
	else if (strcmp(attr->attr.name, "maxY") == 0)
		maxY = calibrationSetting;
	else if (strcmp(attr->attr.name, "swapAxes") == 0)
		swapAxes = calibrationSetting;
	else if (strcmp(attr->attr.name, "invertX") == 0)
		invertX = calibrationSetting;
	else if (strcmp(attr->attr.name, "invertY") == 0)
		invertY = calibrationSetting;
	else if (strcmp(attr->attr.name, "lastPUCoordX") == 0)
		lastPUCoordX = calibrationSetting;
	else if (strcmp(attr->attr.name, "lastPUCoordY") == 0)
		lastPUCoordY = calibrationSetting;

	return count;
}

/* Defines sysfs variable associations */
static struct kobj_attribute minX_attribute =
	__ATTR(minX, 0660, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute maxX_attribute =
	__ATTR(maxX, 0660, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute minY_attribute =
	__ATTR(minY, 0660, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute maxY_attribute =
	__ATTR(maxY, 0660, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute swapAxes_attribute =
	__ATTR(swapAxes, 0660, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute invertX_attribute =
	__ATTR(invertX, 0660, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute invertY_attribute =
	__ATTR(invertY, 0660, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute lastPUCoordX_attribute =
	__ATTR(lastPUCoordX, 0660, calibrationSettings_show, calibrationSettings_store);
static struct kobj_attribute lastPUCoordY_attribute =
	__ATTR(lastPUCoordY, 0660, calibrationSettings_show, calibrationSettings_store);

/*
 * Create a group of calibration attributes so we may work with them
 * as a set.
 */
static struct attribute *attrs[] = {
	&commandDataPending_attribute.attr,
	&commandMode_attribute.attr,
	&receiveBuffer_attribute.attr,
	&sendBuffer_attribute.attr,
	&minX_attribute.attr,
	&maxX_attribute.attr,
	&minY_attribute.attr,
	&maxY_attribute.attr,
	&swapAxes_attribute.attr,
	&invertX_attribute.attr,
	&invertY_attribute.attr,
	&lastPUCoordX_attribute.attr,
	&lastPUCoordY_attribute.attr,
	NULL,	
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *ar1020_kobj;


static irqreturn_t touch_irq_handler_func(int irq, void *dev_id);

/******************************************************************************
Function:
	decodeAR1020Packet()

Description:
	Decode packets of data from a device path using AR1XXX protocol. 
	Returns 1 if a full packet is available, zero otherwise.
******************************************************************************/
int decodeAR1020Packet(struct ar1020_spi_priv* priv, char* packet, int *index, char data)
{
	int returnValue=0;
	int x;
	int y;
	int button;
	int calX;
	int calY;
	packet[*index] = data;

	/****************************************************
	
	Data format, 5 bytes: SYNC, DATA1, DATA2, DATA3, DATA4
	
	SYNC [7:0]: 1,0,0,0,0,TOUCHSTATUS[0:0]
	DATA1[7:0]: 0,X-LOW[6:0]
	DATA2[7:0]: 0,X-HIGH[4:0]
	DATA3[7:0]: 0,Y-LOW[6:0]
	DATA4[7:0]: 0,Y-HIGH[4:0]
	
	TOUCHSTATUS: 0 = Touch up, 1 = Touch down
	
	****************************************************/		
	
	switch ((*index)++) {
		case 0:
			if(data==0x4d)
			{
				dev_dbg(&priv->client->dev, "no valid data available 0x4D\n");
				*index=0;
				returnValue=-1;
			}
			else if (!(0x80 & data))
			{
				dev_dbg(&priv->client->dev, "sync bit not set 0x%02x\n",data);
			    /* Sync bit not set */
			    *index=0;
			    returnValue=0;
			}
			break;

		case (MCHIP_MAX_LENGTH - 1):
			/* verify byte is valid for current index */
			if (0x80 & data)
			{
				dev_dbg(&priv->client->dev, "byte not valid 1 0x%02x 0x%02x\n",packet[0],data);
				/* byte not valid */
				*index=1;
				break;
			}		  

			x = ((packet[2] & 0x1f) << 7) | (packet[1] & 0x7f);
			y= ((packet[4] & 0x1f) << 7) | (packet[3] & 0x7f);
			button = 0 != (packet[0] & 1);

			if (0==button)
			{
				lastPUCoordX=x;
				lastPUCoordY=y;
			}
			
			if (swapAxes)
			{
				int temp=x;
				x=y;
				y=temp;
			}


			if (invertX)
				x=4095-x;

			if (invertY)
				y=4095-y;

			if (x<minX)
				calX=0;
			else if (x>maxX)
				calX=4095;
			else
				/* percentage across calibration area times the maximum controller width */
				calX=((x-minX)*4095)/(maxX-minX);

			if (y<minY)
				calY=0;
			else if (y>maxY)
				calY=4095;
			else
				/* percentage across calibration area times the maximum controller height */
				calY=((y-minY)*4095)/(maxY-minY);		

			//printk(KERN_DEBUG "AR1020 SPI: %d %d %d\n",calX,calY,button);

			input_report_abs(priv->input, ABS_X, calX);
			input_report_abs(priv->input, ABS_Y, calY);
			input_report_key(priv->input, BTN_TOUCH, button);
			input_sync(priv->input);

			returnValue=1;
			break;
		default:
			/* verify byte is valid for current index */
			if (0x80 & data)
			{
				/* byte not valid */
				printk("AR1020 SPI: Byte not valid. Value: 0x%02x Index: 0x%02x\n",data, *index);
				*index=1;
				returnValue=-1;
			}			  
			break;
			
	}

	return returnValue;
}

/******************************************************************************
Function:
	ar1020_spi_open()

Description:
	This function is called on every attempt to open the current device  
	and used for both debugging purposes fullfilling an SPI driver 
	function callback requirement.
******************************************************************************/
static int ar1020_spi_open(struct input_dev *dev)
{
	return 0;
}

/******************************************************************************
Function:
	ar1020_spi_close()

Description:
	This function is called on every attempt to close the current device  
	and used for both debugging purposes fullfilling an SPI driver 
	function callback requirement.
******************************************************************************/
static void ar1020_spi_close(struct input_dev *dev)
{
}



/******************************************************************************
Function:
	ar1020_spi_readdata()

Description:
	When the controller interrupt is asserted, this function is scheduled
	to be called to read the controller data within the 
	touch_irq_handler_func() function.
******************************************************************************/
static void ar1020_spi_readdata(struct work_struct *work)
{
	struct ar1020_spi_priv *priv =
		container_of(work, struct ar1020_spi_priv, work);
	int index=0;
	char buff[9];
	int ret;
	int i;

	/* We want to ensure we only read packets when we are not in the middle of command communication. Disable command mode after receiving command response to resume receiving packets. */
	if (commandMode)
	{
		commandDataPending=1;
		/* process up to 9 bytes */
		strcpy(receiveBuffer,"");

		/* header byte */
		spi_read(priv->client,&buff[0],1);
		snprintf(receiveBuffer,sizeof(receiveBuffer),"0x%02x",0xff&buff[0]);

		if (0x55 != buff[0])
		{
			printk("AR1020 SPI: invalid header byte\n");
			return;
		}

		/* num data bytes */
		spi_read(priv->client,&buff[1],1);
		snprintf(receiveBuffer,sizeof(receiveBuffer),"%s 0x%02x",receiveBuffer,0xff&buff[1]);
		if (buff[1] > 6)
		{
			printk("AR1020 SPI: invalid byte count\n");
			return;
		}

		
		for (i=0;i<buff[1];i++)
		{
			spi_read(priv->client,&buff[i+2],1);			
			snprintf(receiveBuffer,sizeof(receiveBuffer),"%s 0x%02x",receiveBuffer,0xff&buff[i+2]);
		}
		snprintf(receiveBuffer,sizeof(receiveBuffer),"%s\n",receiveBuffer);
		printk(KERN_DEBUG "AR1020 SPI: command response: %s",receiveBuffer);
		return;
	}

//	for (i=0;i<5;i++)
//	{
//	  buff[i]=0;
//	}
	memset(buff,0,sizeof(buff));

	/* process up to 9 bytes */
	for (i=0;i<9;i++)
	{
	  spi_read(priv->client,&buff[index],1);
	  ret=decodeAR1020Packet(priv,buff, &index, buff[index]);
	  /* if a one is returned, then we have a full packet */
	  if (1==ret || ret==-1)
	  {
		break;
          }	
	}
}

/******************************************************************************
Function:
	ar1020_spi_probe()

Description:
	After the kernel's platform specific source files have been modified to 
	reference the "ar1020_spi" driver, this function will then be called.
	This function needs to be called to finish registering the driver.
******************************************************************************/
static int ar1020_spi_probe(struct spi_device *client)
{
	struct ar1020_spi_priv *priv=NULL;
	struct input_dev *input_dev=NULL;
	int err=0;
	int i;
	char buff[5];

    printk("AR1020 SPI: ar1020_spi_probe: begin\n");

	for (i=0;i<5;i++)
	{
		buff[i]=0;
	}

	if (!client) {
		printk(KERN_ERR "AR1020 SPI: client pointer is NULL\n");
		err = -EINVAL;
		goto error;
	}

	if ((!client->irq)) {
		printk(KERN_ERR "AR1020 SPI: no IRQ set for touch controller\n");
		err = -EINVAL;
		goto error;
	}

	client->bits_per_word = 8;
	client->mode = SPI_MODE_1;
	err = spi_setup(client);
	if (err < 0)
	{
		printk(KERN_ERR "AR1020 SPI: failed to setup spi\n");
		return err;
	}

	priv = kzalloc(sizeof(struct ar1020_spi_priv), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!priv) {
		printk(KERN_ERR "AR1020 SPI: kzalloc error\n");
		err = -ENOMEM;
		goto error;
	}

	/* Backup pointer so sysfs helper functions may also have access to private data */
	privRef=priv;

	if (!input_dev)
	{
		printk(KERN_ERR "AR1020 SPI: input allocate error\n");
		err = -ENOMEM;		
		goto error;
	}


	priv->client = client;
	priv->irq = client->irq;
	priv->input = input_dev;




	INIT_WORK(&priv->work, ar1020_spi_readdata);

	input_dev->name = "AR1020 Touchscreen";

	input_dev->open = ar1020_spi_open;
	input_dev->close = ar1020_spi_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, 4095, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 4095, 0, 0);
	err = input_register_device(input_dev);
	if (err)
	{
		printk(KERN_ERR "AR1020 SPI: error registering input device\n");
		goto error;
	}

	for (i=0;i<5;i++)
	{
	  spi_read(priv->client,&buff[i],1);
	}

	/* set type and register gpio pin as our interrupt */
	err = devm_request_threaded_irq(&priv->client->dev, priv->irq, NULL,
			touch_irq_handler_func, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"AR1020 SPI IRQ", priv);

	if (err) {
		dev_err(&priv->client->dev, "Failed to register interrupt\n");
		goto error;
	}

	return 0;

 error:

	if (input_dev)
		input_free_device(input_dev);

	if (priv)
		kfree(priv);

	return err;

	return 0;
}

/******************************************************************************
Function:
	ar1020_spi_remove()

Description:
	Unregister/remove the kernel driver from memory. 
******************************************************************************/
static int ar1020_spi_remove(struct spi_device *client)
{
	struct ar1020_spi_priv *priv = (struct ar1020_spi_priv *)dev_get_drvdata(&client->dev);

	printk("AR1020 SPI: ar1020_spi_remove: begin\n");

	free_irq(priv->irq, priv);
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}



/******************************************************************************
Function:
	touch_irq_handler_func()

Description:
	After the interrupt is asserted for the controller, this
	is the first function that is called.  Since this is a time sensitive
	function, we need to immediately schedule work so the integrity of
	properly system operation 

	This function needs to be called to finish registering the driver.
******************************************************************************/
static irqreturn_t touch_irq_handler_func(int irq, void *dev_id)
{
	struct ar1020_spi_priv *priv = (struct ar1020_spi_priv *)dev_id;
	char buff[5];
	int i;
	int err;
	for (i=0;i<5;i++)
	{
		buff[i]=0;	  
	}

	if (!priv) {
		printk(KERN_ERR "AR1020 SPI: touch_irq_handler_funct: no private data\n");
		err = -EINVAL;
		return err;
	}
	ar1020_spi_readdata(&priv->work);
	 /* delegate SPI transactions since hardware interupts need to be handled very fast */
	//schedule_work(&priv->work);

	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static const struct of_device_id ar1020_dt_ids[] = {
	{ .compatible = "microchip,ar1020-spi"},
	{ }
};
MODULE_DEVICE_TABLE(of, ar1020_dt_ids);
#endif

/* This is the initial set of information information the kernel has
   before probing drivers on the system, */
static struct spi_driver ar1020_spi_driver = {
	.driver = {
		.name	= "ar1020-spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ar1020_spi_probe,
	.remove		= ar1020_spi_remove,
	/* suspend/resume functions not needed since controller automatically
  	   put's itself to sleep mode after configurable short period of time */
};


/******************************************************************************
Function:
	ar1020_spi_init()

Description:
	This function is called during startup even if the platform specific
	files have not been setup yet.
******************************************************************************/
static int __init ar1020_spi_init(void)
{
	int retval;

        printk("AR1020 SPI: ar1020_spi_init: begin\n");
	strcpy(receiveBuffer,"");
	strcpy(sendBuffer,"");

	/*
	 * Creates a kobject "ar1020" that appears as a sub-directory
	 * under "/sys/kernel".
	 */
	ar1020_kobj = kobject_create_and_add("ar1020", kernel_kobj);
	if (!ar1020_kobj)
	{
		printk(KERN_ERR "AR1020 SPI: cannot create kobject\n");
		return -ENOMEM;
	}

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(ar1020_kobj, &attr_group);
	if (retval)
	{
		printk(KERN_ERR "AR1020 SPI: error registering ar1020-spi driver's sysfs interface\n");
		kobject_put(ar1020_kobj);
	}

	return spi_register_driver(&ar1020_spi_driver);
}

/******************************************************************************
Function:
	ar1020_spi_exit()

Description:
	This function is called after ar1020_spi_remove() immediately before
	being removed from the kernel.
******************************************************************************/
static void __exit ar1020_spi_exit(void)
{
	printk("AR1020 SPI: ar1020_i2c_exit begin\n");
	kobject_put(ar1020_kobj);
	spi_unregister_driver(&ar1020_spi_driver);
}

MODULE_AUTHOR("Steve Grahovac <steve.grahovac@microchip.com>");
MODULE_DESCRIPTION("AR1020 touchscreen SPI bus driver");
MODULE_LICENSE("GPL");

/* Enable the ar1020_spi_init() to be run by the kernel during initialization */
module_init(ar1020_spi_init);

/* Enables the ar1020_spi_exit() to be called during cleanup.  This only
has an effect if the driver is compiled as a kernel module. */
module_exit(ar1020_spi_exit);


