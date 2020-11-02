#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

static struct spi_message spi_message_defined;
unsigned int design_sequence_from_user[16];
static struct spidev_data *spi_device_data;
uint8_t image_buffer_to_be_displayed[10][24];
uint8_t tx_ar[2];

void initialisePinData(void);
void setupDisplayPins(void);


/*structure for spi transfer  for message exchange*/
struct spi_transfer spitx = {

		.tx_buf = tx_ar,
		.rx_buf = 0,
		.len = 2,
		.bits_per_word = 8,
		.speed_hz = 10000000,
		.delay_usecs = 1,
		.cs_change = 1,
};

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
		| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
		| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
		| SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;
	u32			speed_hz;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/*static unsigned bufsiz = 4096;
module_para	m(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");*/

struct PinMapping {
	int gpioPin1;
	int gpioPin2;
	int gpioPin3;
	int gpioPin4;

}pinMappings[14];

/*open function for the spi matrix device driver where the gpio pins connected to the spi device are initialized*/
/**
 * Pin Matrix data.
 */
void initialisePinData() {
	printk("\n \n \n Entering Spi_driver_led :: initialisePinData method \n \n \n");
	pinMappings[0].gpioPin1 = 11;
	pinMappings[0].gpioPin2 = 32;
	pinMappings[0].gpioPin3 = 81;
	pinMappings[0].gpioPin4 = 81;

	pinMappings[1].gpioPin1 = 12;
	pinMappings[1].gpioPin2 = 28;
	pinMappings[1].gpioPin3 = 45;
	pinMappings[1].gpioPin4 = 81;

	pinMappings[2].gpioPin1 = 13;
	pinMappings[2].gpioPin2 = 34;
	pinMappings[2].gpioPin3 = 77;
	pinMappings[2].gpioPin4 = 81;

	pinMappings[3].gpioPin1 = 14;
	pinMappings[3].gpioPin2 = 16;
	pinMappings[3].gpioPin3 = 76;
	pinMappings[3].gpioPin4 = 64;

	pinMappings[4].gpioPin1 = 6;
	pinMappings[4].gpioPin2 = 36;
	pinMappings[4].gpioPin3 = 81;
	pinMappings[4].gpioPin4 = 81;

	pinMappings[5].gpioPin1 = 0;
	pinMappings[5].gpioPin2 = 18;
	pinMappings[5].gpioPin3 = 66;
	pinMappings[5].gpioPin4 = 81;

	pinMappings[6].gpioPin1 = 1;
	pinMappings[6].gpioPin2 = 20;
	pinMappings[6].gpioPin3 = 68;
	pinMappings[6].gpioPin4 = 81;

	pinMappings[7].gpioPin1 = 38;
	pinMappings[7].gpioPin2 = 81;
	pinMappings[7].gpioPin3 = 81;
	pinMappings[7].gpioPin4 = 81;

	pinMappings[8].gpioPin1 = 40;
	pinMappings[8].gpioPin2 = 81;
	pinMappings[8].gpioPin3 = 81;
	pinMappings[8].gpioPin4 = 81;

	pinMappings[9].gpioPin1 = 4;
	pinMappings[9].gpioPin2 = 22;
	pinMappings[9].gpioPin3 = 70;
	pinMappings[9].gpioPin4 = 81;

	pinMappings[10].gpioPin1 = 10;
	pinMappings[10].gpioPin2 = 26;
	pinMappings[10].gpioPin3 = 74;
	pinMappings[10].gpioPin4 = 81;

	pinMappings[11].gpioPin1 = 5;
	pinMappings[11].gpioPin2 = 24;
	pinMappings[11].gpioPin3 = 44;
	pinMappings[11].gpioPin4 = 72;

	pinMappings[12].gpioPin1 = 15;
	pinMappings[12].gpioPin2 = 42;
	pinMappings[12].gpioPin3 = 81;
	pinMappings[12].gpioPin4 = 81;

	pinMappings[13].gpioPin1 = 7;
	pinMappings[13].gpioPin2 = 30;
	pinMappings[13].gpioPin3 = 46;
	pinMappings[13].gpioPin4 = 81;
	printk("\n \n \n Exiting Spi_driver_led :: initialisePinData method \n \n \n");
}


/**
 * This is used to setup the display pins for displaying LED Matrix patterns.
 */
void setupDisplayPins() {
	// IO 11, 12 AND 13 are the spi led matrix pins.
	//WE need to use the PIn's first gpio pin as SPI Pin.
	//Hence  we ignore the GPIO Pin1 .0
	//--------------- USE GPIO1 AS WE NEED IT ---------
	printk("\n \n \n ENtering setupDisplayPins \n \n \n");
	if (pinMappings[11].gpioPin2 != 81) {
		gpio_request(pinMappings[11].gpioPin2, "Data out");
		gpio_direction_output(pinMappings[11].gpioPin2, 1);
		gpio_set_value_cansleep(pinMappings[11].gpioPin2,0);

	}
	if (pinMappings[11].gpioPin3 != 81) {
		gpio_request(pinMappings[11].gpioPin3, "Data out");
		gpio_direction_output(pinMappings[11].gpioPin3, 1);
		gpio_set_value_cansleep(pinMappings[11].gpioPin3,1);
	}
	if (pinMappings[11].gpioPin4 != 81) {
		gpio_request(pinMappings[11].gpioPin4, "Data out");
		gpio_direction_output(pinMappings[11].gpioPin4, 1);
		gpio_set_value_cansleep(pinMappings[11].gpioPin4,0);
	}
	//------------- USE GPIO 1 AS WE NEED TO DO CHIP SELECT USING GPIO PIN
	if (pinMappings[12].gpioPin1 != 81) {
		gpio_request(pinMappings[12].gpioPin1, "CHip select");
		gpio_direction_output(pinMappings[12].gpioPin1, 1);

	}
	if (pinMappings[12].gpioPin2 != 81) {
		gpio_request(pinMappings[12].gpioPin2, "Chip Select");
		gpio_direction_output(pinMappings[12].gpioPin2, 1);
		gpio_set_value_cansleep(pinMappings[12].gpioPin2,0);

	}
	//-----------DONT USE GPIO1 : WE NEED SPI
	if (pinMappings[13].gpioPin2 != 81) {
		gpio_request(pinMappings[13].gpioPin2, "Serialclock");
		gpio_direction_output(pinMappings[13].gpioPin2, 1);
		gpio_set_value_cansleep(pinMappings[13].gpioPin2,0);

	}

	if (pinMappings[13].gpioPin3 != 81) {
		gpio_request(pinMappings[13].gpioPin3, "Serialclock");
		gpio_direction_output(pinMappings[13].gpioPin3, 1);
		gpio_set_value_cansleep(pinMappings[13].gpioPin3,1);

	}
	printk("\n \n \n Exiting setupDisplayPins \n \n \n");
}


int spi_kernel_led_func(void *data){
	int i,j, x;
	

	for(i=0;i<16;i++){
		x = design_sequence_from_user[i+1];
		if(design_sequence_from_user[i]==0 && design_sequence_from_user[i+1]==0){
			tx_ar[0] = 0x0c;
			tx_ar[1] = 0x00;
			spi_message_init(&spi_message_defined);
			spi_message_add_tail((void *)&spitx, &spi_message_defined);
			gpio_set_value(15,0);
			spi_sync(spi_device_data->spi, &spi_message_defined);
			gpio_set_value(15,1);
			printk("\n inside clear call");

		}
		else{
			for(j=0;j<24;j=j+2){
				tx_ar[0] = image_buffer_to_be_displayed[design_sequence_from_user[i]][j];
				tx_ar[1] = image_buffer_to_be_displayed[design_sequence_from_user[i]][j+1];
				spi_message_init(&spi_message_defined);
				spi_message_add_tail((void *)&spitx, &spi_message_defined);
				gpio_set_value(15,0);
				spi_sync(spi_device_data->spi, &spi_message_defined);
				gpio_set_value(15,1);
			}
		}
		i=i+1;
		msleep(x);
	}
	return 0;
}
/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos){

	int itr;
	struct task_struct *taskspimat;
	uint8_t pattern_sequence[16];
	copy_from_user((void *)&pattern_sequence, (void * __user)buf, sizeof(pattern_sequence));
	for(itr=0;itr<16;itr++)
	{
		design_sequence_from_user[itr] = pattern_sequence[itr];
	}
	taskspimat = kthread_run(&spi_kernel_led_func, (void *)pattern_sequence,"kthread_spi_led");
	msleep(1000);
	return 0;

}

/**
 * IOCTL function.
 */
static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int i=0, vret=0;
	int l=0, count=0;
	uint8_t user_images[10][24];
	printk(" \n \n \n Entering spidev_ioctl method \n \n \n");
	vret = copy_from_user((void *)&user_images,(void *)arg, sizeof(user_images));

	for(i=0;i<10;i++) {
		for(l=0;l<24;l++) {
			count++;
			image_buffer_to_be_displayed[i][l] = user_images[i][l];
		}
	}

	if(vret != 0) {
		printk("Failure : %d number of bytes that could not be copied.\n",vret);
	}
	return 0;
}

/**
 * We initialise the pins of the display once we open the device.
 */
static int spidev_open(struct inode *inode, struct file *filp) {
	printk("\n \n \n Entering spidev_open \n \n \n ");
	initialisePinData();
	setupDisplayPins();
	printk("\n \n \n Initializing the gpio pins for the spi LED matrix \n \n \n");
	return 0;
}
/**
 * We are releasing the gpio pins when we close the spidev file.
 */
static int spidev_release(struct inode *inode, struct file *filp)
{
	printk("\n \n Entering spidev_release \n \n ");
	printk("\n \n Releasing GPIO Pins \n \n");
	if (pinMappings[11].gpioPin2 != 81) {
		gpio_free(pinMappings[11].gpioPin2);

	}
	if (pinMappings[11].gpioPin3 != 81) {
		gpio_free(pinMappings[11].gpioPin3);
	}
	if (pinMappings[11].gpioPin4 != 81) {
		gpio_free(pinMappings[11].gpioPin4);
	}
	//------------- USE GPIO 1 AS WE NEED TO DO CHIP SELECT USING GPIO PIN
	if (pinMappings[12].gpioPin1 != 81) {
		gpio_free(pinMappings[12].gpioPin1);

	}
	if (pinMappings[12].gpioPin2 != 81) {
		gpio_free(pinMappings[12].gpioPin2);

	}
	//-----------DONT USE GPIO1 : WE NEED SPI
	if (pinMappings[13].gpioPin2 != 81) {
		gpio_free(pinMappings[13].gpioPin2);
	}

	if (pinMappings[13].gpioPin3 != 81) {
		gpio_free(pinMappings[13].gpioPin3);

	}
	printk("\n \n Done Releasing GPIO Pins \n \n");
	return 0;
}

static const struct file_operations spidev_fops = {
		.owner =	THIS_MODULE,
		/* REVISIT switch to aio primitives, so that userspace
		 * gets more complete API coverage.  It'll simplify things
		 * too, except for the locking.
		 */
		.write =	spidev_write,
		.unlocked_ioctl = spidev_ioctl,
		.open =		spidev_open,
		.release =	spidev_release,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

#ifdef CONFIG_OF
static const struct of_device_id spidev_dt_ids[] = {
		{ .compatible = "rohm,dh2228fv" },
		{ .compatible = "lineartechnology,ltc2488" },
		{ .compatible = "ge,achc" },
		{ .compatible = "semtech,sx1301" },
		{},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);
#endif

#ifdef CONFIG_ACPI

/* Dummy SPI devices not to be used in production systems */
#define SPIDEV_ACPI_DUMMY	1

static const struct acpi_device_id spidev_acpi_ids[] = {
		/*
		 * The ACPI SPT000* devices are only meant for development and
		 * testing. Systems used in production should have a proper ACPI
		 * description of the connected peripheral and they should also use
		 * a proper driver instead of poking directly to the SPI bus.
		 */
		{ "SPT0001", SPIDEV_ACPI_DUMMY },
		{ "SPT0002", SPIDEV_ACPI_DUMMY },
		{ "SPT0003", SPIDEV_ACPI_DUMMY },
		{},
};
MODULE_DEVICE_TABLE(acpi, spidev_acpi_ids);

#else
static inline void spidev_probe_acpi(struct spi_device *spi) {}
#endif

/*-------------------------------------------------------------------------*/

static int spidev_probe(struct spi_device *spi)
{

	//struct spidev_data	*spidev;
	int			status;
	unsigned long		minor;
	printk("\n \n \n Entering Spi_dev probe \n \n \n");
	/*
	 * spidev should never be referenced in DT without a specific
	 * compatible string, it is a Linux implementation thing
	 * rather than a description of the hardware.

	if (spi->dev.of_node && !of_match_device(spidev_dt_ids, &spi->dev)) {
		dev_err(&spi->dev, "buggy DT: spidev listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
				!of_match_device(spidev_dt_ids, &spi->dev));
	}*/

	//spidev_probe_acpi(spi);

	/* Allocate driver data */
	spi_device_data = kzalloc(sizeof(*spi_device_data), GFP_KERNEL);
	if (!spi_device_data)
		return -ENOMEM;

	/* Initialize the driver data */
	spi_device_data->spi = spi;
	spin_lock_init(&spi_device_data->spi_lock);
	mutex_init(&spi_device_data->buf_lock);

	INIT_LIST_HEAD(&spi_device_data->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spi_device_data->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, spi_device_data->devt,
				spi_device_data, "spidev%d.%d",
				spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spi_device_data->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	spi_device_data->speed_hz = spi->max_speed_hz;

	if (status == 0)
		spi_set_drvdata(spi, spi_device_data);
	else
		kfree(spi_device_data);

	printk("\n \n \n Exiting spidev_probe method \n \n \n");
	return status;
}

static int spidev_remove(struct spi_device *spi)
{
	//struct spidev_data	*spidev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spi_device_data->spi_lock);
	spi_device_data->spi = NULL;
	spin_unlock_irq(&spi_device_data->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spi_device_data->device_entry);
	device_destroy(spidev_class, spi_device_data->devt);
	clear_bit(MINOR(spi_device_data->devt), minors);
	if (spi_device_data->users == 0)
		kfree(spi_device_data);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver spidev_spi_driver = {
		.driver = {
				.name =		"spidev",
				.of_match_table = of_match_ptr(spidev_dt_ids),
				.acpi_match_table = ACPI_PTR(spidev_acpi_ids),
		},
		.probe =	spidev_probe,
		.remove =	spidev_remove,

		/* NOTE:  suspend/resume methods are not necessary here.
		 * We don't do anything except pass the requests to/from
		 * the underlying controller.  The refrigerator handles
		 * most issues; the controller driver handles the rest.
		 */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	//	BUILD_BUG_ON(N_SPI_MINORS > 256);
	printk("\n \n \n Entering spi_init method \n \n \n ");
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (status < 0)
		return status;

	spidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(spidev_class)) {
		printk("\n Error in registering class \n ");
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
		return PTR_ERR(spidev_class);
	}

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		printk("\n Error in registering driver \n");
		class_destroy(spidev_class);
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	}
	printk("\n \n \n Exiting spi_init method \n \n \n ");
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);


MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
