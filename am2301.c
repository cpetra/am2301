/*
 *  am2301.c
 * The driver uses GPIO interrupts to read 1-wire data.
 * New data will be available as /proc/am2301.
 * The temperature is reported as Celsius degrees.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/ktime.h>
#include <linux/proc_fs.h>


/*   Host:         ~~~~|__|~~~
 *   Sensor ACK:              |__|~~|
 *   Sensor data:                    __|~~~|
 */
enum eState {
	READ_START,
	READ_START_HIGH,
	READ_BIT_START,
	READ_BIT_HIGH,
	READ_BIT_LOW,
	READ_STOP,
};

struct st_inf {
	int t;
	int rh;
};

#define SHORT_DELAY 3
#define DEFAULT_DELAY 5

static int _pin = 24;
static int _read_delay = DEFAULT_DELAY; /* in seconds */
static int _irq = -1;
static volatile int _read_req = READ_STOP;
static struct task_struct *ts = NULL;
static wait_queue_head_t _queue;
static ktime_t _old;
static volatile int _ulen;
static struct st_inf sns;
static int _reads[2] = {0, 0};
static unsigned char _data[5];
#ifdef CONFIG_PROC_FS
static struct proc_dir_entry* entry;
#endif

#define CHECK_RET(r) do { \
		if (r != 0) {			\
			return r;		\
		}				\
	} while (0)

/*
 * GPIO ISR
 * State machine for reading the sensor request.
 * Hopefuly the hardware performs some filtering.
 */
static irqreturn_t read_isr(int irq, void *data)
{
	ktime_t now = ktime_get_real();
	static int bit_count, char_count;

	switch (_read_req) {
	case READ_START:
		if (gpio_get_value(_pin) == 0) {
			_read_req = READ_START_HIGH;
		}
		break;
	case READ_START_HIGH:
		if (gpio_get_value(_pin) == 1) {
			_read_req = READ_BIT_START;
		}
		break;
	case READ_BIT_START:
		if (gpio_get_value(_pin) == 0) {
			_read_req = READ_BIT_HIGH;
			bit_count = 7;
			char_count = 0;
			memset(_data, 0, sizeof(_data));
		}
		break;
	case READ_BIT_HIGH:
		if (gpio_get_value(_pin) == 1) {
			_read_req = READ_BIT_LOW;
		}
		break;
	case READ_BIT_LOW:
		if (gpio_get_value(_pin) == 0) {
			_ulen = ktime_us_delta(now, _old);
			if (_ulen > 40) {
				_data[char_count] |= (1 << bit_count);
			}
			if (--bit_count < 0) {
				char_count++;
				bit_count = 7;
			}
			if (char_count == 5) {
				_read_req = READ_STOP;
				wake_up_interruptible(&_queue);
			} else {
				_read_req = READ_BIT_HIGH;
			}
		}
		break;
	case READ_STOP:
	default:
		break;
	}
	_old = now;
	return IRQ_HANDLED;
}

static int start_read(void)
{
	int ret;

	/*
	 * Set pin high and wait for two milliseconds.
	 */
 	ret = gpio_direction_output(_pin, 1);
	CHECK_RET(ret);

	udelay(2000);

	/*
	 * Set pin low and wait for at least 750 us.
	 * Set it high again, then wait for the sensor to put out a low pulse.
	 */
	gpio_set_value(_pin, 0);
	udelay(800);
	gpio_set_value(_pin, 1);

	_read_req = READ_START;

 	ret = gpio_direction_input(_pin);
	CHECK_RET(ret);

	return 0;
}

static int do_read_data(struct st_inf *s)
{
	unsigned char cks = 0;
	int max_u = 100;

 	if (!wait_event_interruptible_timeout(_queue, (_read_req == READ_STOP), max_u)) {
		_read_req = READ_STOP;
		return -1;
	}

	/*
	 * This seems to fail often.
	 * Assuming that sometimes one bit is lost and, if the values are low enough,
	 * the checksum is identical.
	 */
	cks = _data[0] + _data[1] + _data[2] + _data[3];
	if (cks != _data[4]) {
		return -1;
	}

	/* ToDo: Check negative temperatures */
	s->rh = (int) (int16_t)(((uint16_t) _data[0] << 8) | (uint16_t) _data [1]);
	s->t  = (int) (((uint16_t) _data[2] << 8) | (uint16_t) _data [3]);
	if (s->rh > 1000 || s->rh < 0 || s->t > 800 || s->t < -400 ) {
		return -1;
	}
	return 0;
}

static int read_thread(void *data)
{
	int local_delay = 0;
	struct st_inf s;
	static struct st_inf sp;

        while (!kthread_should_stop()) {

		/*
		 * Do not sleep the whole chunk, otherwise if
		 *  the module is removed it will wait for that whole delay.
		 */
		if (local_delay != 0) {
			local_delay--;
			/* ToDo: Find a better interruptible delay implementation */
			wait_event_interruptible_timeout(_queue, 0, HZ);
			continue;
		}

		local_delay = _read_delay;
		_reads[0]++;

		if (start_read() != 0) {
			continue;
		}

		if (do_read_data(&s) != 0) {
			local_delay = SHORT_DELAY; /* Ignore this reading */
		}
		else {
			if (_reads[1] == 0) {
				local_delay = SHORT_DELAY;
				_reads[1]++ ;

			}
			else {
				if ((s.t - sp.t > 50) ||  /* 5 degrees difference */
				    (s.t - sp.t < -50) ||
				    (s.rh - sp.rh > 100) || /* or 10 RH differene */
				    (s.rh - sp.rh < -100))
				{
					/* Ignore this reading */
					local_delay = SHORT_DELAY;
				}
				else {
					sns = s;
					_reads[1]++;
				}
			}
			sp = s;
		}
        }
        return 0;
}

#ifdef CONFIG_PROC_FS
static int proc_read_status(char *page, char **start, off_t off, int count,
			    int *eof, void *data)
{
	char *p = page;
	int len;

	if (_reads[1] < 2) { /* at least two consecutive readings OK */
		p += sprintf(p, "Not ready\n");
	} else {
		p += sprintf(p, "T     :\t\t%d.%d\n", sns.t / 10, sns.t%10);
		p += sprintf(p, "RH    :\t\t%d.%d\n", sns.rh / 10, sns.rh%10);
		p += sprintf(p, "QUAL  :\t\t%d/%d %d%c\n", _reads[1], _reads[0],
			     _reads[1] * 100 / _reads[0], '\%');
	}
	len = (p - page) - off;
	if (len < 0) {
		len = 0;
	}

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}
#endif

static int __init am2301_init(void)
{
	int ret;

	printk(KERN_INFO "Init am2301\n");

	ret = gpio_request_one(_pin, GPIOF_OUT_INIT_HIGH, "AM2301");

	if (ret != 0) {
		printk(KERN_ERR "Unable to request GPIO, err: %d\n", ret);
		return ret;
	}

	_irq =  gpio_to_irq(_pin);
	if (_irq < 0) {
		printk(KERN_ERR "am2301: Unable to create IRQ\n");
		goto _cleanup_1;

	}

	init_waitqueue_head(&_queue);

        ret = request_irq(_irq, read_isr,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "read_isr", NULL);

	ts = kthread_create(read_thread, NULL, "am2301");

	if (ts) {
		wake_up_process(ts);
	} else {
		printk(KERN_ERR "am2301: Unable to create thread\n");
		goto _cleanup_2;
	}

#ifdef CONFIG_PROC_FS
	entry = create_proc_entry("am2301", S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "am2301: Unable to create proc/am2301\n");
		goto _cleanup_3;
	}
	entry->read_proc = proc_read_status;
#endif
	return 0;

_cleanup_3:
	kthread_stop(ts);
_cleanup_2:
	free_irq(_irq, NULL);
_cleanup_1:
	gpio_free(_pin);

	return -1;
}

static void __exit am2301_exit(void)
{
	if (ts) {
                kthread_stop(ts);
        }

	if (_irq >= 0) {
		free_irq(_irq, NULL);
	}

 	(void) gpio_direction_output(_pin, 1);
	gpio_free(_pin);

	remove_proc_entry("am2301", NULL);
	printk(KERN_INFO "am2301: exit\n");
}

module_init(am2301_init);
module_exit(am2301_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Constantin Petra");
MODULE_DESCRIPTION("AM2301 driver");
module_param(_pin, int, S_IRUGO);
//MODULE_PARM(pin, "i");
MODULE_PARM_DESC(_pin, "Pin number - if not set, assuming 24");
