/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 *  The code contained herein is licensed under the GNU General Public
 *  License. You may obtain a copy of the GNU General Public License
 *  Version 2 or later at the following locations:
 *  *
 *  http://www.opensource.org/licenses/gpl-license.html
 *  http://www.gnu.org/copyleft/gpl.html
*/

#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fsl_devices.h>
#include <linux/suspend.h>

struct wakeup_ctrl {
	int wakeup_irq;
	int usb_irq;
	struct fsl_usb2_wakeup_platform_data *pdata;
	struct task_struct *thread;
	struct completion  event;
};
static struct wakeup_ctrl *g_ctrl;

extern int usb_event_is_otg_wakeup(void);
extern void usb_debounce_id_pin(void);

static void wakeup_clk_gate(struct fsl_usb2_wakeup_platform_data *pdata, bool on)
{
	if (pdata->usb_clock_for_pm)
		pdata->usb_clock_for_pm(on);
}

static bool usb2_is_in_lowpower(struct wakeup_ctrl *ctrl)
{
	int i;
	struct fsl_usb2_wakeup_platform_data *pdata = ctrl->pdata;
	/* all the usb module related the wakeup is in lowpower mode */
	for (i = 0; i < 3; i++) {
		if (pdata->usb_pdata[i]) {
			if (pdata->usb_pdata[i]->phy_lowpower_suspend && !pdata->usb_pdata[i]->lowpower)
				return false;
		}
	}
	return true;
}

static void delay_process_wakeup(struct wakeup_ctrl *ctrl)
{
	int i;
	struct fsl_usb2_wakeup_platform_data *pdata = ctrl->pdata;
	disable_irq_nosync(ctrl->wakeup_irq);
	if ((ctrl->usb_irq > 0) && (ctrl->wakeup_irq != ctrl->usb_irq))
		disable_irq_nosync(ctrl->usb_irq);

	for (i = 0; i < 3; i++) {
		if (pdata->usb_pdata[i]) {
			pdata->usb_pdata[i]->irq_delay = 1;
		}
	}
	complete(&ctrl->event);
}

static irqreturn_t usb_wakeup_handler(int irq, void *_dev)
{
	struct wakeup_ctrl *ctrl = (struct wakeup_ctrl *)_dev;
	irqreturn_t ret = IRQ_NONE;
	if (usb2_is_in_lowpower(ctrl)) {
		printk(KERN_INFO "usb wakeup is here\n");
		delay_process_wakeup(ctrl);
		ret = IRQ_HANDLED;
	}
	return ret;
}

static bool is_wakeup(struct fsl_usb2_platform_data *pdata)
{
	if (pdata->is_wakeup_event)
		return pdata->is_wakeup_event();
	return false;
}
static void wakeup_event_handler(struct wakeup_ctrl *ctrl)
{
	struct fsl_usb2_wakeup_platform_data *pdata = ctrl->pdata;
	int i;

	wakeup_clk_gate(ctrl->pdata, true);
	/* if this is an wakeup event, we should debounce ID pin
	 * so we can get the correct ID value(ID status) here
	 * */
	if (usb_event_is_otg_wakeup())
		usb_debounce_id_pin();

	for (i = 0; i < 3; i++) {
		struct fsl_usb2_platform_data *usb_pdata = pdata->usb_pdata[i];
		if (usb_pdata) {
			usb_pdata->irq_delay = 0;
			if (is_wakeup(usb_pdata)) {
				usb_pdata->wakeup_event = 1;
				if (usb_pdata->usb_clock_for_pm)
					usb_pdata->usb_clock_for_pm(true);
				usb_pdata->lowpower = 0;
			}
		}
	}
	wakeup_clk_gate(ctrl->pdata, false);
}

static int wakeup_event_thread(void *param)
{
	struct wakeup_ctrl *ctrl = (struct wakeup_ctrl *)param;
	struct sched_param sch_param = {.sched_priority = 1};

	sched_setscheduler(current, SCHED_RR, &sch_param);
	while (1) {
		wait_for_completion(&ctrl->event);
		if (kthread_should_stop())
			break;
		wakeup_event_handler(ctrl);
		enable_irq(ctrl->wakeup_irq);
		if ((ctrl->usb_irq > 0) && (ctrl->wakeup_irq != ctrl->usb_irq))
			enable_irq(ctrl->usb_irq);
	}
	return 0;
}

static int  wakeup_dev_probe(struct platform_device *pdev)
{
	struct fsl_usb2_wakeup_platform_data *pdata;
	struct wakeup_ctrl *ctrl = NULL;
	int status;

	printk(KERN_INFO "IMX usb wakeup probe\n");

	if (!pdev || !pdev->dev.platform_data)
		return -ENODEV;
	ctrl = kzalloc(sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	pdata = pdev->dev.platform_data;
	ctrl->pdata = pdata;
	init_completion(&ctrl->event);
	ctrl->wakeup_irq = platform_get_irq(pdev, 0);
	status = request_irq(ctrl->wakeup_irq, usb_wakeup_handler, IRQF_SHARED, "usb_wakeup", (void *)ctrl);
	if (status)
		goto error1;
	ctrl->usb_irq = platform_get_irq(pdev, 1);

	ctrl->thread = kthread_run(wakeup_event_thread, (void *)ctrl, "usb_wakeup thread");
	status = IS_ERR(ctrl->thread) ? -1 : 0;
	if (status)
		goto error2;
	g_ctrl = ctrl;

	return 0;
error2:
	free_irq(ctrl->wakeup_irq, (void *)ctrl);
error1:
	kfree(ctrl);
	return status;
}

static int  wakeup_dev_exit(struct platform_device *pdev)
{
	if (g_ctrl->thread) {
		complete(&g_ctrl->event);
		kthread_stop(g_ctrl->thread);
	}
	free_irq(g_ctrl->wakeup_irq, (void *)g_ctrl);
	kfree(g_ctrl);
	return 0;
}
static struct platform_driver wakeup_d = {
	.probe   = wakeup_dev_probe,
	.remove  = wakeup_dev_exit,
	.driver = {
		.name = "usb_wakeup",
	},
};

static int __init wakeup_dev_init(void)
{
	return platform_driver_register(&wakeup_d);
}
static void __exit wakeup_dev_uninit(void)
{
	platform_driver_unregister(&wakeup_d);
}

subsys_initcall(wakeup_dev_init);
module_exit(wakeup_dev_uninit);

