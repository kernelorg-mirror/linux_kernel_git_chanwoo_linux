// SPDX-License-Identifier: GPL-2.0

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/devfreq.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_opp.h>

#include "governor.h"

struct krait_data {
	struct device *dev;
	struct devfreq *devfreq;

	struct clk *l2_clk;

	unsigned long *freq_table; /* L2 bus clock rate */
	unsigned int *l2_cpufreq; /* L2 target CPU frequency */

	struct notifier_block nb;
};

static int krait_cache_set_opp(struct dev_pm_set_opp_data *data)
{
	unsigned long old_freq = data->old_opp.rate, freq = data->new_opp.rate;
	struct dev_pm_opp_supply *supply = &data->new_opp.supplies[0];
	struct regulator *reg = data->regulators[0];
	struct clk *clk = data->clk;
	struct krait_data *kdata;
	unsigned long idle_freq;
	int ret;

	kdata = (struct krait_data *)dev_get_drvdata(data->dev);

	idle_freq = kdata->freq_table[0];

	if (reg) {
		ret = regulator_set_voltage_triplet(reg, supply->u_volt_min,
						    supply->u_volt,
						    supply->u_volt_max);
		if (ret)
			goto exit;
	}

	/*
	 * Set to idle bin if switching from normal to high bin
	 * or vice versa. It has been notice that a bug is triggered
	 * in cache scaling when more than one bin is scaled, to fix
	 * this we first need to transition to the base rate and then
	 * to target rate
	 */
	if (likely(freq != idle_freq && old_freq != idle_freq)) {
		ret = clk_set_rate(clk, idle_freq);
		if (ret)
			goto exit;
	}

	ret = clk_set_rate(clk, freq);
	if (ret)
		goto exit;

exit:
	return ret;
};

static int krait_cache_target(struct device *dev, unsigned long *freq,
			      u32 flags)
{
	return dev_pm_opp_set_rate(dev, *freq);
};

static int krait_cache_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
	struct krait_data *data = dev_get_drvdata(dev);

	stat->busy_time = 0;
	stat->total_time = 0;
	stat->current_frequency = clk_get_rate(data->l2_clk);

	return 0;
};

static int krait_cache_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct krait_data *data = dev_get_drvdata(dev);

	*freq = clk_get_rate(data->l2_clk);

	return 0;
};

static struct devfreq_dev_profile tegra_devfreq_profile = {
	.target = krait_cache_target,
	.get_dev_status = krait_cache_get_dev_status,
	.get_cur_freq = krait_cache_get_cur_freq
};

static int krait_cache_notifier(struct notifier_block *nb, unsigned long action,
				void *v)
{
	struct cpufreq_freqs *freqs;
	unsigned int cpu, cur_cpu;
	struct krait_data *data;
	struct devfreq *devfreq;
	unsigned long freq;
	int ret = 0;

	if (action != CPUFREQ_POSTCHANGE)
		return NOTIFY_OK;

	data = container_of(nb, struct krait_data, nb);
	devfreq = data->devfreq;

	mutex_lock_nested(&devfreq->lock, SINGLE_DEPTH_NESTING);

	freqs = (struct cpufreq_freqs *)v;
	freq = freqs->new;
	cur_cpu = freqs->policy->cpu;

	/* find the max freq across all core */
	for_each_present_cpu(cpu)
		if (cpu != cur_cpu)
			freq = max(freq, (unsigned long)cpufreq_quick_get(cpu));

	devfreq->governor->get_target_freq(devfreq, &freq);

	ret = devfreq->profile->target(data->dev, &freq, 0);
	if (ret < 0)
		goto out;

	if (devfreq->profile->freq_table &&
	    (devfreq_update_status(devfreq, freq)))
		dev_err(data->dev,
			"Couldn't update frequency transition information.\n");

	devfreq->previous_freq = freq;

out:
	mutex_unlock(&devfreq->lock);
	return notifier_from_errno(ret);
};

static int krait_cache_governor_get_target(struct devfreq *devfreq,
					   unsigned long *freq)
{
	unsigned int *l2_cpufreq;
	unsigned long *freq_table;
	unsigned long target_freq = *freq;
	struct krait_data *data = dev_get_drvdata(devfreq->dev.parent);

	l2_cpufreq = data->l2_cpufreq;
	freq_table = data->freq_table;

	/*
	 * Find the highest l2 freq interval based on the max cpufreq
	 * across all core
	 */
	while (*(l2_cpufreq = l2_cpufreq + 1) && target_freq >= *l2_cpufreq)
		freq_table = freq_table + 1;

	*freq = *freq_table;

	return 0;
};

static int krait_cache_governor_event_handler(struct devfreq *devfreq,
					      unsigned int event, void *data)
{
	struct krait_data *kdata = dev_get_drvdata(devfreq->dev.parent);
	int ret = 0;

	switch (event) {
	case DEVFREQ_GOV_START:
		kdata->nb.notifier_call = krait_cache_notifier;
		ret = cpufreq_register_notifier(&kdata->nb,
						CPUFREQ_TRANSITION_NOTIFIER);
		break;

	case DEVFREQ_GOV_STOP:
		cpufreq_unregister_notifier(&kdata->nb,
					    CPUFREQ_TRANSITION_NOTIFIER);
		break;
	}

	return ret;
};

static struct devfreq_governor krait_devfreq_governor = {
	.name = "krait_governor",
	.get_target_freq = krait_cache_governor_get_target,
	.event_handler = krait_cache_governor_event_handler,
	.immutable = true,
};

static int krait_cache_probe(struct platform_device *pdev)
{
	int ret, count;
	struct opp_table *table;
	struct krait_data *data;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;

	data->l2_clk = devm_clk_get(dev, "l2");
	if (IS_ERR(data->l2_clk))
		return PTR_ERR(data->l2_clk);

	table = dev_pm_opp_set_regulators(dev, (const char *[]){ "l2" }, 1);
	if (IS_ERR(table)) {
		ret = PTR_ERR(table);
		dev_err(dev, "failed to set regulators %d\n", ret);
		return ret;
	}

	ret = PTR_ERR_OR_ZERO(
		dev_pm_opp_register_set_opp_helper(dev, krait_cache_set_opp));
	if (ret)
		return ret;

	ret = dev_pm_opp_of_add_table(dev);
	if (ret) {
		dev_err(dev, "failed to parse L2 freq thresholds\n");
		return ret;
	}

	count = dev_pm_opp_get_opp_count(dev);

	data->l2_cpufreq =
		devm_kzalloc(dev, sizeof(unsigned int) * count, GFP_KERNEL);
	if (!data->l2_cpufreq)
		return -ENOMEM;

	ret = of_property_read_u32_array(node, "l2-cpufreq", data->l2_cpufreq,
					 count);
	if (ret) {
		dev_err(dev, "failed to parse L2 cpufreq thresholds\n");
		return ret;
	}

	ret = devfreq_add_governor(&krait_devfreq_governor);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add governor: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, data);

	data->devfreq = devfreq_add_device(&pdev->dev, &tegra_devfreq_profile,
					   "krait_governor", NULL);

	/* Cache freq_table to quickly get it when needed */
	data->freq_table = data->devfreq->profile->freq_table;

	if (IS_ERR(data->devfreq))
		return PTR_ERR(data->devfreq);

	return 0;
};

static int krait_cache_remove(struct platform_device *pdev)
{
	struct krait_data *data = platform_get_drvdata(pdev);

	dev_pm_opp_remove_table(data->dev);

	return 0;
};

static const struct of_device_id krait_cache_match_table[] = {
	{ .compatible = "qcom,krait-cache" },
	{}
};

static struct platform_driver krait_cache_driver = {
	.probe		= krait_cache_probe,
	.remove		= krait_cache_remove,
	.driver		= {
		.name   = "krait-cache-scaling",
		.of_match_table = krait_cache_match_table,
	},
};
module_platform_driver(krait_cache_driver);

MODULE_DESCRIPTION("Krait CPU Cache Scaling driver");
MODULE_AUTHOR("Ansuel Smith <ansuelsmth@gmail.com>");
MODULE_LICENSE("GPL v2");
