// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PM8110 coulomb counting Battery Monitoring System
 *
 * The state of charge is the remaining charge at the last open circuit
 * voltage (OCV) measurement minus the charge counted by the coulomb counter
 * since then, less the charge that cannot be used before the battery voltage
 * under load drops below the cutoff. The algorithm follows the downstream
 * qpnp-bms driver, Copyright (c) 2011-2014, The Linux Foundation.
 */

#include <linux/alarmtimer.h>
#include <linux/bitfield.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/units.h>
#include <linux/workqueue.h>

/* BMS registers */
#define BMS_PERPH_TYPE			0x04
#define BMS_PERPH_TYPE_BMS		0x0d
#define BMS_PERPH_SUBTYPE_BMS		0x01
#define BMS_CC_DATA_CTL			0x42
#define  BMS_HOLD_OREG_DATA		BIT(0)
#define BMS_CC_DATA			0x8a	/* 36 bits, signed */
#define BMS_OCV_FOR_SOC_DATA		0x90
#define BMS_VSENSE_AVG_DATA		0x98
/* Spare registers, kept over warm resets; layout shared with downstream */
#define BMS_SOC_STORAGE			0xb0
#define  BMS_SOC_STORAGE_MASK		GENMASK(7, 1)
#define  BMS_SOC_STORAGE_INVALID	0x7f
#define BMS_IAVG_STORAGE		0xb1
#define  BMS_IAVG_STORAGE_INVALID	0xff

/* BMS current ADC registers */
#define BMS_IADC_CH_SEL_CTL		0x48
#define  BMS_IADC_CH_SEL_MASK		GENMASK(2, 0)
#define  BMS_IADC_INTERNAL_RSENSE	0
#define BMS_IADC_NOMINAL_RSENSE		0xf4
#define  BMS_IADC_NOMINAL_RSENSE_SIGN	BIT(7)
#define  BMS_IADC_NOMINAL_RSENSE_DEV	GENMASK(6, 0)

#define RSENSE_NOMINAL_NOHM		10000000
#define RSENSE_NOHM_PER_BIT		15625

/* Coulomb counter and VSENSE resolution, at the ideal current ADC gain */
#define CC_RESOLUTION_N			542535
#define CC_RESOLUTION_D			100000
/* The coulomb counter accumulates every 56 sleep clock cycles */
#define CC_READING_TICKS		56
#define SLEEP_CLK_HZ			32764

#define SECS_PER_HOUR			3600

/* The OCV is a voltage ADC code, measured through a 1/3 divider */
#define VBAT_DIVIDER			3
#define VADC_REF_625MV_UV		625000

#define IAVG_SAMPLES			16
#define IAVG_MIN_UA			250000
#define IAVG_MIN_DELTA_S		2
#define IAVG_STORAGE_STEP_UA		10000
#define UUC_SECONDS_PER_PERCENT		60

#define LOW_SOC_THRESHOLD		15
#define LOW_VOLTAGE_MARGIN_UV		20000
#define NO_ADJUST_SOC_THRESHOLD		98
#define HOLD_SOC_EST			3
#define STORE_SOC_MIN_TEMP_DECIC	50

#define CALC_INTERVAL_MS		20000
#define CALC_INTERVAL_LOW_SOC_MS	5000

/* Charging resumes this far below the end threshold, if no start is set */
#define CHARGE_LIMIT_HYSTERESIS		3
/* Wake from suspend this often to stop charging at the end threshold */
#define CHARGE_LIMIT_WAKE_S		(5 * 60)

struct pm8110_bms {
	struct device *dev;
	struct regmap *regmap;
	u32 base;
	u32 iadc_base;
	struct power_supply *psy;
	struct power_supply_battery_info *info;
	struct iio_channel *vbat_chan;
	struct iio_channel *ref_625mv_chan;
	struct iio_channel *ref_1250mv_chan;
	struct iio_channel *iadc_chan;
	struct iio_channel *temp_chan;
	struct delayed_work work;
	struct mutex lock;

	u32 rsense_uohm;
	int fcc_uah;

	/* OCV measurement the remaining charge is based on */
	u16 ocv_raw;
	int ocv_uv;
	/* Remaining charge when the coulomb counter was last zero */
	int ref_charge_uah;

	int cc_uah;
	int last_cc_uah;
	ktime_t last_calc;
	int iavg_samples[IAVG_SAMPLES];
	int iavg_index;
	int iavg_count;
	/* Average current since the last calculation, positive if discharging */
	int iavg_ua;
	/* Average load assumed for the unusable charge */
	int uuc_iavg_ua;
	int pc_unusable;
	int uuc_uah;
	int temp_decic;
	int last_soc_est;

	int calculated_soc;
	int soc;
	int status;
	bool initialized;
	bool unbound;

	/* Charge limit, enforced through the charger */
	struct fwnode_handle *charger_fwnode;
	struct alarm charge_limit_alarm;
	int start_threshold;
	int end_threshold;
	bool charge_inhibited;
};

static int pm8110_bms_read(struct pm8110_bms *bms, u16 reg, void *val,
			   size_t count)
{
	return regmap_bulk_read(bms->regmap, bms->base + reg, val, count);
}

/* Read the OCV, coulomb counter and average sense voltage atomically */
static int pm8110_bms_read_raw(struct pm8110_bms *bms, u16 *ocv_raw,
			       s64 *cc_raw, s16 *vsense_raw)
{
	__le16 ocv, vsense;
	u8 cc[5];
	int ret, ret2;

	ret = regmap_set_bits(bms->regmap, bms->base + BMS_CC_DATA_CTL,
			      BMS_HOLD_OREG_DATA);
	if (ret)
		return ret;

	ret = pm8110_bms_read(bms, BMS_OCV_FOR_SOC_DATA, &ocv, sizeof(ocv));
	if (!ret)
		ret = pm8110_bms_read(bms, BMS_CC_DATA, cc, sizeof(cc));
	if (!ret)
		ret = pm8110_bms_read(bms, BMS_VSENSE_AVG_DATA, &vsense,
				      sizeof(vsense));

	ret2 = regmap_clear_bits(bms->regmap, bms->base + BMS_CC_DATA_CTL,
				 BMS_HOLD_OREG_DATA);
	if (ret)
		return ret;
	if (ret2)
		return ret2;

	if (ocv_raw)
		*ocv_raw = le16_to_cpu(ocv);
	if (cc_raw)
		*cc_raw = sign_extend64((u64)cc[4] << 32 | (u32)cc[3] << 24 |
					cc[2] << 16 | cc[1] << 8 | cc[0], 35);
	if (vsense_raw)
		*vsense_raw = (s16)le16_to_cpu(vsense);

	return 0;
}

/* Convert a coulomb counter or VSENSE code to microvolts */
static int pm8110_bms_code_to_uv(struct pm8110_bms *bms, s64 code, s64 *uv)
{
	int ret, num, den;

	ret = iio_read_channel_attribute(bms->iadc_chan, &num, &den,
					 IIO_CHAN_INFO_CALIBSCALE);
	if (ret < 0)
		return ret;
	if (den <= 0)
		return -EINVAL;

	*uv = div_s64(code * CC_RESOLUTION_N, CC_RESOLUTION_D);
	*uv = div_s64(*uv * num, den);

	return 0;
}

/* Charge counted since the last OCV measurement, positive if discharged */
static int pm8110_bms_cc_to_uah(struct pm8110_bms *bms, s64 cc_raw, int *uah)
{
	s64 uv, pvh;
	int ret;

	ret = pm8110_bms_code_to_uv(bms, cc_raw, &uv);
	if (ret)
		return ret;

	/* Scaled in two steps to not overflow on a full 36-bit counter */
	pvh = div_s64(uv * CC_READING_TICKS * 100000,
		      SLEEP_CLK_HZ * SECS_PER_HOUR) * 10;
	*uah = div_s64(pvh, bms->rsense_uohm);

	return 0;
}

/* Battery current, positive if discharging */
static int pm8110_bms_vsense_to_ua(struct pm8110_bms *bms, s16 vsense_raw,
				   int *ua)
{
	s64 uv;
	int ret;

	ret = pm8110_bms_code_to_uv(bms, vsense_raw, &uv);
	if (ret)
		return ret;

	*ua = div_s64(uv * MICRO, bms->rsense_uohm);

	return 0;
}

static int pm8110_bms_ocv_raw_to_uv(struct pm8110_bms *bms, u16 ocv_raw,
				    int *uv)
{
	int ret, ref_625mv, ref_1250mv;

	/* Calibrate against the voltage ADC references */
	ret = iio_read_channel_raw(bms->ref_625mv_chan, &ref_625mv);
	if (ret < 0)
		return ret;

	ret = iio_read_channel_raw(bms->ref_1250mv_chan, &ref_1250mv);
	if (ret < 0)
		return ret;

	if (ref_1250mv <= ref_625mv)
		return -EINVAL;

	*uv = VADC_REF_625MV_UV + div_s64((s64)(ocv_raw - ref_625mv) *
					  VADC_REF_625MV_UV,
					  ref_1250mv - ref_625mv);
	*uv *= VBAT_DIVIDER;

	return 0;
}

static int pm8110_bms_read_temp(struct pm8110_bms *bms)
{
	int ret, temp;

	if (!bms->temp_chan)
		return 250;

	ret = iio_read_channel_processed(bms->temp_chan, &temp);
	if (ret < 0) {
		dev_warn_ratelimited(bms->dev,
				     "failed to read temperature: %d\n", ret);
		return bms->temp_decic;
	}

	return temp / 100;
}

/* Remaining capacity for an OCV, in hundredths of a percent */
static int pm8110_bms_ocv_to_pc(struct pm8110_bms *bms, int ocv_uv)
{
	const struct power_supply_battery_ocv_table *table;
	int i, len;

	table = power_supply_find_ocv2cap_table(bms->info,
						bms->temp_decic / 10, &len);

	for (i = 0; i < len; i++)
		if (ocv_uv > table[i].ocv)
			break;

	if (i == 0)
		return table[0].capacity * 100;
	if (i == len)
		return table[len - 1].capacity * 100;

	return table[i].capacity * 100 +
	       div_s64((s64)(ocv_uv - table[i].ocv) *
		       (table[i - 1].capacity - table[i].capacity) * 100,
		       table[i - 1].ocv - table[i].ocv);
}

static int pm8110_bms_rbatt_uohm(struct pm8110_bms *bms)
{
	struct power_supply_battery_info *info = bms->info;
	int percent = 100;

	if (info->resist_table)
		percent = power_supply_temp2resist_simple(info->resist_table,
							  info->resist_table_size,
							  bms->temp_decic / 10);

	return info->factory_internal_resistance_uohm / 100 * percent;
}

static int pm8110_bms_soc_from_charge(struct pm8110_bms *bms, int charge_uah)
{
	return DIV_S64_ROUND_CLOSEST((s64)(charge_uah - bms->uuc_uah) * 100,
				     bms->fcc_uah - bms->uuc_uah);
}

static int pm8110_bms_charge_for_soc(struct pm8110_bms *bms, int soc)
{
	return div_s64((s64)soc * (bms->fcc_uah - bms->uuc_uah), 100) +
	       bms->uuc_uah;
}

static void pm8110_bms_update_iavg(struct pm8110_bms *bms, int ibat_ua,
				   int delta_s)
{
	int i, sum = 0;

	if (delta_s >= IAVG_MIN_DELTA_S && bms->last_cc_uah != INT_MIN)
		bms->iavg_ua = div_s64((s64)(bms->cc_uah - bms->last_cc_uah) *
				       SECS_PER_HOUR, delta_s);
	else
		bms->iavg_ua = ibat_ua;

	bms->last_cc_uah = bms->cc_uah;

	if (delta_s < IAVG_MIN_DELTA_S && bms->iavg_count)
		return;

	/* A nominal load keeps the unusable charge sane while charging */
	bms->iavg_samples[bms->iavg_index] = max(bms->iavg_ua, IAVG_MIN_UA);
	bms->iavg_index = (bms->iavg_index + 1) % IAVG_SAMPLES;
	bms->iavg_count = min(bms->iavg_count + 1, IAVG_SAMPLES);

	for (i = 0; i < bms->iavg_count; i++)
		sum += bms->iavg_samples[i] / 1000;

	bms->uuc_iavg_ua = sum / bms->iavg_count * 1000;
}

/*
 * The unusable charge is what is left in the battery when the voltage under
 * the average load drops to the cutoff voltage.
 */
static void pm8110_bms_update_uuc(struct pm8110_bms *bms, int delta_s)
{
	int unusable_uv, pc, max_change;

	unusable_uv = bms->info->voltage_min_design_uv +
		      div_s64((s64)pm8110_bms_rbatt_uohm(bms) * bms->uuc_iavg_ua,
			      MICRO);
	pc = DIV_ROUND_UP(pm8110_bms_ocv_to_pc(bms, unusable_uv), 100);

	/* Let the unusable charge change by about 1% per minute only */
	max_change = max(delta_s / UUC_SECONDS_PER_PERCENT, 1);
	if (bms->pc_unusable >= 0)
		pc = clamp(pc, bms->pc_unusable - max_change,
			   bms->pc_unusable + max_change);

	bms->pc_unusable = pc;
	bms->uuc_uah = div_s64((s64)bms->fcc_uah * pc, 100);
}

static int pm8110_bms_get_status(struct pm8110_bms *bms)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property_from_supplier(bms->psy,
						      POWER_SUPPLY_PROP_STATUS,
						      &val);
	if (ret)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	return val.intval;
}

/*
 * Near empty, pull the coulomb counted state of charge towards the one
 * estimated from the battery voltage, so it reaches 0% around the cutoff.
 */
static int pm8110_bms_adjust_low_soc(struct pm8110_bms *bms, int soc,
				     int vbat_uv, int ibat_ua)
{
	int ocv_est_uv, soc_est, n, delta_uah, limit_uah;
	bool low_voltage;

	ocv_est_uv = vbat_uv + div_s64((s64)ibat_ua *
				       pm8110_bms_rbatt_uohm(bms), MICRO);
	soc_est = pm8110_bms_soc_from_charge(bms,
		div_s64((s64)bms->fcc_uah *
			pm8110_bms_ocv_to_pc(bms, ocv_est_uv), 10000));
	soc_est = clamp(soc_est, 0, 100);

	low_voltage = vbat_uv < bms->info->voltage_min_design_uv +
				LOW_VOLTAGE_MARGIN_UV;

	if (!low_voltage && (soc_est == soc || soc_est > LOW_SOC_THRESHOLD ||
			     soc >= NO_ADJUST_SOC_THRESHOLD)) {
		bms->last_soc_est = soc_est;
		return soc;
	}

	if (bms->last_soc_est < 0)
		bms->last_soc_est = soc;

	n = clamp(soc + soc_est + bms->last_soc_est, 1, 200);
	bms->last_soc_est = soc_est;

	delta_uah = div_s64((s64)(soc - soc_est) *
			    (bms->fcc_uah - bms->uuc_uah), 100 * n);

	if (low_voltage) {
		/* Never correct upwards when about to cut off */
		delta_uah = max(delta_uah, 0);
	} else {
		limit_uah = bms->fcc_uah / 1000;
		delta_uah = clamp(delta_uah, -limit_uah, limit_uah);
	}

	bms->ref_charge_uah -= delta_uah;
	soc = pm8110_bms_soc_from_charge(bms, bms->ref_charge_uah -
					 bms->cc_uah);
	soc = clamp(soc, 0, 100);

	/* Report 0% only when the voltage agrees */
	if (soc == 0 && soc_est >= HOLD_SOC_EST)
		soc = 1;

	dev_dbg(bms->dev, "low soc: vbat %d ibat %d soc_est %d delta %d soc %d\n",
		vbat_uv, ibat_ua, soc_est, delta_uah, soc);

	return soc;
}

static void pm8110_bms_store_soc(struct pm8110_bms *bms)
{
	unsigned int soc = BMS_SOC_STORAGE_INVALID, iavg = 0;

	/* The capacity at cold temperatures is not trustworthy over a reboot */
	if (bms->temp_decic > STORE_SOC_MIN_TEMP_DECIC)
		soc = bms->soc + 1;

	if (bms->uuc_iavg_ua > IAVG_MIN_UA)
		iavg = min((bms->uuc_iavg_ua - IAVG_MIN_UA) / IAVG_STORAGE_STEP_UA,
			   BMS_IAVG_STORAGE_INVALID - 1);

	regmap_update_bits(bms->regmap, bms->base + BMS_SOC_STORAGE,
			   BMS_SOC_STORAGE_MASK,
			   FIELD_PREP(BMS_SOC_STORAGE_MASK, soc));
	regmap_write(bms->regmap, bms->base + BMS_IAVG_STORAGE, iavg);
}

/* Returns the state of charge saved before the last reset, if any */
static int pm8110_bms_load_soc(struct pm8110_bms *bms)
{
	unsigned int val;
	int ret, soc;

	ret = regmap_read(bms->regmap, bms->base + BMS_SOC_STORAGE, &val);
	if (ret)
		return ret;

	soc = FIELD_GET(BMS_SOC_STORAGE_MASK, val) - 1;
	if (soc < 0 || soc > 100)
		return -ENODATA;

	ret = regmap_read(bms->regmap, bms->base + BMS_IAVG_STORAGE, &val);
	if (!ret && val != BMS_IAVG_STORAGE_INVALID) {
		for (bms->iavg_count = 0; bms->iavg_count < IAVG_SAMPLES;
		     bms->iavg_count++)
			bms->iavg_samples[bms->iavg_count] =
				IAVG_MIN_UA + val * IAVG_STORAGE_STEP_UA;
		bms->uuc_iavg_ua = bms->iavg_samples[0];
	}

	return soc;
}

static void pm8110_bms_calculate(struct pm8110_bms *bms)
{
	int ret, vbat_uv, ibat_ua, soc, delta_s, status, stored_soc = -1;
	s16 vsense_raw;
	u16 ocv_raw;
	s64 cc_raw;
	ktime_t now;

	ret = pm8110_bms_read_raw(bms, &ocv_raw, &cc_raw, &vsense_raw);
	if (!ret)
		ret = pm8110_bms_cc_to_uah(bms, cc_raw, &bms->cc_uah);
	if (!ret)
		ret = pm8110_bms_vsense_to_ua(bms, vsense_raw, &ibat_ua);
	if (!ret)
		ret = iio_read_channel_processed(bms->vbat_chan, &vbat_uv);
	if (ret < 0) {
		dev_err(bms->dev, "failed to read battery state: %d\n", ret);
		return;
	}

	bms->temp_decic = pm8110_bms_read_temp(bms);

	now = ktime_get_boottime();
	delta_s = div_s64(ktime_ms_delta(now, bms->last_calc), MSEC_PER_SEC);
	bms->last_calc = now;

	if (!bms->initialized)
		stored_soc = pm8110_bms_load_soc(bms);

	/* The coulomb counter restarts with each new OCV measurement */
	if (!bms->initialized || ocv_raw != bms->ocv_raw) {
		ret = pm8110_bms_ocv_raw_to_uv(bms, ocv_raw, &bms->ocv_uv);
		if (ret) {
			dev_err(bms->dev, "failed to convert OCV: %d\n", ret);
			return;
		}

		bms->ocv_raw = ocv_raw;
		bms->ref_charge_uah = div_s64((s64)bms->fcc_uah *
				pm8110_bms_ocv_to_pc(bms, bms->ocv_uv), 10000);
		bms->last_cc_uah = INT_MIN;
		bms->unbound = true;

		dev_dbg(bms->dev, "new OCV %d uV, remaining charge %d uAh\n",
			bms->ocv_uv, bms->ref_charge_uah);
	}

	pm8110_bms_update_iavg(bms, ibat_ua, delta_s);
	pm8110_bms_update_uuc(bms, delta_s);

	/*
	 * The OCV may be stale after a warm reset or when another OS cleared
	 * the coulomb counter, so continue from the saved state of charge.
	 */
	if (stored_soc >= 0)
		bms->ref_charge_uah = pm8110_bms_charge_for_soc(bms, stored_soc) +
				      bms->cc_uah;

	status = pm8110_bms_get_status(bms);
	if (status == POWER_SUPPLY_STATUS_FULL)
		bms->ref_charge_uah = bms->fcc_uah + bms->cc_uah;

	soc = pm8110_bms_soc_from_charge(bms, bms->ref_charge_uah - bms->cc_uah);
	soc = clamp(soc, 0, 100);

	if (status != POWER_SUPPLY_STATUS_CHARGING &&
	    status != POWER_SUPPLY_STATUS_FULL)
		soc = pm8110_bms_adjust_low_soc(bms, soc, vbat_uv, ibat_ua);

	if (soc == 0 && vbat_uv > bms->info->voltage_min_design_uv)
		soc = 1;
	if (soc == 100 && status == POWER_SUPPLY_STATUS_CHARGING)
		soc = 99;

	bms->calculated_soc = soc;

	/* Smooth the reported value, unless enough time passed */
	if (bms->initialized && !bms->unbound) {
		if (soc > bms->soc && status != POWER_SUPPLY_STATUS_CHARGING &&
		    status != POWER_SUPPLY_STATUS_FULL)
			soc = bms->soc;
		soc = clamp(soc, bms->soc - 1, bms->soc + 1);
	}

	dev_dbg(bms->dev,
		"ocv %d cc %d ref %d uuc %d iavg %d/%d ibat %d vbat %d temp %d status %d soc %d/%d\n",
		bms->ocv_uv, bms->cc_uah, bms->ref_charge_uah, bms->uuc_uah,
		bms->iavg_ua, bms->uuc_iavg_ua, ibat_ua, vbat_uv,
		bms->temp_decic, status, bms->calculated_soc, soc);

	if (!bms->initialized || soc != bms->soc || status != bms->status) {
		bms->soc = soc;
		bms->status = status;
		power_supply_changed(bms->psy);
	}

	pm8110_bms_store_soc(bms);
	bms->initialized = true;
	bms->unbound = false;
}

struct pm8110_bms_charger_data {
	struct fwnode_handle *fwnode;
	union power_supply_propval val;
	int ret;
};

static int pm8110_bms_set_charger_behaviour(struct power_supply *psy,
					    void *data)
{
	struct pm8110_bms_charger_data *d = data;

	if (psy->dev.fwnode != d->fwnode)
		return 0;

	d->ret = power_supply_set_property(psy,
					   POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
					   &d->val);
	return 1;
}

static int pm8110_bms_inhibit_charge(struct pm8110_bms *bms, bool inhibit)
{
	struct pm8110_bms_charger_data d = {
		.fwnode = bms->charger_fwnode,
		.val.intval = inhibit ? POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
					POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO,
		.ret = -ENODEV,
	};

	if (!d.fwnode)
		return -ENODEV;

	power_supply_for_each_psy(&d, pm8110_bms_set_charger_behaviour);

	return d.ret;
}

/*
 * Stop charging at the end threshold and resume below the start threshold.
 * Nothing measures the charge while the system sleeps, so wake it up
 * periodically while charging towards the limit.
 */
static void pm8110_bms_charge_limit(struct pm8110_bms *bms)
{
	int start = bms->start_threshold;
	bool inhibit = false;
	int ret;

	if (start <= 0 || start >= bms->end_threshold)
		start = bms->end_threshold - CHARGE_LIMIT_HYSTERESIS;

	if (bms->end_threshold < 100)
		inhibit = bms->soc >= bms->end_threshold ||
			  (bms->charge_inhibited && bms->soc > start);

	if (inhibit != bms->charge_inhibited) {
		ret = pm8110_bms_inhibit_charge(bms, inhibit);
		if (ret)
			dev_warn(bms->dev, "failed to %s charging: %d\n",
				 inhibit ? "inhibit" : "resume", ret);
		else
			bms->charge_inhibited = inhibit;
	}

	if (bms->end_threshold < 100 && !bms->charge_inhibited &&
	    bms->status == POWER_SUPPLY_STATUS_CHARGING)
		alarm_start_timer(&bms->charge_limit_alarm,
				  ktime_set(CHARGE_LIMIT_WAKE_S, 0), true);
	else
		alarm_try_to_cancel(&bms->charge_limit_alarm);
}

static void pm8110_bms_charge_limit_alarm(struct alarm *alarm, ktime_t now)
{
	struct pm8110_bms *bms = container_of(alarm, struct pm8110_bms,
					      charge_limit_alarm);

	mod_delayed_work(system_freezable_power_efficient_wq, &bms->work, 0);
}

static void pm8110_bms_work(struct work_struct *work)
{
	struct pm8110_bms *bms = container_of(work, struct pm8110_bms,
					      work.work);
	unsigned int delay;

	mutex_lock(&bms->lock);
	pm8110_bms_calculate(bms);
	if (bms->initialized)
		pm8110_bms_charge_limit(bms);
	delay = bms->soc > LOW_SOC_THRESHOLD ? CALC_INTERVAL_MS :
					       CALC_INTERVAL_LOW_SOC_MS;
	mutex_unlock(&bms->lock);

	queue_delayed_work(system_freezable_power_efficient_wq, &bms->work,
			   msecs_to_jiffies(delay));
}

static int pm8110_bms_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct pm8110_bms *bms = power_supply_get_drvdata(psy);
	struct power_supply_battery_info *info = bms->info;
	s16 vsense_raw;
	int ret = 0;

	mutex_lock(&bms->lock);

	if (!bms->initialized) {
		ret = -ENODATA;
		goto out;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bms->status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = info->technology;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bms->soc;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = iio_read_channel_processed(bms->vbat_chan, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = bms->ocv_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = info->voltage_min_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = pm8110_bms_read_raw(bms, NULL, NULL, &vsense_raw);
		if (!ret)
			ret = pm8110_bms_vsense_to_ua(bms, vsense_raw,
						      &val->intval);
		val->intval = -val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = -bms->iavg_ua;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bms->fcc_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = clamp(bms->ref_charge_uah - bms->cc_uah, 0,
				    bms->fcc_uah);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bms->temp_decic;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = bms->start_threshold;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = bms->end_threshold;
		break;
	default:
		ret = -EINVAL;
		break;
	}

out:
	mutex_unlock(&bms->lock);
	return ret;
}

static int pm8110_bms_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct pm8110_bms *bms = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		if (val->intval < 0 || val->intval > 100)
			return -EINVAL;
		mutex_lock(&bms->lock);
		bms->start_threshold = val->intval;
		mutex_unlock(&bms->lock);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		if (val->intval < 1 || val->intval > 100)
			return -EINVAL;
		if (!bms->charger_fwnode)
			return -EOPNOTSUPP;
		mutex_lock(&bms->lock);
		bms->end_threshold = val->intval;
		mutex_unlock(&bms->lock);
		break;
	default:
		return -EINVAL;
	}

	if (READ_ONCE(bms->fcc_uah))
		mod_delayed_work(system_freezable_power_efficient_wq,
				 &bms->work, 0);

	return 0;
}

static int pm8110_bms_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
}

static void pm8110_bms_external_power_changed(struct power_supply *psy)
{
	struct pm8110_bms *bms = power_supply_get_drvdata(psy);

	/* Calculations start once the battery info is known */
	if (READ_ONCE(bms->fcc_uah))
		mod_delayed_work(system_freezable_power_efficient_wq,
				 &bms->work, 0);
}

static const enum power_supply_property pm8110_bms_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_desc pm8110_bms_desc = {
	.name = "pm8110-bms",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pm8110_bms_properties,
	.num_properties = ARRAY_SIZE(pm8110_bms_properties),
	.get_property = pm8110_bms_get_property,
	.set_property = pm8110_bms_set_property,
	.property_is_writeable = pm8110_bms_property_is_writeable,
	.external_power_changed = pm8110_bms_external_power_changed,
};

static void pm8110_bms_release_charger(void *data)
{
	struct pm8110_bms *bms = data;

	if (bms->charge_inhibited)
		pm8110_bms_inhibit_charge(bms, false);

	fwnode_handle_put(bms->charger_fwnode);
}

static void pm8110_bms_cancel_alarm(void *data)
{
	struct pm8110_bms *bms = data;

	alarm_cancel(&bms->charge_limit_alarm);
}

static int pm8110_bms_read_rsense(struct pm8110_bms *bms)
{
	unsigned int val;
	int ret, dev;

	ret = regmap_read(bms->regmap, bms->iadc_base + BMS_IADC_CH_SEL_CTL,
			  &val);
	if (ret)
		return ret;

	if (FIELD_GET(BMS_IADC_CH_SEL_MASK, val) != BMS_IADC_INTERNAL_RSENSE)
		return dev_err_probe(bms->dev, -EINVAL,
				     "only the internal sense resistor is supported\n");

	/* Trimmed deviation from the nominal value, in sign-magnitude */
	ret = regmap_read(bms->regmap, bms->iadc_base + BMS_IADC_NOMINAL_RSENSE,
			  &val);
	if (ret)
		return ret;

	dev = FIELD_GET(BMS_IADC_NOMINAL_RSENSE_DEV, val) * RSENSE_NOHM_PER_BIT;
	if (val & BMS_IADC_NOMINAL_RSENSE_SIGN)
		dev = -dev;

	bms->rsense_uohm = (RSENSE_NOMINAL_NOHM + dev) / 1000;

	return 0;
}

static int pm8110_bms_check_battery_info(struct pm8110_bms *bms)
{
	struct power_supply_battery_info *info = bms->info;

	if (info->charge_full_design_uah <= 0 ||
	    info->voltage_min_design_uv <= 0 ||
	    info->factory_internal_resistance_uohm <= 0 ||
	    !info->ocv_table[0])
		return dev_err_probe(bms->dev, -EINVAL,
				     "battery capacity, minimum voltage, resistance and OCV tables are required\n");

	return 0;
}

static struct iio_channel *pm8110_bms_get_chan(struct pm8110_bms *bms,
					       const char *name)
{
	struct iio_channel *chan = devm_iio_channel_get(bms->dev, name);

	if (IS_ERR(chan))
		dev_err_probe(bms->dev, PTR_ERR(chan),
			      "failed to get %s channel\n", name);

	return chan;
}

static int pm8110_bms_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {};
	struct pm8110_bms *bms;
	u32 reg[2];
	u8 type[2];
	int ret;

	bms = devm_kzalloc(dev, sizeof(*bms), GFP_KERNEL);
	if (!bms)
		return -ENOMEM;

	bms->dev = dev;
	bms->pc_unusable = -1;
	bms->last_soc_est = -1;
	bms->last_cc_uah = INT_MIN;
	bms->temp_decic = 250;
	bms->status = POWER_SUPPLY_STATUS_UNKNOWN;
	platform_set_drvdata(pdev, bms);

	bms->regmap = dev_get_regmap(dev->parent, NULL);
	if (!bms->regmap)
		return -ENODEV;

	ret = device_property_read_u32_array(dev, "reg", reg, ARRAY_SIZE(reg));
	if (ret)
		return ret;

	bms->base = reg[0];
	bms->iadc_base = reg[1];

	ret = regmap_bulk_read(bms->regmap, bms->base + BMS_PERPH_TYPE, type,
			       sizeof(type));
	if (ret)
		return ret;

	if (type[0] != BMS_PERPH_TYPE_BMS || type[1] != BMS_PERPH_SUBTYPE_BMS)
		return dev_err_probe(dev, -ENODEV,
				     "unknown peripheral type %#x/%#x\n",
				     type[0], type[1]);

	ret = pm8110_bms_read_rsense(bms);
	if (ret)
		return ret;

	bms->vbat_chan = pm8110_bms_get_chan(bms, "vbat");
	if (IS_ERR(bms->vbat_chan))
		return PTR_ERR(bms->vbat_chan);

	bms->ref_625mv_chan = pm8110_bms_get_chan(bms, "ref_625mv");
	if (IS_ERR(bms->ref_625mv_chan))
		return PTR_ERR(bms->ref_625mv_chan);

	bms->ref_1250mv_chan = pm8110_bms_get_chan(bms, "ref_1250mv");
	if (IS_ERR(bms->ref_1250mv_chan))
		return PTR_ERR(bms->ref_1250mv_chan);

	bms->iadc_chan = pm8110_bms_get_chan(bms, "iadc");
	if (IS_ERR(bms->iadc_chan))
		return PTR_ERR(bms->iadc_chan);

	bms->temp_chan = devm_iio_channel_get(dev, "temp");
	if (IS_ERR(bms->temp_chan)) {
		ret = PTR_ERR(bms->temp_chan);
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret,
					     "failed to get temp channel\n");
		bms->temp_chan = NULL;
	}

	ret = devm_mutex_init(dev, &bms->lock);
	if (ret)
		return ret;

	bms->end_threshold = 100;
	bms->charger_fwnode = fwnode_find_reference(dev_fwnode(dev),
						    "power-supplies", 0);
	if (IS_ERR(bms->charger_fwnode))
		bms->charger_fwnode = NULL;

	/* Runs after the work and the alarm are cancelled */
	ret = devm_add_action_or_reset(dev, pm8110_bms_release_charger, bms);
	if (ret)
		return ret;

	ret = devm_delayed_work_autocancel(dev, &bms->work, pm8110_bms_work);
	if (ret)
		return ret;

	alarm_init(&bms->charge_limit_alarm, ALARM_BOOTTIME,
		   pm8110_bms_charge_limit_alarm);
	ret = devm_add_action_or_reset(dev, pm8110_bms_cancel_alarm, bms);
	if (ret)
		return ret;

	cfg.drv_data = bms;
	cfg.fwnode = dev_fwnode(dev);

	bms->psy = devm_power_supply_register(dev, &pm8110_bms_desc, &cfg);
	if (IS_ERR(bms->psy))
		return dev_err_probe(dev, PTR_ERR(bms->psy),
				     "failed to register battery\n");

	ret = power_supply_get_battery_info(bms->psy, &bms->info);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get battery info\n");

	ret = pm8110_bms_check_battery_info(bms);
	if (ret)
		return ret;

	WRITE_ONCE(bms->fcc_uah, bms->info->charge_full_design_uah);

	queue_delayed_work(system_freezable_power_efficient_wq, &bms->work, 0);

	return 0;
}

static int pm8110_bms_resume(struct device *dev)
{
	struct pm8110_bms *bms = dev_get_drvdata(dev);

	/* Let the reported value follow whatever happened during suspend */
	mutex_lock(&bms->lock);
	bms->unbound = true;
	mutex_unlock(&bms->lock);

	mod_delayed_work(system_freezable_power_efficient_wq, &bms->work, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(pm8110_bms_pm_ops, NULL, pm8110_bms_resume);

static const struct of_device_id pm8110_bms_of_match[] = {
	{ .compatible = "qcom,pm8110-bms" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8110_bms_of_match);

static struct platform_driver pm8110_bms_driver = {
	.probe = pm8110_bms_probe,
	.driver = {
		.name = "pm8110-bms",
		.of_match_table = pm8110_bms_of_match,
		.pm = pm_sleep_ptr(&pm8110_bms_pm_ops),
	},
};
module_platform_driver(pm8110_bms_driver);

MODULE_DESCRIPTION("Qualcomm PM8110 coulomb counting BMS driver");
MODULE_LICENSE("GPL");
