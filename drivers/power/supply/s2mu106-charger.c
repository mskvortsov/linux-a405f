// SPDX-License-Identifier: GPL-2.0
/*
 * Battery Charger Driver for Samsung S2MU106 PMIC.
 *
 * MVP port: covers input/fast-charge current, float voltage, top-off
 * current and status/health/charge-type reporting. Deliberately leaves
 * out OTG boost, AFC/HV negotiation and wireless (WCIN) charging - those
 * depend on the MUIC and USB-PD blocks of the same PMIC, which aren't
 * wired up in this tree. Without adapter-type negotiation, charging stays
 * at the configured default input current regardless of what is plugged
 * in (i.e. no fast-charge speeds).
 *
 * Two things can gate the charge-enable bit, ANDed together in
 * s2mu106_chg_apply_state(): a manual on/off toggle via
 * POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR (AUTO enables, INHIBIT_CHARGE disconnects
 * the charge path, not just starves it - see s2mu106_chg_desc.charge_behaviours),
 * and POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, which stops charging above a
 * configured capacity. There's no capacity-crossing IRQ from this chip, so the
 * limit is enforced by polling the fuel gauge's CAPACITY periodically. Absent
 * either gate, the chip only actually draws current once it sees a valid VBUS
 * on its own, so there's nothing else to gate without cable-type detection.
 *
 * POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE is also implemented, but it's
 * best-effort, not true reverse power flow: this chip has no register that
 * disconnects VBUS or reverses the buck (checked the downstream driver and
 * header for an input-suspend/HZ bit - there isn't one; OTG boost powers
 * *external* devices, it doesn't drop the system's own input path either).
 * What it does instead is drop the input current limit to the IC's 50 mA
 * floor via s2mu106_chg_set_input_current_limit() - the same register
 * POWER_SUPPLY_PROP_CURRENT_MAX already writes - so VBUS can no longer cover
 * system load and the deficit is made up from the battery. That's a real
 * negative battery current, but it's created by starving the input, not by
 * making the chip discharge on purpose: if system draw ever drops below the
 * 50 mA floor (e.g. deep suspend), the "discharge" stalls. To keep that
 * failure mode bounded, s2mu106_chg_apply_force_discharge_safety_exit() backs
 * out to AUTO (restoring the saved input limit) on VBUS removal, suspend and
 * driver unbind, and entry is refused below S2MU106_FORCE_DISCHARGE_MIN_CAPACITY.
 *
 * Register layout and the vendor init sequence in s2mu106_chg_hw_init()
 * are taken from Samsung's downstream s2mu106_charger.c; most of the
 * init writes are undocumented (no public datasheet) and are kept
 * verbatim rather than reinterpreted, except for the OTG-only bits
 * (fault debounce, boost OCP), which are dropped along with OTG support.
 *
 * Copyright (C) 2016 Samsung Electronics
 * Copyright (C) 2026 <your name here>
 */

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/units.h>
#include <linux/workqueue.h>

/* Chip-wide IRQ demux mask (also on this i2c address). */
#define S2MU106_REG_IPINT_MASK		0x07
#define S2MU106_IPINT_CHG		BIT(6)

#define S2MU106_REG_CHG_INT1		0x01
#define S2MU106_REG_CHG_INT2		0x02
#define S2MU106_REG_CHG_INT3		0x03
#define S2MU106_REG_CHG_INT1M		0x08
#define S2MU106_REG_CHG_INT2M		0x09
#define S2MU106_REG_CHG_INT3M		0x0a

#define S2MU106_REG_CHG_STATUS0	0x0e
#define S2MU106_REG_CHG_STATUS1	0x0f
#define S2MU106_REG_CHG_STATUS3	0x11

#define S2MU106_REG_CHG_CTRL0		0x18
#define S2MU106_REG_CHG_CTRL1		0x19
#define S2MU106_REG_CHG_CTRL5		0x1d
#define S2MU106_REG_CHG_CTRL7		0x1f
#define S2MU106_REG_CHG_CTRL10		0x22
#define S2MU106_REG_CHG_CTRL12		0x24
/* Buck switching mode (async/sync); no named register block downstream. */
#define S2MU106_REG_BUCK_SW_MODE	0x3a

/* S2MU106_REG_CHG_CTRL0 */
#define S2MU106_CTRL0_MODE_MASK	GENMASK(3, 0)
#define S2MU106_MODE_OFF		0x0
#define S2MU106_MODE_BUCK		0x1
#define S2MU106_MODE_CHG		0x3

/* S2MU106_REG_CHG_CTRL1 */
#define S2MU106_CTRL1_ICHGIN_MASK	GENMASK(6, 0)

/* S2MU106_REG_CHG_CTRL5 */
#define S2MU106_CTRL5_VFVBAT_MASK	GENMASK(6, 0)

/* S2MU106_REG_CHG_CTRL7 */
#define S2MU106_CTRL7_IFAST_MASK	GENMASK(5, 0)

/* S2MU106_REG_CHG_CTRL10 */
#define S2MU106_CTRL10_ITOPOFF_MASK	GENMASK(7, 4)

/* S2MU106_REG_CHG_STATUS0: raw status bits, undocumented beyond CHGIN[2:0]. */
#define S2MU106_STATUS0_CHGIN_MASK	GENMASK(2, 0)
#define S2MU106_CHGIN_VALID_1		0x3
#define S2MU106_CHGIN_VALID_2		0x5

/* S2MU106_REG_CHG_STATUS1 */
#define S2MU106_STATUS1_DONE		BIT(0)
#define S2MU106_STATUS1_TOPOFF		BIT(1)
#define S2MU106_STATUS1_NO_INPUT	BIT(7)
#define S2MU106_STATUS1_FAULT_MASK	GENMASK(7, 4)
#define S2MU106_STATUS1_FAULT_SHIFT	4
#define S2MU106_FAULT_NORMAL		0
#define S2MU106_FAULT_WD_SUSPEND	1
#define S2MU106_FAULT_WD_RST		2
#define S2MU106_FAULT_TSD		3

/* S2MU106_REG_CHG_STATUS3 */
#define S2MU106_STATUS3_DET_BAT	BIT(0)
#define S2MU106_STATUS3_BAT_MASK	GENMASK(3, 1)
#define S2MU106_STATUS3_BAT_SHIFT	1

/* S2MU106_REG_CHG_CTRL12 */
#define S2MU106_CTRL12_WDT_CLEAR	BIT(0)
#define S2MU106_CTRL12_WDT_TIME_MASK	GENMASK(3, 1)
#define S2MU106_CTRL12_WDT_TIME_80S	5
#define S2MU106_CTRL12_WDT_EN		BIT(4)
#define S2MU106_CTRL12_WDT_AP_RESET	BIT(5)

/* Conversion limits, from the downstream set_*() helpers. */
#define S2MU106_ICHGIN_MIN_MA		50
#define S2MU106_ICHGIN_MAX_MA		3000
#define S2MU106_IFAST_MIN_MA		100
#define S2MU106_IFAST_MAX_MA		3200
#define S2MU106_VFVBAT_MIN_MV		3900
#define S2MU106_VFVBAT_MAX_MV		4530
#define S2MU106_ITOPOFF_MIN_MA		100
#define S2MU106_ITOPOFF_MAX_MA		475

/* Fallbacks if "monitored-battery" doesn't supply these. */
#define S2MU106_DEF_VFVBAT_UV		4200000
#define S2MU106_DEF_IFAST_UA		1000000
#define S2MU106_DEF_ITOPOFF_UA		150000
/* No adapter-type negotiation (no MUIC); stay at a USB2-safe input limit. */
#define S2MU106_DEF_ICHGIN_UA		500000

/* No IRQ tells us when capacity crosses the limit, so poll for it. */
#define S2MU106_CHARGE_LIMIT_POLL_MS	30000
/* Resume charging once capacity drops this many percent below the limit. */
#define S2MU106_CHARGE_LIMIT_HYSTERESIS	3
#define S2MU106_CHARGE_LIMIT_DISABLED	100

/*
 * Below this capacity, refuse to enter FORCE_DISCHARGE: it's best-effort
 * (see the file comment) and can stall without warning if system draw ever
 * dips under the 50 mA floor, so don't let it run the battery to empty.
 */
#define S2MU106_FORCE_DISCHARGE_MIN_CAPACITY	20

struct s2mu106_chg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	int topoff_ua;
	struct delayed_work charge_limit_work;
	int charge_limit;
	bool user_enabled;	/* manual on/off via POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR */
	bool limit_enabled;	/* automatic gate from charge_limit_work */
	bool charging_enabled;	/* cached actual hardware state (CHG vs BUCK) */
	bool force_discharge;	/* input pinned to the IC floor, see the file comment */
	int saved_input_ua;	/* input limit to restore when force_discharge ends */
};

static const struct regmap_config s2mu106_chg_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static void s2mu106_chg_set_input_current_limit(struct s2mu106_chg *priv, int ua)
{
	int ma = ua / (MICRO / MILLI);
	u8 data;

	if (ma <= S2MU106_ICHGIN_MIN_MA)
		data = 0x00;
	else if (ma <= 100)
		data = 0x02;
	else if (ma <= S2MU106_ICHGIN_MAX_MA)
		data = (ma - 50) / 25;
	else
		data = 0x62;

	regmap_update_bits(priv->regmap, S2MU106_REG_CHG_CTRL1,
			    S2MU106_CTRL1_ICHGIN_MASK, data);
}

static int s2mu106_chg_get_input_current_limit(struct s2mu106_chg *priv, int *ua)
{
	unsigned int data;
	int ret;

	ret = regmap_read(priv->regmap, S2MU106_REG_CHG_CTRL1, &data);
	if (ret < 0)
		return ret;

	data &= S2MU106_CTRL1_ICHGIN_MASK;
	*ua = (data * 25 + 50) * (MICRO / MILLI);

	return 0;
}

static void s2mu106_chg_set_fast_charge_current(struct s2mu106_chg *priv, int ua)
{
	int ma = ua / (MICRO / MILLI);
	u8 data;

	if (ma <= S2MU106_IFAST_MIN_MA)
		data = 0x01;
	else if (ma <= S2MU106_IFAST_MAX_MA)
		data = (ma / 50) - 1;
	else
		data = 0x3d;

	regmap_update_bits(priv->regmap, S2MU106_REG_CHG_CTRL7,
			    S2MU106_CTRL7_IFAST_MASK, data);
}

static int s2mu106_chg_get_fast_charge_current(struct s2mu106_chg *priv, int *ua)
{
	unsigned int data;
	int ret;

	ret = regmap_read(priv->regmap, S2MU106_REG_CHG_CTRL7, &data);
	if (ret < 0)
		return ret;

	data &= S2MU106_CTRL7_IFAST_MASK;
	*ua = (data + 1) * 50 * (MICRO / MILLI);

	return 0;
}

static void s2mu106_chg_set_float_voltage(struct s2mu106_chg *priv, int uv)
{
	int mv = uv / (MICRO / MILLI);
	u8 data;

	if (mv <= S2MU106_VFVBAT_MIN_MV)
		data = 0;
	else if (mv <= S2MU106_VFVBAT_MAX_MV)
		data = (mv - S2MU106_VFVBAT_MIN_MV) / 5;
	else
		data = 0x7f;

	regmap_update_bits(priv->regmap, S2MU106_REG_CHG_CTRL5,
			    S2MU106_CTRL5_VFVBAT_MASK, data);
}

static int s2mu106_chg_get_float_voltage(struct s2mu106_chg *priv, int *uv)
{
	unsigned int data;
	int ret;

	ret = regmap_read(priv->regmap, S2MU106_REG_CHG_CTRL5, &data);
	if (ret < 0)
		return ret;

	data &= S2MU106_CTRL5_VFVBAT_MASK;
	*uv = (data * 5 + S2MU106_VFVBAT_MIN_MV) * (MICRO / MILLI);

	return 0;
}

static void s2mu106_chg_set_topoff_current(struct s2mu106_chg *priv, int ua)
{
	int ma = ua / (MICRO / MILLI);
	u8 data;

	if (ma <= S2MU106_ITOPOFF_MIN_MA)
		data = 0;
	else if (ma <= S2MU106_ITOPOFF_MAX_MA)
		data = (ma - 100) / 25;
	else
		data = 0x0f;

	regmap_update_bits(priv->regmap, S2MU106_REG_CHG_CTRL10,
			    S2MU106_CTRL10_ITOPOFF_MASK,
			    data << 4);
}

static bool s2mu106_chg_is_input_valid(struct s2mu106_chg *priv)
{
	unsigned int data;

	if (regmap_read(priv->regmap, S2MU106_REG_CHG_STATUS0, &data) < 0)
		return false;

	data &= S2MU106_STATUS0_CHGIN_MASK;

	return data == S2MU106_CHGIN_VALID_1 || data == S2MU106_CHGIN_VALID_2;
}

static bool s2mu106_chg_get_present(struct s2mu106_chg *priv)
{
	unsigned int data;

	if (regmap_read(priv->regmap, S2MU106_REG_CHG_STATUS3, &data) < 0)
		return false;

	return data & S2MU106_STATUS3_DET_BAT;
}

/*
 * Gates the charge block itself (CHG vs BUCK-only mode - not just current
 * starvation, the charge path is actually disconnected). Two independent
 * things can each veto charging: the user's manual on/off toggle and the
 * charge_limit poller; only run when both agree it should be on.
 */
static void s2mu106_chg_apply_state(struct s2mu106_chg *priv)
{
	bool enable = priv->user_enabled && priv->limit_enabled;

	if (priv->charging_enabled == enable)
		return;

	regmap_update_bits(priv->regmap, S2MU106_REG_CHG_CTRL0, S2MU106_CTRL0_MODE_MASK,
			    enable ? S2MU106_MODE_CHG : S2MU106_MODE_BUCK);
	priv->charging_enabled = enable;
}

static int s2mu106_chg_get_capacity(struct s2mu106_chg *priv, int *capacity)
{
	struct power_supply *fg;
	union power_supply_propval value;
	int ret;

	fg = power_supply_get_by_name("s2mu106-fuel-gauge");
	if (!fg)
		return -ENODEV;

	ret = power_supply_get_property(fg, POWER_SUPPLY_PROP_CAPACITY, &value);
	power_supply_put(fg);
	if (ret < 0)
		return ret;

	*capacity = value.intval;
	return 0;
}

/* Undoes the input-limit floor from force_discharge; leaves user_enabled alone. */
static void s2mu106_chg_restore_input_limit(struct s2mu106_chg *priv)
{
	if (!priv->force_discharge)
		return;

	s2mu106_chg_set_input_current_limit(priv, priv->saved_input_ua);
	priv->force_discharge = false;
}

static int s2mu106_chg_enter_force_discharge(struct s2mu106_chg *priv)
{
	int capacity, ret;

	ret = s2mu106_chg_get_capacity(priv, &capacity);
	if (!ret && capacity <= S2MU106_FORCE_DISCHARGE_MIN_CAPACITY) {
		dev_warn(priv->dev,
			 "refusing force-discharge at %d%% capacity (floor is %d%%)\n",
			 capacity, S2MU106_FORCE_DISCHARGE_MIN_CAPACITY);
		return -EBUSY;
	}

	if (!priv->force_discharge) {
		ret = s2mu106_chg_get_input_current_limit(priv, &priv->saved_input_ua);
		if (ret < 0)
			return ret;
	}

	priv->force_discharge = true;
	s2mu106_chg_set_input_current_limit(priv, S2MU106_ICHGIN_MIN_MA * (MICRO / MILLI));

	return 0;
}

/*
 * Safety net: back out to AUTO (charging re-enabled, input limit restored)
 * rather than leaving the input pinned at the IC floor with nothing
 * watching it. Called on VBUS removal, suspend and driver unbind.
 */
static void s2mu106_chg_force_discharge_safety_exit(struct s2mu106_chg *priv)
{
	if (!priv->force_discharge)
		return;

	s2mu106_chg_restore_input_limit(priv);
	priv->user_enabled = true;
	s2mu106_chg_apply_state(priv);
}

/*
 * Kicks the charger's own watchdog (it will otherwise stop charging on
 * timeout) and, if it had already fired, restores the charge block to
 * whatever state we currently want it in.
 */
static void s2mu106_chg_wdt_clear(struct s2mu106_chg *priv)
{
	unsigned int status1;

	regmap_update_bits(priv->regmap, S2MU106_REG_CHG_CTRL12,
			    S2MU106_CTRL12_WDT_CLEAR, S2MU106_CTRL12_WDT_CLEAR);

	if (regmap_read(priv->regmap, S2MU106_REG_CHG_STATUS1, &status1) < 0)
		return;

	status1 = (status1 & S2MU106_STATUS1_FAULT_MASK) >> S2MU106_STATUS1_FAULT_SHIFT;
	if (status1 == S2MU106_FAULT_WD_SUSPEND || status1 == S2MU106_FAULT_WD_RST) {
		regmap_update_bits(priv->regmap, S2MU106_REG_CHG_CTRL0,
				    S2MU106_CTRL0_MODE_MASK, S2MU106_MODE_BUCK);
		regmap_update_bits(priv->regmap, S2MU106_REG_CHG_CTRL0,
				    S2MU106_CTRL0_MODE_MASK,
				    priv->charging_enabled ? S2MU106_MODE_CHG : S2MU106_MODE_BUCK);
	}
}

/*
 * No IRQ tells us when capacity crosses the configured limit, so poll the
 * fuel gauge periodically. Hysteresis avoids rapidly toggling the charge
 * block right at the threshold.
 */
static void s2mu106_chg_limit_work(struct work_struct *work)
{
	struct s2mu106_chg *priv = container_of(work, struct s2mu106_chg,
						 charge_limit_work.work);
	struct power_supply *fg;
	union power_supply_propval value;

	if (priv->force_discharge && !s2mu106_chg_is_input_valid(priv))
		s2mu106_chg_force_discharge_safety_exit(priv);

	if (priv->charge_limit >= S2MU106_CHARGE_LIMIT_DISABLED) {
		priv->limit_enabled = true;
		s2mu106_chg_apply_state(priv);
		goto reschedule;
	}

	fg = power_supply_get_by_name("s2mu106-fuel-gauge");
	if (!fg)
		goto reschedule;

	if (power_supply_get_property(fg, POWER_SUPPLY_PROP_CAPACITY, &value) == 0) {
		if (value.intval >= priv->charge_limit)
			priv->limit_enabled = false;
		else if (value.intval <= priv->charge_limit - S2MU106_CHARGE_LIMIT_HYSTERESIS)
			priv->limit_enabled = true;
		s2mu106_chg_apply_state(priv);
	}
	power_supply_put(fg);

reschedule:
	schedule_delayed_work(&priv->charge_limit_work,
			       msecs_to_jiffies(S2MU106_CHARGE_LIMIT_POLL_MS));
}

static void s2mu106_chg_cancel_limit_work(void *data)
{
	struct s2mu106_chg *priv = data;

	cancel_delayed_work_sync(&priv->charge_limit_work);
}

static void s2mu106_chg_force_discharge_exit_action(void *data)
{
	s2mu106_chg_force_discharge_safety_exit(data);
}

static int s2mu106_chg_get_status(struct s2mu106_chg *priv)
{
	struct power_supply *fg;
	union power_supply_propval value;
	unsigned int status1;
	int ret;

	/*
	 * We've deliberately put the chip in BUCK-only mode; trust that over
	 * whatever the (partly undocumented) status bits say in this state.
	 */
	if (!priv->charging_enabled)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	ret = regmap_read(priv->regmap, S2MU106_REG_CHG_STATUS1, &status1);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	if (status1 & S2MU106_STATUS1_NO_INPUT)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	fg = power_supply_get_by_name("s2mu106-fuel-gauge");
	if (!fg)
		return POWER_SUPPLY_STATUS_CHARGING;

	/*
	 * CURRENT_NOW, not CURRENT_AVG: measured live, current_avg sits
	 * frozen around +4.6mA on this chip regardless of whether the
	 * battery is actually charging or idle - see the comment on
	 * s2mu106_fg_get_status() in s2mu106-battery.c. A threshold check
	 * against it here would report CHARGING almost unconditionally.
	 */
	ret = power_supply_get_property(fg, POWER_SUPPLY_PROP_CURRENT_NOW, &value);
	power_supply_put(fg);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_CHARGING;

	if (status1 & (S2MU106_STATUS1_TOPOFF | S2MU106_STATUS1_DONE))
		/* Tapered below the top-off threshold: charging is done. */
		return value.intval < priv->topoff_ua ? POWER_SUPPLY_STATUS_FULL :
							 POWER_SUPPLY_STATUS_CHARGING;

	/*
	 * Bulk/CC charging has no dedicated status bit of its own on this
	 * chip - only NO_INPUT and TOPOFF/DONE are documented, and STATUS0
	 * carries no independently verifiable "charging in progress" field
	 * (a previous version of this code guessed one from undocumented
	 * magic values ported alongside the vendor init sequence; it went
	 * stale against a live device that was genuinely charging per the
	 * fuel gauge). Current actually flowing into the battery is ground
	 * truth instead - see s2mu106_fg_get_status()'s use of the same
	 * signal in s2mu106-battery.c.
	 */
	return value.intval > 0 ? POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int s2mu106_chg_get_charge_type(struct s2mu106_chg *priv)
{
	unsigned int status3;
	int bat_status;

	if (regmap_read(priv->regmap, S2MU106_REG_CHG_STATUS3, &status3) < 0)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	bat_status = (status3 & S2MU106_STATUS3_BAT_MASK) >> S2MU106_STATUS3_BAT_SHIFT;

	switch (bat_status) {
	case 0x2:
	case 0x3:
	case 0x6:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int s2mu106_chg_get_health(struct s2mu106_chg *priv)
{
	unsigned int status1;
	int fault;

	s2mu106_chg_wdt_clear(priv);

	if (regmap_read(priv->regmap, S2MU106_REG_CHG_STATUS1, &status1) < 0)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	fault = (status1 & S2MU106_STATUS1_FAULT_MASK) >> S2MU106_STATUS1_FAULT_SHIFT;

	switch (fault) {
	case S2MU106_FAULT_TSD:
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	case S2MU106_FAULT_WD_SUSPEND:
	case S2MU106_FAULT_WD_RST:
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	default:
		return POWER_SUPPLY_HEALTH_GOOD;
	}
}

static irqreturn_t s2mu106_chg_handle_irq(int irq, void *data)
{
	struct s2mu106_chg *priv = data;
	unsigned int int1, int2, int3;

	/* Reading INT1-3 acknowledges and clears their pending bits. */
	regmap_read(priv->regmap, S2MU106_REG_CHG_INT1, &int1);
	regmap_read(priv->regmap, S2MU106_REG_CHG_INT2, &int2);
	regmap_read(priv->regmap, S2MU106_REG_CHG_INT3, &int3);

	power_supply_changed(priv->psy);

	return IRQ_HANDLED;
}

static const enum power_supply_property s2mu106_chg_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
};

static int s2mu106_chg_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct s2mu106_chg *priv = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = s2mu106_chg_get_status(priv);
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = s2mu106_chg_is_input_valid(priv);
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = s2mu106_chg_get_present(priv);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = s2mu106_chg_get_charge_type(priv);
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = s2mu106_chg_get_health(priv);
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return s2mu106_chg_get_input_current_limit(priv, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return s2mu106_chg_get_fast_charge_current(priv, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		return s2mu106_chg_get_float_voltage(priv, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = priv->charge_limit;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = S2MU106_CHARGE_LIMIT_DISABLED;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (priv->force_discharge)
			val->intval = POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE;
		else
			val->intval = priv->user_enabled ? POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO :
							    POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s2mu106_chg_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *val)
{
	struct s2mu106_chg *priv = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		/*
		 * The core's sysfs store doesn't validate against what we
		 * advertise in charge_behaviours (that bitmask is only
		 * consulted for the sysfs "available" listing on show) -
		 * a raw integer write skips text matching entirely, so this
		 * switch is the only thing rejecting unsupported/bogus values.
		 */
		switch (val->intval) {
		case POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO:
			s2mu106_chg_restore_input_limit(priv);
			priv->user_enabled = true;
			break;
		case POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE:
			s2mu106_chg_restore_input_limit(priv);
			priv->user_enabled = false;
			break;
		case POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE: {
			int ret = s2mu106_chg_enter_force_discharge(priv);

			if (ret)
				return ret;
			priv->user_enabled = false;
			break;
		}
		default:
			return -EINVAL;
		}
		s2mu106_chg_apply_state(priv);
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		s2mu106_chg_set_input_current_limit(priv, val->intval);
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		s2mu106_chg_set_fast_charge_current(priv, val->intval);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		if (val->intval < 1 || val->intval > S2MU106_CHARGE_LIMIT_DISABLED)
			return -EINVAL;
		priv->charge_limit = val->intval;
		mod_delayed_work(system_wq, &priv->charge_limit_work, 0);
		return 0;
	default:
		return -EINVAL;
	}
}

static int s2mu106_chg_property_is_writeable(struct power_supply *psy,
					      enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		return 1;
	default:
		return 0;
	}
}

static const struct power_supply_desc s2mu106_chg_desc = {
	.name = "s2mu106-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	/* FORCE_DISCHARGE is best-effort (input-limit floor) - see the file comment. */
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			      BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE) |
			      BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE),
	.properties = s2mu106_chg_properties,
	.num_properties = ARRAY_SIZE(s2mu106_chg_properties),
	.get_property = s2mu106_chg_get_property,
	.set_property = s2mu106_chg_set_property,
	.property_is_writeable = s2mu106_chg_property_is_writeable,
};

/*
 * Vendor init sequence ported verbatim from downstream's s2mu106_chg_init(),
 * minus the OTG-only writes (boost OCP, OTG fault debounce). Most of these
 * registers are undocumented outside this sequence - no public datasheet.
 */
static void s2mu106_chg_hw_init(struct s2mu106_chg *priv)
{
	struct regmap *rm = priv->regmap;

	regmap_update_bits(rm, 0x8b, BIT(4), 0x00);

	/* Don't clear the watchdog before enabling it, to avoid a reset loop. */
	regmap_update_bits(rm, S2MU106_REG_CHG_CTRL12, S2MU106_CTRL12_WDT_CLEAR, 0x00);
	regmap_update_bits(rm, S2MU106_REG_CHG_CTRL12, S2MU106_CTRL12_WDT_TIME_MASK,
			    S2MU106_CTRL12_WDT_TIME_80S << 1);
	/* Watchdog timeout stops charging only, it doesn't reset the AP. */
	regmap_update_bits(rm, S2MU106_REG_CHG_CTRL12,
			    S2MU106_CTRL12_WDT_EN | S2MU106_CTRL12_WDT_AP_RESET,
			    S2MU106_CTRL12_WDT_EN);

	/* ICR disable */
	regmap_update_bits(rm, 0x7d, BIT(1), BIT(1));

	/* Type-C reset off */
	regmap_update_bits(rm, 0xec, BIT(7), 0x00);

	/* MRSTB 1s */
	regmap_write(rm, 0xe5, 0x08);

	/* 3-level buck OCP tuning */
	regmap_update_bits(rm, 0x82, 0xf0, 0xf0);
	regmap_write(rm, 0xa3, 0x72);
	regmap_write(rm, 0xa4, 0x32);

	/* Switching ramp delay 128us */
	regmap_update_bits(rm, 0x92, 0x0f, 0x05);

	regmap_update_bits(rm, 0xa6, 0x0f, 0x00);

	/* Prevent sudden power-off on a false water-detect trip. */
	regmap_update_bits(rm, 0x88, BIT(5), BIT(5));
	regmap_write(rm, 0xf3, 0x00);
	regmap_update_bits(rm, 0x8c, BIT(7), 0x00);
	regmap_update_bits(rm, 0x90, BIT(2), 0x00);

	/* Max charging by frequency change when duty is maxed out. */
	regmap_update_bits(rm, 0x7a, BIT(4), BIT(4));
}

static int s2mu106_chg_apply_battery_info(struct s2mu106_chg *priv)
{
	struct power_supply_battery_info *info;
	int voltage_uv = S2MU106_DEF_VFVBAT_UV;
	int current_ua = S2MU106_DEF_IFAST_UA;
	int topoff_ua = S2MU106_DEF_ITOPOFF_UA;
	int ret;

	ret = power_supply_get_battery_info(priv->psy, &info);
	if (ret == -ENOMEM)
		return ret;

	if (!ret) {
		if (info->constant_charge_voltage_max_uv > 0)
			voltage_uv = info->constant_charge_voltage_max_uv;
		if (info->constant_charge_current_max_ua > 0)
			current_ua = info->constant_charge_current_max_ua;
		if (info->charge_term_current_ua > 0)
			topoff_ua = info->charge_term_current_ua;
		power_supply_put_battery_info(priv->psy, info);
	} else {
		dev_warn(priv->dev, "no monitored-battery info, using defaults\n");
	}

	s2mu106_chg_set_float_voltage(priv, voltage_uv);
	s2mu106_chg_set_fast_charge_current(priv, current_ua);
	s2mu106_chg_set_topoff_current(priv, topoff_ua);
	s2mu106_chg_set_input_current_limit(priv, S2MU106_DEF_ICHGIN_UA);
	priv->topoff_ua = topoff_ua;

	return 0;
}

static int s2mu106_chg_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s2mu106_chg *priv;
	struct power_supply_config psy_cfg = {};
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	priv->regmap = devm_regmap_init_i2c(client, &s2mu106_chg_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to initialize regmap\n");

	psy_cfg.drv_data = priv;
	psy_cfg.fwnode = dev_fwnode(dev);
	priv->psy = devm_power_supply_register(dev, &s2mu106_chg_desc, &psy_cfg);
	if (IS_ERR(priv->psy))
		return dev_err_probe(dev, PTR_ERR(priv->psy),
				     "failed to register power supply subsystem\n");

	s2mu106_chg_hw_init(priv);

	ret = s2mu106_chg_apply_battery_info(priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read battery info\n");

	priv->charge_limit = S2MU106_CHARGE_LIMIT_DISABLED;
	priv->user_enabled = true;
	priv->limit_enabled = true;
	s2mu106_chg_apply_state(priv);
	regmap_update_bits(priv->regmap, S2MU106_REG_BUCK_SW_MODE, 0x03, 0x01);

	INIT_DELAYED_WORK(&priv->charge_limit_work, s2mu106_chg_limit_work);
	ret = devm_add_action_or_reset(dev, s2mu106_chg_cancel_limit_work, priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register cleanup action\n");
	schedule_delayed_work(&priv->charge_limit_work,
			       msecs_to_jiffies(S2MU106_CHARGE_LIMIT_POLL_MS));

	/* Don't leave the input pinned at the floor if we're unbound mid force-discharge. */
	ret = devm_add_action_or_reset(dev, s2mu106_chg_force_discharge_exit_action, priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register cleanup action\n");

	/*
	 * This IRQ line is shared with the MUIC/AFC/PM/haptic-boost blocks
	 * of the same PMIC at the top-level demux; none of those are wired
	 * up here, so mask them off and only unmask the charger sources.
	 */
	regmap_write(priv->regmap, S2MU106_REG_IPINT_MASK, (u8)~S2MU106_IPINT_CHG);
	regmap_write(priv->regmap, S2MU106_REG_CHG_INT1M, 0x00);
	regmap_write(priv->regmap, S2MU106_REG_CHG_INT2M, 0x00);
	regmap_write(priv->regmap, S2MU106_REG_CHG_INT3M, 0x00);

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					 s2mu106_chg_handle_irq, IRQF_ONESHOT,
					 s2mu106_chg_desc.name, priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	return 0;
}

/* Don't rely on the poll worker while suspended: back out of force_discharge now. */
static int s2mu106_chg_suspend(struct device *dev)
{
	s2mu106_chg_force_discharge_safety_exit(dev_get_drvdata(dev));

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s2mu106_chg_pm_ops, s2mu106_chg_suspend, NULL);

static const struct of_device_id s2mu106_chg_of_match_table[] = {
	{ .compatible = "samsung,s2mu106-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mu106_chg_of_match_table);

static struct i2c_driver s2mu106_chg_i2c_driver = {
	.probe = s2mu106_chg_i2c_probe,
	.driver = {
		.name = "s2mu106-charger",
		.of_match_table = s2mu106_chg_of_match_table,
		.pm = pm_ptr(&s2mu106_chg_pm_ops),
	},
};
module_i2c_driver(s2mu106_chg_i2c_driver);

MODULE_DESCRIPTION("Samsung S2MU106 PMIC Battery Charger Driver");
MODULE_LICENSE("GPL");
