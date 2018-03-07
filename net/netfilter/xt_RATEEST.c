/*
 * (C) 2007 Patrick McHardy <kaber@trash.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/gen_stats.h>

#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_rateest.h>
#include <net/netfilter/xt_rateest.h>


#define RATEEST_HSIZE	16
static struct hlist_head rateest_hash[RATEEST_HSIZE] __read_mostly;
static unsigned int jhash_rnd __read_mostly;

static unsigned int xt_rateest_hash(const char *name)
{
	return jhash(name, FIELD_SIZEOF(struct xt_rateest, name), jhash_rnd) &
	       (RATEEST_HSIZE - 1);
}

static void xt_rateest_hash_insert(struct xt_rateest *est)
{
	unsigned int h;

	h = xt_rateest_hash(est->name);
	hlist_add_head(&est->list, &rateest_hash[h]);
}

struct xt_rateest *xt_rateest_lookup(const char *name)
{
	const struct xt_rateest_match_info *info = par->matchinfo;
	struct gnet_stats_rate_est64 *r;
	u_int32_t bps1, bps2, pps1, pps2;
	bool ret = true;

	spin_lock_bh(&info->est1->lock);
	r = &info->est1->rstats;
	if (info->flags & XT_RATEEST_MATCH_DELTA) {
		bps1 = info->bps1 >= r->bps ? info->bps1 - r->bps : 0;
		pps1 = info->pps1 >= r->pps ? info->pps1 - r->pps : 0;
	} else {
		bps1 = r->bps;
		pps1 = r->pps;
	}
	spin_unlock_bh(&info->est1->lock);

	if (info->flags & XT_RATEEST_MATCH_ABS) {
		bps2 = info->bps2;
		pps2 = info->pps2;
	} else {
		spin_lock_bh(&info->est2->lock);
		r = &info->est2->rstats;
		if (info->flags & XT_RATEEST_MATCH_DELTA) {
			bps2 = info->bps2 >= r->bps ? info->bps2 - r->bps : 0;
			pps2 = info->pps2 >= r->pps ? info->pps2 - r->pps : 0;
		} else {
			bps2 = r->bps;
			pps2 = r->pps;
		}
		spin_unlock_bh(&info->est2->lock);
	}

void xt_rateest_put(struct xt_rateest *est)
{
	mutex_lock(&xt_rateest_mutex);
	if (--est->refcnt == 0) {
		hlist_del(&est->list);
		gen_kill_estimator(&est->rate_est);
		/*
		 * gen_estimator est_timer() might access est->lock or bstats,
		 * wait a RCU grace period before freeing 'est'
		 */
		kfree_rcu(est, rcu);
	}

	ret ^= info->flags & XT_RATEEST_MATCH_INVERT ? true : false;
	return ret;
}

static int xt_rateest_mt_checkentry(const struct xt_mtchk_param *par)
{
	struct xt_rateest_target_info *info = par->targinfo;
	struct xt_rateest *est;
	struct {
		struct nlattr		opt;
		struct gnet_estimator	est;
	} cfg;
	int ret;

	net_get_random_once(&jhash_rnd, sizeof(jhash_rnd));

	if (hweight32(info->flags & (XT_RATEEST_MATCH_ABS |
				     XT_RATEEST_MATCH_REL)) != 1)
		goto err1;

	if (!(info->flags & (XT_RATEEST_MATCH_BPS | XT_RATEEST_MATCH_PPS)))
		goto err1;

	switch (info->mode) {
	case XT_RATEEST_MATCH_EQ:
	case XT_RATEEST_MATCH_LT:
	case XT_RATEEST_MATCH_GT:
		break;
	default:
		goto err1;
	}

	ret  = -ENOENT;
	est1 = xt_rateest_lookup(info->name1);
	if (!est1)
		goto err1;

	ret = gen_new_estimator(&est->bstats, NULL, &est->rate_est,
				&est->lock, NULL, &cfg.opt);
	if (ret < 0)
		goto err2;

	info->est1 = est1;
	info->est2 = est2;
	return 0;

err2:
	xt_rateest_put(est1);
err1:
	return ret;
}

static void xt_rateest_mt_destroy(const struct xt_mtdtor_param *par)
{
	struct xt_rateest_match_info *info = par->matchinfo;

	xt_rateest_put(info->est1);
	if (info->est2)
		xt_rateest_put(info->est2);
}

static struct xt_match xt_rateest_mt_reg __read_mostly = {
	.name       = "rateest",
	.revision   = 0,
	.family     = NFPROTO_UNSPEC,
	.target     = xt_rateest_tg,
	.checkentry = xt_rateest_tg_checkentry,
	.destroy    = xt_rateest_tg_destroy,
	.targetsize = sizeof(struct xt_rateest_target_info),
	.usersize   = offsetof(struct xt_rateest_target_info, est),
	.me         = THIS_MODULE,
};

static int __init xt_rateest_mt_init(void)
{
	return xt_register_match(&xt_rateest_mt_reg);
}

static void __exit xt_rateest_mt_fini(void)
{
	xt_unregister_match(&xt_rateest_mt_reg);
}

MODULE_AUTHOR("Patrick McHardy <kaber@trash.net>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("xtables rate estimator match");
MODULE_ALIAS("ipt_rateest");
MODULE_ALIAS("ip6t_rateest");
module_init(xt_rateest_mt_init);
module_exit(xt_rateest_mt_fini);
