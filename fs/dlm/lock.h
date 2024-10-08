/* SPDX-License-Identifier: GPL-2.0-only */
/******************************************************************************
*******************************************************************************
**
**  Copyright (C) 2005-2007 Red Hat, Inc.  All rights reserved.
**
**
*******************************************************************************
******************************************************************************/

#ifndef __LOCK_DOT_H__
#define __LOCK_DOT_H__

void dlm_dump_rsb(struct dlm_rsb *r);
void dlm_dump_rsb_name(struct dlm_ls *ls, const char *name, int len);
void dlm_print_lkb(struct dlm_lkb *lkb);
void dlm_receive_message_saved(struct dlm_ls *ls, const struct dlm_message *ms,
			       uint32_t saved_seq);
void dlm_receive_buffer(const union dlm_packet *p, int nodeid);
int dlm_modes_compat(int mode1, int mode2);
void free_inactive_rsb(struct dlm_rsb *r);
void dlm_put_rsb(struct dlm_rsb *r);
void dlm_hold_rsb(struct dlm_rsb *r);
int dlm_put_lkb(struct dlm_lkb *lkb);
int dlm_lock_recovery_try(struct dlm_ls *ls);
void dlm_lock_recovery(struct dlm_ls *ls);
void dlm_unlock_recovery(struct dlm_ls *ls);
void dlm_rsb_scan(struct timer_list *timer);
void resume_scan_timer(struct dlm_ls *ls);

int dlm_master_lookup(struct dlm_ls *ls, int from_nodeid, const char *name,
		      int len, unsigned int flags, int *r_nodeid, int *result);

int dlm_search_rsb_tree(struct rhashtable *rhash, const void *name, int len,
			struct dlm_rsb **r_ret);

void dlm_recover_purge(struct dlm_ls *ls, const struct list_head *root_list);
void dlm_purge_mstcpy_locks(struct dlm_rsb *r);
void dlm_recover_grant(struct dlm_ls *ls);
int dlm_recover_waiters_post(struct dlm_ls *ls);
void dlm_recover_waiters_pre(struct dlm_ls *ls);
int dlm_recover_master_copy(struct dlm_ls *ls, const struct dlm_rcom *rc,
			    __le32 *rl_remid, __le32 *rl_result);
int dlm_recover_process_copy(struct dlm_ls *ls, const struct dlm_rcom *rc,
			     uint64_t seq);

int dlm_user_request(struct dlm_ls *ls, struct dlm_user_args *ua, int mode,
	uint32_t flags, void *name, unsigned int namelen);
int dlm_user_convert(struct dlm_ls *ls, struct dlm_user_args *ua_tmp,
	int mode, uint32_t flags, uint32_t lkid, char *lvb_in);
int dlm_user_adopt_orphan(struct dlm_ls *ls, struct dlm_user_args *ua_tmp,
	int mode, uint32_t flags, void *name, unsigned int namelen,
	uint32_t *lkid);
int dlm_user_unlock(struct dlm_ls *ls, struct dlm_user_args *ua_tmp,
	uint32_t flags, uint32_t lkid, char *lvb_in);
int dlm_user_cancel(struct dlm_ls *ls,  struct dlm_user_args *ua_tmp,
	uint32_t flags, uint32_t lkid);
int dlm_user_purge(struct dlm_ls *ls, struct dlm_user_proc *proc,
	int nodeid, int pid);
int dlm_user_deadlock(struct dlm_ls *ls, uint32_t flags, uint32_t lkid);
void dlm_clear_proc_locks(struct dlm_ls *ls, struct dlm_user_proc *proc);
int dlm_debug_add_lkb(struct dlm_ls *ls, uint32_t lkb_id, char *name, int len,
		      int lkb_nodeid, unsigned int lkb_flags, int lkb_status);
int dlm_debug_add_lkb_to_waiters(struct dlm_ls *ls, uint32_t lkb_id,
				 int mstype, int to_nodeid);

static inline int is_master(struct dlm_rsb *r)
{
	WARN_ON_ONCE(r->res_nodeid == -1);

	return !r->res_nodeid;
}

static inline void lock_rsb(struct dlm_rsb *r)
{
	spin_lock_bh(&r->res_lock);
}

static inline void unlock_rsb(struct dlm_rsb *r)
{
	spin_unlock_bh(&r->res_lock);
}

#endif

