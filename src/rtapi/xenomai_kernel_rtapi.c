#include "config.h"	

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>		/* replaces malloc.h in recent kernels */
#include <linux/ctype.h>	/* isdigit */
#include <linux/delay.h>	/* udelay */
#include <asm/uaccess.h>	/* copy_from_user() */
#include <asm/msr.h>		/* rdtscll() */

#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#endif
#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/cpumask.h>	/* NR_CPUS, cpu_online() */
#endif

#include <native/heap.h>
#include <native/task.h>
#include <native/intr.h>
#include  <rtdk.h>

#include "rtapi.h"		/* public RTAPI decls */
#include "rtapi_common.h"	/* shared realtime/nonrealtime stuff */

static RT_HEAP master_heap;
static RT_HEAP shmem_heap_array[RTAPI_MAX_SHMEMS + 1];        
static rthal_trap_handler_t old_trap_handler;
static int rtapi_trap_handler(unsigned event, rthal_pipeline_stage_t *stage, void *data);

static void *shmem_addr_array[RTAPI_MAX_SHMEMS + 1];

/* module parameters */

/* other module information */
MODULE_AUTHOR("John Kasunich, Fred Proctor, Paul Corner & Michael Haberler");
MODULE_DESCRIPTION("Portable Real Time API for RTAI and Xenomai");
MODULE_LICENSE("GPL");


#include "rtapi_proc.h"		/* proc filesystem decls & code */

/* the following are internal functions that do the real work associated
   with deleting tasks, etc.  They do not check the mutex that protects
   the internal data structures.  When someone calls an rtapi_xxx_delete()
   function, the rtapi funct gets the mutex before calling one of these
   internal functions.  When internal code that already has the mutex
   needs to delete something, it calls these functions directly.
*/
static int module_delete(int module_id);
static int shmem_delete(int shmem_id, int module_id);

/***********************************************************************
*                   INIT AND SHUTDOWN FUNCTIONS                        *
************************************************************************/

int init_module(void)
{
    int n;

    /* say hello */
    rtapi_print_msg(RTAPI_MSG_INFO, "RTAPI: Init\n");
    /* get master shared memory block from OS and save its address */
    if ((n = rt_heap_create(&master_heap, MASTER_HEAP, 
			    sizeof(rtapi_data_t), H_SHARED)) != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "RTAPI: ERROR: rt_heap_create() returns %d\n", n);
	return -EINVAL;
    }
    if ((n = rt_heap_alloc(&master_heap, 0, TM_INFINITE , (void **)&rtapi_data)) != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "RTAPI: ERROR: rt_heap_alloc() returns %d\n", n);
	return -EINVAL;
    }
    /* perform a global init if needed */
    init_rtapi_data(rtapi_data);
    /* check revision code */
    if (rtapi_data->rev_code != REV_CODE) {
	/* mismatch - release master shared memory block */
	rtapi_print_msg(RTAPI_MSG_ERR, "RTAPI: ERROR: version mismatch %d vs %d\n", rtapi_data->rev_code, REV_CODE);

	rt_heap_delete(&master_heap);
	return -EINVAL;
    }
    /* set up local pointers to global data */
    module_array = rtapi_data->module_array;
    task_array = rtapi_data->task_array;
    shmem_array = rtapi_data->shmem_array;

    /* perform local init */
    for (n = 0; n <= RTAPI_MAX_TASKS; n++) {
	ostask_array[n] = NULL;
    }
    for (n = 0; n <= RTAPI_MAX_SHMEMS; n++) {
	shmem_addr_array[n] = NULL;
    }
    rtapi_data->timer_running = 0;
    rtapi_data->timer_period = 0;
    max_delay = DEFAULT_MAX_DELAY;

    /* rt_linux_use_fpu informs the scheduler that floating point arithmetic */
    /* operations will be used also by foreground Linux processes, i.e. the */
    /* Linux kernel itself (unlikely) and any of its processes. */

#if defined(RTAPI_RTAI)
    // FIXME unsure how this relates to Xenomai
    rt_linux_use_fpu(1);
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    /* on SMP machines, we want to put RT code on the last CPU */
    n = NR_CPUS-1;
    while ( ! cpu_online(n) ) {
	n--;
    }
    rtapi_data->rt_cpu = n;
#else
    /* old kernel, the SMP hooks aren't available, so use CPU 0 */
    rtapi_data->rt_cpu = 0;
#endif


#ifdef CONFIG_PROC_FS
    /* set up /proc/rtapi */
    if (proc_init() != 0) {
	rtapi_print_msg(RTAPI_MSG_WARN,
	    "RTAPI: WARNING: Could not activate /proc entries\n");
	proc_clean();
    }
#endif

    old_trap_handler = rthal_trap_catch((rthal_trap_handler_t) rtapi_trap_handler);

    /* done */
    rtapi_print_msg(RTAPI_MSG_INFO, "RTAPI: Init complete\n");
    return 0;
}

/* This cleanup code attempts to fix any messes left by modules
that fail to load properly, or fail to clean up after themselves */

void cleanup_module(void)
{
    int n;

    if (rtapi_data == NULL) {
	/* never got inited, nothing to do */
	return;
    }
    /* grab the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    rtapi_print_msg(RTAPI_MSG_INFO, "RTAPI: Exiting\n");

    /* clean up leftover modules (start at 1, we don't use ID 0 */
    for (n = 1; n <= RTAPI_MAX_MODULES; n++) {
	if (module_array[n].state == REALTIME) {
	    rtapi_print_msg(RTAPI_MSG_WARN,
		"RTAPI: WARNING: module '%s' (ID: %02d) did not call rtapi_exit()\n",
		module_array[n].name, n);
	    module_delete(n);
	}
    }
    /* cleaning up modules should clean up everything, if not there has
       probably been an unrecoverable internal error.... */
 
    for (n = 1; n <= RTAPI_MAX_SHMEMS; n++) {
	if (shmem_array[n].rtusers > 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"RTAPI: ERROR: shared memory block %02d not deleted\n", n);
	}
    }
    for (n = 1; n <= RTAPI_MAX_TASKS; n++) {
	if (task_array[n].state != EMPTY) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"RTAPI: ERROR: task %02d not deleted\n", n);
	    /* probably un-recoverable, but try anyway */
	    rtapi_task_pause(n);
	    task_array[n].state = DELETE_LOCKED;
	    rtapi_task_delete(n);
	}
    }
    if (rtapi_data->timer_running != 0) {
	rtapi_data->timer_period = 0;
	timer_counts = 0;
	rtapi_data->timer_running = 0;
	max_delay = DEFAULT_MAX_DELAY;
    }
    rtapi_mutex_give(&(rtapi_data->mutex));
#ifdef CONFIG_PROC_FS
    proc_clean();
#endif
    /* release master shared memory block */
    rt_heap_delete(&master_heap);
    rthal_trap_catch(old_trap_handler);

    rtapi_print_msg(RTAPI_MSG_INFO, "RTAPI: Exit complete\n");
    return;
}

/***********************************************************************
*                   GENERAL PURPOSE FUNCTIONS                          *
************************************************************************/

/* all RTAPI init is done when the rtapi kernel module
is insmoded.  The rtapi_init() and rtapi_exit() functions
simply register that another module is using the RTAPI.
For other RTOSes, things might be different, especially
if the RTOS does not use modules. */

int rtapi_init(const char *modname)
{
    int n, module_id;
    module_data *module;

    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: initing module %s\n", modname);
    /* get the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    /* find empty spot in module array */
    n = 1;
    while ((n <= RTAPI_MAX_MODULES) && (module_array[n].state != NO_MODULE)) {
	n++;
    }
    if (n > RTAPI_MAX_MODULES) {
	/* no room */
	rtapi_mutex_give(&(rtapi_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR, "RTAPI: ERROR: reached module limit %d\n",
	    n);
	return -EMFILE;
    }
    /* we have space for the module */
    module_id = n;
    module = &(module_array[n]);
    /* update module data */
    module->state = REALTIME;
    if (modname != NULL) {
	/* use name supplied by caller, truncating if needed */
	rtapi_snprintf(module->name, RTAPI_NAME_LEN, "%s", modname);
    } else {
	/* make up a name */
	rtapi_snprintf(module->name, RTAPI_NAME_LEN, "RTMOD%03d", module_id);
    }
    rtapi_data->rt_module_count++;
    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: module '%s' loaded, ID: %d\n",
	module->name, module_id);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return module_id;
}

int rtapi_exit(int module_id)
{
    int retval;

    rtapi_mutex_get(&(rtapi_data->mutex));
    retval = module_delete(module_id);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return retval;
}

static int module_delete(int module_id)
{
    module_data *module;
    char name[RTAPI_NAME_LEN + 1];
    int n;

    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: module %d exiting\n", module_id);
    /* validate module ID */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	return -EINVAL;
    }
    /* point to the module's data */
    module = &(module_array[module_id]);
    /* check module status */
    if (module->state != REALTIME) {
	/* not an active realtime module */
	return -EINVAL;
    }
    /* clean up any mess left behind by the module */
    for (n = 1; n <= RTAPI_MAX_TASKS; n++) {
	if ((task_array[n].state != EMPTY)
	    && (task_array[n].owner == module_id)) {
	    rtapi_print_msg(RTAPI_MSG_WARN,
		"RTAPI: WARNING: module '%s' failed to delete task %02d\n",
		module->name, n);
	    task_array[n].state = DELETE_LOCKED;
	    rtapi_task_delete(n);
	}
    }
    for (n = 1; n <= RTAPI_MAX_SHMEMS; n++) {
	if (test_bit(module_id, shmem_array[n].bitmap)) {
	    rtapi_print_msg(RTAPI_MSG_WARN,
		"RTAPI: WARNING: module '%s' failed to delete shmem %02d\n",
		module->name, n);
	    shmem_delete(n, module_id);
	}
    }
 
    /* use snprintf() to do strncpy(), since we don't have string.h */
    rtapi_snprintf(name, RTAPI_NAME_LEN, "%s", module->name);
    /* update module data */
    module->state = NO_MODULE;
    module->name[0] = '\0';
    rtapi_data->rt_module_count--;
    if (rtapi_data->rt_module_count == 0) {
	if (rtapi_data->timer_running != 0) {
	    rtapi_data->timer_period = 0;
	    timer_counts = 0;
	    max_delay = DEFAULT_MAX_DELAY;
	    rtapi_data->timer_running = 0;
	}
    }
    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: module %d exited, name: '%s'\n",
	module_id, name);
    return 0;
}

/***********************************************************************
*                  SHARED MEMORY RELATED FUNCTIONS                     *
************************************************************************/

int rtapi_shmem_new(int key, int module_id, unsigned long int size)
{
    int n;
    int shmem_id;
    shmem_data *shmem;
    char shm_name[20];

    /* key must be non-zero, and also cannot match the key that RTAPI uses */
    if ((key == 0) || (key == RTAPI_KEY)) {
	return -EINVAL;
    }
    /* get the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    /* validate module_id */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return -EINVAL;
    }
    if (module_array[module_id].state != REALTIME) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return -EINVAL;
    }

    /* check if a block is already open for this key */
    for (n = 1; n <= RTAPI_MAX_SHMEMS; n++) {
	if (shmem_array[n].key == key) {
	    /* found a match */
	    shmem_id = n;
	    shmem = &(shmem_array[n]);
	    /* is it big enough? */
	    if (shmem->size < size) {
		rtapi_mutex_give(&(rtapi_data->mutex));
		return -EINVAL;
	    }
	    /* yes, has it been mapped into kernel space? */
	    if (shmem->rtusers == 0) {
		/* no, map it and save the address */

		rtapi_print_msg(RTAPI_MSG_ERR, 
				"RTAPI: UNSUPPORTED OPERATION - cannot map user segment %d into kernel\n",n);
		rtapi_mutex_give(&(rtapi_data->mutex));
		return -ENOMEM;
	    }
	    /* is this module already using it? */
	    if (test_bit(module_id, shmem->bitmap)) {
		rtapi_mutex_give(&(rtapi_data->mutex));
		return -EINVAL;
	    }
	    /* update usage data */
	    set_bit(module_id, shmem->bitmap);
	    shmem->rtusers++;
	    /* announce another user for this shmem */
	    rtapi_print_msg(RTAPI_MSG_DBG,
		"RTAPI: shmem %02d opened by module %02d\n",
		shmem_id, module_id);
	    rtapi_mutex_give(&(rtapi_data->mutex));
	    return shmem_id;
	}
    }
    /* find empty spot in shmem array */
    n = 1;
    while ((n <= RTAPI_MAX_SHMEMS) && (shmem_array[n].key != 0)) {
	n++;
    }
    if (n > RTAPI_MAX_SHMEMS) {
	/* no room */
	rtapi_mutex_give(&(rtapi_data->mutex));
	return -EMFILE;
    }
    /* we have space for the block data */
    shmem_id = n;
    shmem = &(shmem_array[n]);

    /* get shared memory block from OS and save its address */

    snprintf(shm_name, sizeof(shm_name), "shm-%d", shmem_id);
    if ((n = rt_heap_create(&shmem_heap_array[shmem_id], shm_name, 
			    size, H_SHARED)) != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "RTAPI: ERROR: rt_heap_create() returns %d\n", n);
	return -EINVAL;
    }
    if ((n = rt_heap_alloc(&shmem_heap_array[shmem_id], 0, TM_INFINITE , 
			   (void **)&shmem_addr_array[shmem_id])) != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "RTAPI: ERROR: rt_heap_alloc() returns %d\n", n);
	return -EINVAL;
    }

    if (shmem_addr_array[shmem_id] == NULL) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return -ENOMEM;
    }
    /* the block has been created, update data */
    set_bit(module_id, shmem->bitmap);
    shmem->key = key;
    shmem->rtusers = 1;
    shmem->ulusers = 0;
    shmem->size = size;
    rtapi_data->shmem_count++;
    /* zero the first word of the shmem area */
    *((long int *) (shmem_addr_array[shmem_id])) = 0;
    /* announce the birth of a brand new baby shmem */
    rtapi_print_msg(RTAPI_MSG_DBG,
	"RTAPI: shmem %02d created by module %02d, key: %d, size: %lu\n",
	shmem_id, module_id, key, size);

    /* and return the ID to the proud parent */
    rtapi_mutex_give(&(rtapi_data->mutex));
    return shmem_id;
}

int rtapi_shmem_delete(int shmem_id, int module_id)
{
    int retval;

    rtapi_mutex_get(&(rtapi_data->mutex));
    retval = shmem_delete(shmem_id, module_id);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return retval;
}

static int shmem_delete(int shmem_id, int module_id)
{
    shmem_data *shmem;

    /* validate shmem ID */
    if ((shmem_id < 1) || (shmem_id > RTAPI_MAX_SHMEMS)) {
	return -EINVAL;
    }
    /* point to the shmem's data */
    shmem = &(shmem_array[shmem_id]);
    /* is the block valid? */
    if (shmem->key == 0) {
	return -EINVAL;
    }
    /* validate module_id */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	return -EINVAL;
    }
    if (module_array[module_id].state != REALTIME) {
	return -EINVAL;
    }
    /* is this module using the block? */
    if (test_bit(module_id, shmem->bitmap) == 0) {
	return -EINVAL;
    }
    /* OK, we're no longer using it */
    clear_bit(module_id, shmem->bitmap);
    shmem->rtusers--;
    /* is somebody else still using the block? */
    if (shmem->rtusers > 0) {
	/* yes, we're done for now */
	rtapi_print_msg(RTAPI_MSG_DBG,
	    "RTAPI: shmem %02d closed by module %02d\n", shmem_id, module_id);
	return 0;
    }
    /* no other realtime users, free the shared memory from kernel space */

    rt_heap_delete(&shmem_heap_array[shmem_id]);

    shmem_addr_array[shmem_id] = NULL;
    shmem->rtusers = 0;
    /* are any user processes using the block? */
    if (shmem->ulusers > 0) {
	/* yes, we're done for now */
	rtapi_print_msg(RTAPI_MSG_DBG,
	    "RTAPI: shmem %02d unmapped by module %02d\n", shmem_id,
	    module_id);
	return 0;
    }
    /* no other users at all, this ID is now free */
    /* update the data array and usage count */
    shmem->key = 0;
    shmem->size = 0;
    rtapi_data->shmem_count--;
    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: shmem %02d freed by module %02d\n",
	shmem_id, module_id);
    return 0;
}

int rtapi_shmem_getptr(int shmem_id, void **ptr)
{
    /* validate shmem ID */
    if ((shmem_id < 1) || (shmem_id > RTAPI_MAX_SHMEMS)) {
	return -EINVAL;
    }
    /* is the block mapped? */
    if (shmem_addr_array[shmem_id] == NULL) {
	return -EINVAL;
    }
    /* pass memory address back to caller */
    *ptr = shmem_addr_array[shmem_id];
    return 0;
}


/* starting with kernel 2.6, symbols that are used by other modules
   _must_ be explicitly exported.  2.4 and earlier kernels exported
   all non-static global symbols by default, so these explicit exports
   were not needed.  For 2.4 and older, you should define EXPORT_SYMTAB
   (before including module.h) to make these explicit exports work and 
   minimize pollution of the kernel namespace.  But EXPORT_SYMTAB
   must not be defined for 2.6, so the best place to do it is 
   probably in the makefiles somewhere (as a -D option to gcc).
*/

EXPORT_SYMBOL(rtapi_init);
EXPORT_SYMBOL(rtapi_exit);
EXPORT_SYMBOL(rtapi_shmem_new);
EXPORT_SYMBOL(rtapi_shmem_delete);
EXPORT_SYMBOL(rtapi_shmem_getptr);
