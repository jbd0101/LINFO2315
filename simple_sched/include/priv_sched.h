#include<stdlib.h>
#include<stdio.h>

#include"sched.h"

/** Scheduler private API
 * Simple tickless, Round-Robin strategy task scheduler.
 */

/** Scheduler structure
 */
struct sched {
    struct sched_task *current;		/**< Current task executed by the scheduler. */
    struct sched_task *head;		/**< Head of the task list. Should always point on the
					     firstly added task. */
    struct sched_task *tail;		/**< Tail of the task list. Should be updated at each task
					     addition. */
};

// The scheduler of our simple RTOS.
struct sched my_sched = { 0 };

/** Internal representation of a task.
 */
struct sched_task {
    void (*callback)(void);	/**< Task callback to execute. */
    
    struct sched_task *next;	/**< Next task to run. */
};

/** Execute a specific task.
 * @param[in]	task	The current task to execute.
 */
void taskRun(struct sched_task *task);
