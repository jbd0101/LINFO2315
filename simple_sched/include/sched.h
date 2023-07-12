typedef struct sched *sched_t;

/** Scheduler API
 * Simple tickless, Round-Robin strategy task scheduler.
 */

/** Scheduler representation of a task.
 */
struct task {
    void (*callback)(void);		/**< The task callback. */
};

/** @brief Initialize the scheduler.
 * @return		A pointer to the scheduler.
 */
sched_t schedInit(void);

/** Add a task to the scheduler.
 * @param[in]	sched	The scheduler to which we add a task.
 * @param[in]	task	The task to add.
 * @return		0 if no error else 1.
 */
int taskAdd(sched_t sched, struct task *task);

/** Start the scheduler. This function never returns.
 * @param[in]	sched	The scheduler to start.
 */
void schedStart(sched_t sched,int (*reset)(void));
