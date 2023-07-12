#include "include/priv_sched.h"

sched_t schedInit(void) {
    // TODO
    sched_t sched= (sched_t) malloc(sizeof(struct sched));
    sched->head =NULL;
    sched->tail = NULL;
    sched->current = NULL;
    return sched;

}

int taskAdd(sched_t sched, struct task *task) {
    struct sched_task * tsk = malloc(sizeof(struct sched_task));
    tsk->next = NULL;
    tsk->callback = task->callback;

    if(sched->head ==NULL) {
        sched->head = tsk;
        sched->tail = tsk;
        sched->current = tsk;
    }else{
        sched->tail->next = tsk;
        sched->tail = tsk;
    }

    // TODO
    return 1;
}

void taskRun(struct sched_task *task) {
    task->callback();
    // TODO
}

void schedStart(sched_t sched,int (*reset)(void)) {
    if(sched->current == NULL) printf("rien a effecteur \n");
    while(1){
        int comeback = reset();
        if(comeback==1) {
            sched->current = sched->current->next;
            if(sched->current==NULL) sched->current = sched->head; //aka on y met du wd40
            continue;
        }
        taskRun(sched->current);
        sched->current = sched->current->next;
        if(sched->current==NULL) sched->current = sched->head; //aka on y met du wd40

    }
    // TODO
}
