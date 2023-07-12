# Simple Time-sliced Scheduler

During the [last session](../simple_sched/README.md), we saw how to implement a very simple round-robin task scheduler.
We also discovered the interrupt system of the ESP32s3 SoC and implemented a basic one-shot ISR.

The goals of this session are the following:
- [ ] Leverage the ESP32s3 interrupt system to introduce time-slices in our scheduler.
- [ ] Discover the Rust programming language

> You **must** complete the previous practical session before beginning this one.

## Time-slicing

In this part of the session, we will extend the basic scheduler we implemented in the last session to introduce time-slices.
A time-slice is a constraint on the duration of a given task.
At the end of a time-slice, the RTOS' scheduler is called to schedule the next task according to the defined policy.
In our case, we use a round-robin strategy.

Here are the steps for this part:
1. Move your interrupt handling functions in the [scheduler](sched.c).
2. Modify your ISR to reconfigure your interrupt after use.
3. Modify the `task_run` function to introduce time-slices.
4. Prove that your scheduler is able to perform preemption by registering various tasks. 

> *Hint*: For this part, you should use the [longjmp command](https://linux.die.net/man/3/longjmp).

## Rust introduction (II)

Do the [following exercises](https://lighthearted-llama-a670af.netlify.app/exercises/course-2/afternoon.html)
