# University of Bristol - Concurrent Computing
* Above is a kernel I developed for Cortex-A8 emulated by QEMU (download it [here](https://www.qemu.org/) ). The skeleton was provided by the University
* Most of it is written in C and Assembly. The core stuff of the kernel can be found in [hilevel.c](https://github.com/kartsridhar/CC-Kernel/blob/master/question/kernel/hilevel.c)

### Features
* Dynamic creation and termination of user programs. For example: P3, P4, P5 and 2B
* Priority-based scheduling of the PCBs. I do not use any data structure here however have made a linked-list library for it [here]().
* Inter Process Communication (Message Passing/Pipes) demonstrated by a program that solves the [Dining Philosophers problem](https://en.wikipedia.org/wiki/Dining_philosophers_problem)


