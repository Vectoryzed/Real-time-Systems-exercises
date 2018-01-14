#define TICK_PERIOD 10000000 //period = 10000000ns = 10ms
#define STACK_SIZE 10000 //10k size
#define SHMNAM 123

struct data {
	unsigned int x[10]; //tempo corrente [ms]
	unsigned short y[10]; //ampiezza della forma d'onda
	short wcet[10];
	short late[10]; //lateness
};
