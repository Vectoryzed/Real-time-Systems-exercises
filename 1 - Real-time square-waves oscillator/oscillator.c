#include </usr/realtime/include/rtai_shm.h> 
#include </usr/realtime/include/rtai_sched.h>
#include "header.h"

static RT_TASK tasks[10];
static struct data *shm;

//routine per la generazione della forma d'onda da parte del task i-esimo
static void make_square_wave(int i) {
	unsigned int count = 0; //variabile che tiene conto delle commutazioni di ampiezza da effettuare
	short wcet=0, exe_time;

	while (1) {
//andremo a prelevare il CPU time in ns (rt_get_cpu_time) e NON il tempo in 'internal count units' (rt_get_time)

		exe_time = rt_get_cpu_time_ns(); //prelevo l'istante relativo all'inizio del burst dell'istanza del task i-esimo (start time)
		shm->x[i-1] = rt_get_cpu_time_ns()*1000000; //preleva il tempo di sistema [ms]
		if ( count < 10 )
			shm->y[i-1] = 0;
		else
			shm->y[i-1] = i;

		count = (++count)%20;

		exe_time = rt_get_cpu_time_ns() - exe_time; //calcolo il burst dell'istanza del task i-esimo 
		if (exe_time > wcet) //se il tempo di esecuzione appena calcolato è maggiore del WCET precedentemente stimato
			wcet = exe_time; //aggiorno il WCET
		
		shm->wcet[i-1] = wcet; // il WCET è approssimato poichè il calcolo dell'execution time viene fatto prima di terminare il task
		
		shm->late[i-1] = (exe_time - (TICK_PERIOD*i))*1000000; //calcolo della lateness [ms], che per def. è la differenza tra il
//finishing time e la deadline assoluta del task
		
		if ( shm->late[i-1] > -1 )
			rt_printk("Error: deadline raggiunta o sforata!\n");

		rt_task_wait_period(); // <=> end_cycle() : il task entra nello stato IDLE
	}
}

int init_module(void) {
	int i;
	RTIME tick_period;
	rt_set_periodic_mode();
	
	shm = rtai_kmalloc( SHMNAM, sizeof(struct data) );
	for(i=1; i<=10; i++) {
        	rt_task_init(&tasks[i-1], make_square_wave, i, STACK_SIZE, 10-i, 1, 0); //settiamo le priorità in modo inverso per verificare il RM
		tick_period = nano2count(TICK_PERIOD*i);
		if (i==1) 
			tick_period = start_rt_timer(tick_period); //avvia il timer una volta sola per tutte, settandolo al periodo più piccolo
		rt_task_make_periodic(&tasks[i-1], rt_get_time() + tick_period, tick_period); //lancio del task
	}
	
	rt_spv_RMS(hard_cpu_id()); //Rate Monotonic Scheduling
	return 0;
}

void cleanup_module(void) {
	int i;
	stop_rt_timer();
	
	for (i=0; i<10; i++) 	
		rt_task_delete(&tasks[i]); //dealloca il task
    
        rtai_kfree(SHMNAM); //libera l'area di memoria
	return;
}
