//------------------- CONTROLLER.C ---------------------- 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <rtai_msg.h>
#include <sys/io.h>
#include <signal.h>
#include <math.h> // -lm in Makefile
#include "parameters.h"
#define CPUMAP 0x1

//emulates the controller

static RT_TASK* main_Task;
static RT_TASK* wdog_Task;
static RT_TASK* wdog_Task2;

static RT_TASK* read_Task;
static RT_TASK* read_Task2;
static RT_TASK* filter_Task;
static RT_TASK* filter_Task2;
static RT_TASK* control_Task;
static RT_TASK* control_Task2;
static RT_TASK* write_Task;
static RT_TASK* write_Task2;

static int keep_on_running = 1;

static pthread_t wdog_thread;
static pthread_t wdog_thread2;

static pthread_t read_thread;
static pthread_t read_thread2;
static pthread_t filter_thread;
static pthread_t filter_thread2;
static pthread_t control_thread;
static pthread_t control_thread2;
static pthread_t write_thread;
static pthread_t write_thread2;

static RTIME sampl_interv;

static void endme(int dummy) {keep_on_running = 0;}

int* sensor;
int* actuator;
int* reference;

int buffer[BUF_SIZE];
int buffer2[BUF_SIZE];
int head = 0;
int head2 = 0;
int tail = 0;
int tail2 = 0;

// data to be monitored
int avg = 0;
int avg2 = 0;
int control = 0;
int control2 = 0;

SEM* space_avail;
SEM* space_avail2;
SEM* meas_avail;
SEM* meas_avail2;

static void* wdog_loop(void* par) {
	if (!(wdog_Task = rt_task_init_schmod(nam2num("WDOG"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT WDOG TASK\n");
		exit(1);
	}
	
	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(wdog_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	rt_printk("WDOG : created.\n");

	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int count = 0;
	unsigned int err_v[N * 2];
	RT_TASK* sender = NULL;
	RT_TASK* rcv = NULL;
	unsigned int sgn = 0;
	unsigned int error = 0;
	
	while (keep_on_running)
	{
		rt_printk("WDOG : active!\n");
		i %= 2;

		sender = i ? filter_Task : read_Task;
		rcv = rt_receive_timed(sender, &sgn, sampl_interv);

		rt_printk("WDOG : signal from %s %s.\n", (i ? "FILTER" : "ACQUIRE"), (rcv == sender ? "received in time" : "not received"));

		err_v[count] = rcv != sender ? SGN : 0;

		i++;

		if (count == (N * 2) - 1)
		{
			error = 0;
			for (j = 0; (j < N * 2) && !error; j++)
			{
				if (err_v[j])
					error = SGN;
			}

			rt_send(wdog_Task2, error);
			rt_printk("WDOG : replicas output must be %s.\n", (error ? "enabled" : "disabled"));
			count = 0;
		}
		else
			count++;

		rt_task_wait_period( );
	}
	rt_task_delete(wdog_Task);
	return 0;
}

static void* wdog_loop2(void* par)
{
	if (!(wdog_Task2 = rt_task_init_schmod(nam2num("WD2"), 2, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT WDOG2 TASK\n");
		exit(1);
	}
	
	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(wdog_Task2, expected, sampl_interv);
	rt_make_hard_real_time();

	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int err_v[3];
	RT_TASK* sender = NULL;
	RT_TASK* rcv = NULL;
	unsigned int sgn = 0;
	unsigned int error = 0;

	rt_printk("WDOG2 : created.\n");
	
	while (keep_on_running)
	{
		rt_printk("WDOG2 : active!\n");

		if (!i)
		{
			sender = wdog_Task;
			rcv = rt_receive(sender, &sgn);

			rt_printk("WDOG2 : signal <%d> from WDOG %s.\n", sgn, (rcv == sender ? "received in time" : "not received"));
		}
		else
		{
			sender = i == 2 ? write_Task : control_Task;
			rcv = rt_receive_timed(sender, &sgn, sampl_interv);

			rt_printk("WDOG2 : signal from %s %s.\n", (i == 2 ? "ACTUATOR" : "CONTROL"), (rcv == sender ? "received in time" : "not received"));
		}

		if (i == 0)
			err_v[i] = (rcv != sender) || sgn;
		else
			err_v[i] = rcv != sender ? SGN : 0;

		if (i == 2)
		{
			error = 0;
			for (j = 0; j < 3 && !error; j++)
			{
				if (err_v[j])
					error = SGN;
			}

			rt_send(write_Task, error);
			rt_send(write_Task2, error);
			rt_printk("WDOG2 : replicas output must be %s.\n", (error ? "enabled" : "disabled"));
			i = 0;
		}
		else
			i++;

		rt_task_wait_period( );
	}
	rt_task_delete(wdog_Task2);
	return 0;
}

static void* acquire_loop(void* par) {
	
	if (!(read_Task = rt_task_init_schmod(nam2num("READER"), 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SENSOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(read_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	unsigned int msg = SGN;
	int sensor_val = 0;

	rt_printk("ACQUIRE : created.\n");

	while (keep_on_running)
	{
		rt_printk("ACQUIRE : active!\n");

		// DATA ACQUISITION FROM PLANT
		rt_sem_wait(space_avail);
		
		buffer[head] = (*sensor);
		sensor_val = buffer[head];
		rt_send(read_Task2, sensor_val);
		head = (head + 1) % BUF_SIZE;

		rt_sem_signal(meas_avail);

		rt_printk("ACQUIRE : sent value %d to ACQUIRE2.\n", sensor_val);

		// SLEEP TEST
		// rt_sleep(sampl_interv);
			
		RT_TASK* receiver = rt_send(wdog_Task, msg);
		if (receiver != wdog_Task)
			rt_printk("ACQUIRE : signal to WDOG not sent.\n");
		else
			rt_printk("ACQUIRE : sent signal to WDOG.\n");
		
		rt_task_wait_period();
	}
	rt_task_delete(read_Task);
	return 0;
}

static void* acquire_loop2(void* par) {
	
	if (!(read_Task2 = rt_task_init_schmod(nam2num("R2"), 4, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SENSOR2 TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(read_Task2, expected, sampl_interv);
	rt_make_hard_real_time();

	rt_printk("ACQUIRE2 : created.\n");

	int sensor_val = 0;

	while (keep_on_running)
	{
		rt_printk("ACQUIRE2 : active!\n");
		rt_receive(read_Task, &sensor_val);

		// DATA ACQUISITION FROM PLANT
		rt_sem_wait(space_avail2);

		rt_printk("ACQUIRE2 : received value %d from ACQUIRE.\n", sensor_val);
		
		buffer2[head2] = sensor_val;
		head2 = (head2 + 1) % BUF_SIZE;

		rt_sem_signal(meas_avail2);

		rt_task_wait_period();
	}
	rt_task_delete(read_Task2);
	return 0;
}

static void* filter_loop(void * par) {

	if (!(filter_Task = rt_task_init_schmod(nam2num("FILTER"), 5, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT FILTER TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(filter_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	unsigned int msg = SGN;

	rt_printk("FILTER : created.\n");

	int cnt = BUF_SIZE;
	unsigned int sum = 0;

	while (keep_on_running)
	{
		rt_printk("FILTER : active!\n");
		// FILTERING (average)
		rt_sem_wait(meas_avail);

		sum += buffer[tail];
		tail = (tail + 1) % BUF_SIZE;
	
		rt_sem_signal(space_avail);
		
		cnt--;

		if (cnt == 0) {
			cnt = BUF_SIZE;
			avg = sum / BUF_SIZE;
			sum = 0;
			// sends the average measure to the controller
			rt_send(control_Task, avg);		
		}
			
		// SLEEP TEST
		// rt_sleep(sampl_interv);

		RT_TASK* receiver = rt_send(wdog_Task, msg);
		if (receiver != wdog_Task)
			rt_printk("FILTER : signal to WDOG not sent.\n");
		else
			rt_printk("FILTER : sent signal to WDOG.\n");
		
		rt_task_wait_period();
	}
	rt_task_delete(filter_Task);
	return 0;
}

static void* filter_loop2(void * par) {

	if (!(filter_Task2 = rt_task_init_schmod(nam2num("F2"), 6, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT FILTER2 TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(filter_Task2, expected, sampl_interv);
	rt_make_hard_real_time();

	rt_printk("FILTER2 : created.\n");

	int cnt = BUF_SIZE;
	double prod = 1;

	while (keep_on_running)
	{
		rt_printk("FILTER2 : active!\n");
		// FILTERING (average)
		rt_sem_wait(meas_avail2);

		prod *= buffer2[tail2];
		tail2 = (tail2 + 1) % BUF_SIZE;
	
		rt_sem_signal(space_avail2);
		
		cnt--;

		if (cnt == 0) {
			cnt = BUF_SIZE;
			avg2 = (int)pow(prod, 0.1); // GEOMETRIC AVERAGE
			prod = 1;
			// sends the average measure to the controller
			rt_send(control_Task2, avg2);		
		}
		rt_task_wait_period();
	}
	rt_task_delete(filter_Task2);
	return 0;
}

static void* control_loop(void* par) {

	if (!(control_Task = rt_task_init_schmod(nam2num("CONTROL"), 7, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	unsigned int msg = SGN;

	rt_printk("CONTROL : created.\n");

	unsigned int plant_state = 0;
	int error = 0;
	unsigned int control_action = 0;
	while (keep_on_running)
	{
		rt_printk("CONTROL : active!\n");
		// receiving the average plant state from the filter
		rt_receive(0, &plant_state);

		// computation of the control law
		error = (*reference) - plant_state;

		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
		else control_action = 3;

		// sending the control action to the actuator
		rt_send(write_Task, control_action);
			
		// SLEEP TEST
		// rt_sleep(sampl_interv);
		
		RT_TASK* receiver = rt_send(wdog_Task2, msg);
		if (receiver != wdog_Task2)
			rt_printk("CONTROL : signal to WDOG2 not sent.\n");
		else
			rt_printk("CONTROL : sent signal to WDOG2.\n");
		
		rt_task_wait_period();

	}
	rt_task_delete(control_Task);
	return 0;
}

static void* control_loop2(void* par) {

	if (!(control_Task2 = rt_task_init_schmod(nam2num("C2"), 8, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL2 TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task2, expected, sampl_interv);
	rt_make_hard_real_time();

	rt_printk("CONTROL2 : created.\n");

	unsigned int plant_state = 0;
	int error = 0;
	unsigned int control_action = 0;
	while (keep_on_running)
	{
		rt_printk("CONTROL2 : active!\n");
		// receiving the average plant state from the filter
		rt_receive(0, &plant_state);

		// computation of the control law
		error = (*reference) - plant_state;

		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
		else control_action = 3;

		// sending the control action to the actuator
		rt_send(write_Task2, control_action);

		rt_task_wait_period();

	}
	rt_task_delete(control_Task2);
	return 0;
}

static void* actuator_loop(void* par) {

	if (!(write_Task = rt_task_init_schmod(nam2num("WRITE"), 9, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT ACTUATOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(write_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	unsigned int msg = SGN;
	unsigned int msg2 = SGN;

	rt_printk("ACTUATOR : created.\n");

	unsigned int control_action = 0;

	while (keep_on_running)
	{
		rt_printk("ACTUATOR : active!\n");

		// receiving the control action from the controller
		rt_receive(0, &control_action);
		
		switch (control_action) {
			case 1: control = 1; break;
			case 2:	control = -1; break;
			case 3:	control = 0; break;
			default: control = 0;
		}

		// SLEEP TEST
		// rt_sleep(sampl_interv);

		RT_TASK* receiver = rt_send(wdog_Task2, msg);
		if (receiver != wdog_Task2)
			rt_printk("ACTUATOR : signal to WDOG2 not sent.\n");
		else
			rt_printk("ACTUATOR : sent signal to WDOG2.\n");
		
		rt_receive(wdog_Task2, &msg2);
		rt_printk("ACTUATOR : output must be %s.\n", (!msg2 ? "enabled" : "disabled"));

		if (!msg2)
			(*actuator) = control;
		
		rt_task_wait_period();
	}
	rt_task_delete(write_Task);
	return 0;
}

static void* actuator_loop2(void* par) {

	if (!(write_Task2 = rt_task_init_schmod(nam2num("W2"), 10, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT ACTUATOR2 TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(write_Task2, expected, sampl_interv);
	rt_make_hard_real_time();

	rt_printk("ACTUATOR2 : created.\n");

	unsigned int control_action = 0;
	unsigned int msg = 0;

	while (keep_on_running)
	{
		rt_printk("ACTUATOR2 : active!\n");
		// receiving the control action from the controller
		rt_receive(0, &control_action);
		
		switch (control_action) {
			case 1: control2 = 1; break;
			case 2:	control2 = -1; break;
			case 3:	control2 = 0; break;
			default: control2 = 0;
		}
		
		rt_receive(wdog_Task2, &msg);
		rt_printk("ACTUATOR2 : output must be %s.\n", (msg ? "enabled" : "disabled"));

		if (msg)
			(*actuator) = control2;

		rt_task_wait_period();
	}
	rt_task_delete(write_Task2);
	return 0;
}


int main(void)
{
	printf("The controller is STARTED!\n");
 	signal(SIGINT, endme);

	if (!(main_Task = rt_task_init_schmod(nam2num("MAINTSK"), 0, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN TASK\n");
		exit(1);
	}

	//attach to data shared with the controller
	sensor = rtai_malloc(SEN_SHM, sizeof(int));
	actuator = rtai_malloc(ACT_SHM, sizeof(int));
	reference = rtai_malloc(REFSENS, sizeof(int));
		
	(*reference) = 110;

	space_avail = rt_typed_sem_init(SPACE_SEM, BUF_SIZE, CNT_SEM | PRIO_Q);
	space_avail2 = rt_typed_sem_init(SPACE_SEM2, BUF_SIZE, CNT_SEM | PRIO_Q);
	meas_avail = rt_typed_sem_init(MEAS_SEM, 0, CNT_SEM | PRIO_Q);
	meas_avail2 = rt_typed_sem_init(MEAS_SEM2, 0, CNT_SEM | PRIO_Q);

	if (rt_is_hard_timer_running()) {
		printf("Skip hard real_timer setting...\n");
	} else {
		rt_set_oneshot_mode();
		start_rt_timer(0);
	}

	sampl_interv = nano2count(CNTRL_TIME);
	
	// CONTROL THREADS 

	pthread_create(&wdog_thread, NULL, wdog_loop, NULL);
	pthread_create(&wdog_thread2, NULL, wdog_loop2, NULL);
	pthread_create(&read_thread, NULL, acquire_loop, NULL);
	pthread_create(&read_thread2, NULL, acquire_loop2, NULL);
	pthread_create(&filter_thread, NULL, filter_loop, NULL);
	pthread_create(&filter_thread2, NULL, filter_loop2, NULL);
	pthread_create(&control_thread, NULL, control_loop, NULL);
	pthread_create(&control_thread2, NULL, control_loop2, NULL);
	pthread_create(&write_thread, NULL, actuator_loop, NULL);
	pthread_create(&write_thread2, NULL, actuator_loop2, NULL);

	while (keep_on_running) {
		printf("Control: %d\n",(*actuator));
		rt_sleep(10000000);
	}

    	stop_rt_timer();
	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_shm_free(REFSENS);

	// rimozione semafori fantasma
	rt_task_delete(main_Task);
 	printf("The controller is STOPPED\n");
	return 0;
}

