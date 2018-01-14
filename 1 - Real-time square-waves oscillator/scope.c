/*#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>*/

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include "/usr/realtime/include/rtai_shm.h"
#include "header.h"

//def. di un task RT a livello utente
static int end;
static void endme() { end=1; }

int main (void) {
	
	int i;
        struct data *shm = rtai_malloc (SHMNAM,1); //usa la shared memory a livello utente (già è stata allocata, perciò 1)
	signal(SIGINT, endme); //signal è una funzione che cattura segnali da terminale; SIGINT indica una interrupt dalla tastiera (cattura il segnale ctrl+c)

        while (!end) {
		for(i=0; i<10; i++)
        		printf("TASK %d: [%u ms, %d] C=%d ns, L=%d ms\n", i, shm->x[i], shm->y[i], shm->wcet[i], shm->late[i]);
		//sleep(1);
   	}

	return 0;
}
