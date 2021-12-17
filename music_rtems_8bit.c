/**********************************************************
 *  INCLUDES
 *********************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>

#include <rtems.h>
#include <rtems/shell.h>
#include <rtems/untar.h>
#include <bsp.h>

#ifdef RASPBERRYPI
#include <bsp/i2c.h>
#endif


/**********************************************************
 *  CONSTANTS
 *********************************************************/
#define NSEC_PER_SEC 1000000000UL

#ifdef RASPBERRYPI
#define DEV_NAME "/dev/i2c"
#else
#define DEV_NAME "/dev/com1"
#endif
//TODO: a lo mejorno es sin 1bit
#define FILE_NAME "/let_it_be.raw"

#define PERIOD_TASK_SEC    0            /* Period of Task */
#define PERIOD_TASK_NSEC  32000000    /* Period of Task */
#define SEND_SIZE 128                /* BYTES */

#define TARFILE_START _binary_tarfile_start
#define TARFILE_SIZE _binary_tarfile_size

#define SLAVE_ADDR 0x8

/**********************************************************
 *  GLOBALS
 *********************************************************/
extern int _binary_tarfile_start;
extern int _binary_tarfile_size;
int repMode=1; //0=pause,1=reanudar
pthread_mutex_t mutex;
pthread_cond_t cond;
struct timespec timeout;
struct sched_param param;
pthread_attr_t attr;
struct timespec startTimerSound, endTimerSound, diffTimerSound, cycleTimerSound;
unsigned char buf[SEND_SIZE];
int fd_file = -1;
int fd_serie = -1;
int ret = 0;

/**********************************************************
 * Function: diffTime
 *********************************************************/
void diffTime(struct timespec end,
              struct timespec start,
              struct timespec *diff)
{
    if (end.tv_nsec < start.tv_nsec) {
        diff->tv_nsec = NSEC_PER_SEC - start.tv_nsec + end.tv_nsec;
        diff->tv_sec = end.tv_sec - (start.tv_sec+1);
    } else {
        diff->tv_nsec = end.tv_nsec - start.tv_nsec;
        diff->tv_sec = end.tv_sec - start.tv_sec;
    }
}

/**********************************************************
 * Function: addTime
 *********************************************************/
void addTime(struct timespec end,
             struct timespec start,
             struct timespec *add)
{
    unsigned long aux;
    aux = start.tv_nsec + end.tv_nsec;
    add->tv_sec = start.tv_sec + end.tv_sec +
                  (aux / NSEC_PER_SEC);
    add->tv_nsec = aux % NSEC_PER_SEC;
}

/**********************************************************
 * Function: compTime
 *********************************************************/
int compTime(struct timespec t1,
             struct timespec t2)
{
    if (t1.tv_sec == t2.tv_sec) {
        if (t1.tv_nsec == t2.tv_nsec) {
            return (0);
        } else if (t1.tv_nsec > t2.tv_nsec) {
            return (1);
        } else if (t1.tv_sec < t2.tv_sec) {
            return (-1);
        }
    } else if (t1.tv_sec > t2.tv_sec) {
        return (1);
    } else if (t1.tv_sec < t2.tv_sec) {
        return (-1);
    }
    return (0);
}



void* sendSound(void* args){
    while (1) {
        // read from music file
        if (repMode==1){
            ret = read(fd_file, buf, SEND_SIZE);

            if (ret < 0) {
                printf("read: error reading file\n");
                exit(-1);
            }
        }


        // write on the serial/I2C port
        if (repMode==1){
            ret = write(fd_serie, buf, SEND_SIZE);
            if (ret < 0) {
                printf("write: error writting serial\n");
                exit(-1);
            }
        }
        else if (repMode==0){
            ret = write(fd_serie, "\0", SEND_SIZE);
            if (ret < 0) {
                printf("write: error writting serial\n");
                exit(-1);
            }
        }




        // get end time, calculate lapso and sleep
        clock_gettime(CLOCK_REALTIME, &endTimerSound);
        diffTime(endTimerSound, startTimerSound, &diffTimerSound);
        if (0 >= compTime(cycleTimerSound, diffTimerSound)) {
            printf("ERROR: lasted long than the cycle\n");
            exit(-1);
        }
        diffTime(cycleTimerSound, diffTimerSound, &diffTimerSound);
        nanosleep(&diffTimerSound, NULL);
        addTime(startTimerSound, cycleTimerSound, &startTimerSound);

    }

}

void* receiveOrder(void* args){
    struct timespec init;
    struct timespec end;
    struct timespec diff;
    while(1) {
        clock_gettime(CLOCK_REALTIME, &init);
        int oldValue=repMode;
        if(scanf("%d", &repMode)==0)repMode=oldValue;


        clock_gettime(CLOCK_REALTIME, &end);

        diffTime(end, init, &diff);
        //sleep(2 - diff.tv_sec);
        sleep(2);
        init.tv_sec = init.tv_sec + 2;
    }

}

void* showRepMode(void* args){

    while (1){
        if (repMode==0){
            printf("Reproduccion en pausa\n");
        }
        else if (repMode==1){
            printf("Reproduccion en marcha\n");
        }
        sleep(5);
    }
}

/*****************************************************************************
 * Function: Init()
 *****************************************************************************/
rtems_task Init (rtems_task_argument ignored)
{
    printf("Populating Root file system from TAR file.\n");
    Untar_FromMemory((unsigned char *) (&TARFILE_START),
                     (unsigned long) &TARFILE_SIZE);

    rtems_shell_init("SHLL", RTEMS_MINIMUM_STACK_SIZE * 4,
                     100, "/dev/foobar", false, true, NULL);

#ifdef RASPBERRYPI
    // Init the i2C driver
	            rpi_i2c_init();

	            // bus registering, this init the ports needed for the conexion
	            // and register the device under /dev/i2c
	#define I2C_HZ 1000000
	            printf("Register I2C device %s (%d, Hz) \n",DEV_NAME, I2C_HZ);
	            rpi_i2c_register_bus("/dev/i2c", I2C_HZ);

	            // open device file
	            printf("open I2C device %s \n",DEV_NAME);
	            fd_serie = open(DEV_NAME, O_RDWR);
	            if (fd_serie < 0) {
	                printf("open: error opening serial %s\n", DEV_NAME);
	                exit(-1);
	            }

	            // register the address of the slave to comunicate with
	            ioctl(fd_serie, I2C_SLAVE, SLAVE_ADDR);
#else
    /* Open serial port */
    printf("open serial device %s \n", DEV_NAME);
    fd_serie = open(DEV_NAME, O_RDWR);
    if (fd_serie < 0) {
        printf("open: error opening serial %s\n", DEV_NAME);
        exit(-1);
    }
#endif

    /* Open music file */
    printf("open file %s begin\n", FILE_NAME);
    fd_file = open(FILE_NAME, O_RDWR);
    if (fd_file < 0) {
        perror("open: error opening file \n");
        exit(-1);
    }

    // loading cycle time
    cycleTimerSound.tv_sec = PERIOD_TASK_SEC;
    cycleTimerSound.tv_nsec = PERIOD_TASK_NSEC;

    clock_gettime(CLOCK_REALTIME, &startTimerSound);

    pthread_t t1;
    pthread_t t2;
    pthread_t t3;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_mutex_init( &mutex, NULL );
    pthread_cond_init( &cond, NULL );
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    while(1){
        if ( pthread_create( &t1, &attr, sendSound, NULL) ||
             pthread_setschedparam(t1, SCHED_FIFO, &param) ) {
            printf(
                    "Thread cannot be created or you have not enough privileges \n"
                    "    to set priority!!!!\n");
            exit(1);
        }
        param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
        pthread_attr_setschedparam(&attr, &param);
        if ( pthread_create( &t2, &attr, receiveOrder, NULL) ||
             pthread_setschedparam(t2, SCHED_FIFO, &param) ) {
            printf(
                    "Thread cannot be created or you have not enough privileges \n"
                    "    to set priority!!!!\n");
            exit(1);
        }
        param.sched_priority = sched_get_priority_max(SCHED_FIFO) -2;
        pthread_attr_setschedparam(&attr, &param);
        if ( pthread_create( &t3, &attr, showRepMode, NULL) ||
             pthread_setschedparam(t3, SCHED_FIFO, &param) ) {
            printf(
                    "Thread cannot be created or you have not enough privileges \n"
                    "    to set priority!!!!\n");
            exit(1);
        }

        pthread_join(t3,NULL);
        pthread_join(t2,NULL);
        pthread_join(t1,NULL);
        //sleep(200);
    }



} /* End of Init() */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_APPLICATION_NEEDS_LIBBLOCK
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 20
#define CONFIGURE_UNIFIED_WORK_AREAS
#define CONFIGURE_UNLIMITED_OBJECTS
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
