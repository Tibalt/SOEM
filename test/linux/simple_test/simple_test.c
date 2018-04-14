/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

/* elfin_txpdo, copied from elfin_ethercat_client.h */
const int AXIS1_STATUSWORD=0;
const int AXIS1_ACTPOSITION=1;
const int AXIS1_ACTCUR=2;
const int AXIS1_ERRORCODE=3;

const int AXIS2_STATUSWORD=4;
const int AXIS2_ACTPOSITION=5;
const int AXIS2_ACTCUR=6;
const int AXIS2_ERRORCODE=7;

const int UDM_STATUS=8;
const int ACCELERATION_X=9;
const int ACCELERATION_Y=10;
const int ACCELERATION_Z=11;

/* elfin_rxpdo, copied from elfin_ethercat_client.h */
const int AXIS1_CONTROLWORD=0;
const int AXIS1_TARGET_POSITION=1;
const int AXIS1_ENDATPOS_FLASH=2;
const int AXIS1_FEEDFORWARD_CUR=3;

const int AXIS2_CONTROLWORD=4;
const int AXIS2_TARGET_POSITION=5;
const int AXIS2_ENDATPOS_FLASH=6;
const int AXIS2_FEEDFORWARD_CUR=7;

const int UDM_CMD=8;

/* elfin pdo_input(txpdo) channel definations, copied from elfin_ethercat_client.cpp */
int channel_pdo_input[12]={0, 4, 8, 12, 40, 44, 48, 52, 80, 84, 88, 92};

/* elfin pdo_output(rxpdo) channel definations, copied from elfin_ethercat_client.cpp */
int channel_pdo_output[9]={0, 4, 8, 12, 40, 44, 48, 52, 80};

/* mixed copied from elfin_ethercat_client.cpp and elfin_ethercat_manager.cpp */
uint32_t readInput_unit(int n, int slave_no) {
	if(slave_no > ec_slavecount) {
		printf("ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
		exit(1);
	}
	char map[4];
        int i;
	for(i=0; i<4; i++) {
		int channel =  channel_pdo_input[n] + i;

		if (channel*8 >= ec_slave[slave_no].Ibits) {
			printf("ERROR : slave_no(%d) : channel(%d) is larger than Input bits (%d)\n", slave_no, channel*8, ec_slave[slave_no].Ibits);
			exit(1);
		}

		map[i] = ec_slave[slave_no].inputs[channel];
	}

	uint32_t value_tmp=*(uint32_t *)(map);
        printf("readInput_unit(%d, %d) = 0x%08x\n", n, slave_no, value_tmp);

        //ec_send_processdata();
        //wkc = ec_receive_processdata(EC_TIMEOUTRET);

	return value_tmp;
}

/* mixed copied from elfin_ethercat_client.cpp and elfin_ethercat_manager.cpp */
void writeOutput_unit(int n, uint32_t val, int slave_no) {
	if (slave_no > ec_slavecount) {
		printf("ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
		exit(1);
	}

	char map;
        int i;
	for(i=0; i<4; i++) {
		int channel =  channel_pdo_output[n] + i;

		if (channel*8 >= ec_slave[slave_no].Obits) {
			printf("ERROR : slave_no(%d) : channel(%d) is larger than Output bits (%d)\n", slave_no, channel*8, ec_slave[slave_no].Obits);
			exit(1);
		}

		map=(char)((val>>8*i) & 0x00ff);
		ec_slave[slave_no].outputs[channel] = map;
	}
}

void resetFault(int slave_no)
{
    writeOutput_unit(AXIS1_CONTROLWORD, 0x87, slave_no);
    writeOutput_unit(AXIS2_CONTROLWORD, 0x87, slave_no);
	ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(10000);

    writeOutput_unit(AXIS1_TARGET_POSITION, readInput_unit(AXIS1_ACTPOSITION, slave_no), slave_no);
    writeOutput_unit(AXIS2_TARGET_POSITION, readInput_unit(AXIS2_ACTPOSITION, slave_no),slave_no);
	ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
    osal_usleep(10000);

    writeOutput_unit(AXIS1_CONTROLWORD, 0x07, slave_no);
    writeOutput_unit(AXIS2_CONTROLWORD, 0x07,slave_no);
	ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
}


/* copied from elfin_ethercat_client.cpp */
void clearPoseFault(int slave_no) {
	// channel1
	writeOutput_unit(/*elfin_rxpdo::*/AXIS1_CONTROLWORD, 0x011f, slave_no);
	writeOutput_unit(/*elfin_rxpdo::*/AXIS2_CONTROLWORD, 0x001f, slave_no);
	writeOutput_unit(/*elfin_rxpdo::*/UDM_CMD, 0x0800, slave_no);
	ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
	osal_usleep(100000);
	// channel2
	writeOutput_unit(/*elfin_rxpdo::*/AXIS1_CONTROLWORD, 0x001f, slave_no);
	writeOutput_unit(/*elfin_rxpdo::*/AXIS2_CONTROLWORD, 0x011f, slave_no);
	writeOutput_unit(/*elfin_rxpdo::*/UDM_CMD, 0x0800, slave_no);
	ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
	osal_usleep(100000);
	ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
}

/* copied from elfin_ethercat_client.cpp */
int recognizePose(int slave_no) {
	if(readInput_unit(/*elfin_txpdo::*/UDM_STATUS, slave_no) == 0x11110000)
	{
		//channel1
		writeOutput_unit(/*elfin_rxpdo::*/AXIS1_CONTROLWORD, 0x201f, slave_no);
		writeOutput_unit(/*elfin_rxpdo::*/AXIS2_CONTROLWORD, 0x001f, slave_no);
		writeOutput_unit(/*elfin_rxpdo::*/UDM_CMD, 0x0300, slave_no);
		ec_send_processdata();
		wkc = ec_receive_processdata(EC_TIMEOUTRET);
		struct timespec before, tick;
		clock_gettime(CLOCK_REALTIME, &before);
		clock_gettime(CLOCK_REALTIME, &tick);
		while(1)
		{
			if(readInput_unit(/*elfin_txpdo::*/UDM_STATUS, slave_no) == 0xffff0000)
			{
				writeOutput_unit(/*elfin_rxpdo::*/AXIS1_CONTROLWORD, 0x001f, slave_no);
				writeOutput_unit(/*elfin_rxpdo::*/AXIS2_CONTROLWORD, 0x001f, slave_no);
				writeOutput_unit(/*elfin_rxpdo::*/UDM_CMD, 0x0000, slave_no);
				ec_send_processdata();
				wkc = ec_receive_processdata(EC_TIMEOUTRET);
				osal_usleep(100000);
				break;
			}
			if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
			{
				printf("recognizePose failed while pose recognition in slave %i, channel 1\n", slave_no);
				return 0;
			}
			osal_usleep(100000);
			clock_gettime(CLOCK_REALTIME, &tick);
		}
	}
	else 
	{
		printf("recognizePose failed in slave %i, channel 1, the reason might be there is a fault or the motor is enabled\n", slave_no);
		return 0;
	}
	if(readInput_unit(UDM_STATUS,slave_no) == 0x11110000)
	{
		//channel2
		writeOutput_unit(AXIS1_CONTROLWORD, 0x001f,slave_no);
		writeOutput_unit(AXIS2_CONTROLWORD, 0x201f,slave_no);
		writeOutput_unit(UDM_CMD, 0x3000,slave_no);
		ec_send_processdata();
		wkc = ec_receive_processdata(EC_TIMEOUTRET);
		struct timespec before, tick;
		clock_gettime(CLOCK_REALTIME, &before);
		clock_gettime(CLOCK_REALTIME, &tick);
		while(1)
		{
			if(readInput_unit(UDM_STATUS,slave_no) == 0xffff0000)
			{
				writeOutput_unit(AXIS1_CONTROLWORD, 0x001f,slave_no);
				writeOutput_unit(AXIS2_CONTROLWORD, 0x001f,slave_no);
				writeOutput_unit(UDM_CMD, 0x0000,slave_no);
				ec_send_processdata();
				wkc = ec_receive_processdata(EC_TIMEOUTRET);
				osal_usleep(100000);
				break;
			}
			if(tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 20e+9)
			{
				printf("recognizePose failed while pose recognition in slave %i, channel 2", slave_no);
				return 0;
			}
			osal_usleep(100000);
			clock_gettime(CLOCK_REALTIME, &tick);
		}
	}
	else
	{
		printf("recognizePose failed in slave %i, channel 2, the reason might be there is a fault or the motor is enabled", slave_no);
		return 0;
	}


	return 1;
}

void simpletest(char *ifname)
{
    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
	   printf("ec_init on %s succeeded.\n",ifname);
	   /* find and auto-config slaves */


	   if ( ec_config_init(FALSE) > 0 )
	   {
		   printf("%d slaves found and configured.\n",ec_slavecount);

		   ec_config_map(&IOmap);

		   ec_configdc();

		   printf("Slaves mapped, state to SAFE_OP.\n");
		   /* wait for all slaves to reach SAFE_OP state */
		   if(ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4) != EC_STATE_SAFE_OP) 
		   {
			   printf("Could not set EC_STATE_SAFE_OP\n");
			   return;
		   }

		   oloop = ec_slave[0].Obytes;
		   if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
		   if (oloop > 8) oloop = 8;
		   iloop = ec_slave[0].Ibytes;
		   if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
		   if (iloop > 8) iloop = 8;

		   printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

		   printf("Request operational state for all slaves\n");
		   expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
		   printf("Calculated workcounter %d\n", expectedWKC);
		   ec_slave[0].state = EC_STATE_OPERATIONAL;
		   /* send one valid process data to make outputs in slaves happy*/
		   ec_send_processdata();
		   ec_receive_processdata(EC_TIMEOUTRET);
		   /* request OP state for all slaves */
		   ec_writestate(0);
		   chk = 40;
		   /* wait for all slaves to reach OP state */
		   do
		   {
			   ec_send_processdata();
			   ec_receive_processdata(EC_TIMEOUTRET);
			   ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
		   }
		   while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
		   printf("chk=%d\n", chk);

		   if(ec_statecheck(0,EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL) 
		   {
			   printf("OPERATIONAL state not set, exiting\n");
			   return;
		   }

		   for(i = 1; i<=ec_slavecount && i <= 3; i++){ 
			   resetFault(i);
		   }

		   //readInput_unit(/*elfin_txpdo::*/UDM_STATUS, 1);
		   /* function setEnable, copied from elfin_ethercat_client.cpp */
		   //for(i = 1; i<=ec_slavecount && i<=3; i++) 
		   //{
		   //	readInput_unit(/*elfin_txpdo::*/UDM_STATUS, i);
		   // }

		   //exit(1);
		   //clearPoseFault(1);



		   //readInput_unit(/*elfin_txpdo::*/UDM_STATUS, 1);
		   //exit(1);

		   for(i = 1; i<=ec_slavecount && i <= 3; i++) 
		   {

			   if(readInput_unit(/*elfin_txpdo::*/UDM_STATUS, i) == 0x6666)
			   {
				   clearPoseFault(i);
				   if(!recognizePose(i))
				   {
					   return;
				   }
			   }

			   //enable
			   if(readInput_unit(/*elfin_txpdo::*/UDM_STATUS, i) == 0x11110000)
			   {
				   writeOutput_unit(/*elfin_rxpdo::*/AXIS1_TARGET_POSITION, readInput_unit(/*elfin_txpdo::*/AXIS1_ACTPOSITION, i), i);
				   writeOutput_unit(/*elfin_rxpdo::*/AXIS2_TARGET_POSITION, readInput_unit(/*elfin_txpdo::*/AXIS2_ACTPOSITION, i), i);
				   ec_send_processdata();
				   wkc = ec_receive_processdata(EC_TIMEOUTRET);
				   osal_usleep(100000);

				   writeOutput_unit(/*elfin_rxpdo::*/AXIS1_CONTROLWORD, 0x801f, i);
				   writeOutput_unit(/*elfin_rxpdo::*/AXIS2_CONTROLWORD, 0x801f, i);
				   writeOutput_unit(/*elfin_rxpdo::*/UDM_CMD, 0x0044, i);
				   ec_send_processdata();
				   wkc = ec_receive_processdata(EC_TIMEOUTRET);
				   osal_usleep(100000);
			   }
			   else
			   {
				   printf("UDM status is not 0x11110000 in slave %d, the reason might be there is a fault or the motor is enabled\n", i);
				   return;
			   }
		   }

		   //if (ec_slave[0].state == EC_STATE_OPERATIONAL )
		   //{
		   //   printf("Operational state reached for all slaves.\n");
		   //   inOP = TRUE;
		   //       /* cyclic loop */
		   //   for(i = 1; i <= 10000; i++)
		   //   {
		   //      ec_send_processdata();
		   //      wkc = ec_receive_processdata(EC_TIMEOUTRET);

		   //           if(wkc >= expectedWKC)
		   //           {
		   //               printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

		   //               for(j = 0 ; j < oloop; j++)
		   //               {
		   //                   printf(" %2.2x", *(ec_slave[0].outputs + j));
		   //               }

		   //               printf(" I:");
		   //               for(j = 0 ; j < iloop; j++)
		   //               {
		   //                   printf(" %2.2x", *(ec_slave[0].inputs + j));
		   //               }
		   //               printf(" T:%"PRId64"\r",ec_DCtime);
		   //               needlf = TRUE;
		   //           }
		   //           osal_usleep(5000);

		   //     }
		   //       inOP = FALSE;
		   //   }
		   //   else
		   //   {
		   //       printf("Not all slaves reached operational state.\n");
		   //       inOP = FALSE;
		   //   }
		   //   else
		   //   {
		   //       printf("Not all slaves reached operational state.\n");
		   //       ec_readstate();
		   //       for(i = 1; i<=ec_slavecount ; i++)
		   //       {
		   //           if(ec_slave[i].state != EC_STATE_OPERATIONAL)
		   //           {
		   //               printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
		   //                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
		   //           }
		   //       }
		   //   }
		   //   printf("\nRequest init state for all slaves\n");
		   //   ec_slave[0].state = EC_STATE_INIT;
		   //   /* request INIT state for all slaves */
		   //   ec_writestate(0);
	   }
	   else
	   {
		   printf("No slaves found!\n");
	   }
	   printf("End simple test, close socket\n");
	   /* stop SOEM, close socket */
	   ec_close();
   }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
//      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      //osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}
