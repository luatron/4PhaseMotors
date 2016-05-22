//###########################################################################
// Description:
//! \addtogroup f2806x_example_list
//! <h1>ePWM Action Qualifier Module using Upcount mode (epwm_up_aq)</h1>
//!
//! This example configures ePWM1, ePWM2, ePWM3 to produce a waveform with
//! independent modulation on EPWMxA and EPWMxB. The compare values CMPA
//! and CMPB are modified within the ePWM's ISR. The TB counter is in upmode.
//!
//! Monitor the ePWM1 - ePWM3 pins on an oscilloscope.
//!
//! \b External \b Connections \n
//!  - EPWM1A is on GPIO0
//!  - EPWM1B is on GPIO1
//!  - EPWM2A is on GPIO2
//!  - EPWM2B is on GPIO3
//!  - EPWM3A is on GPIO4
//!  - EPWM3B is on GPIO5
//!
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V150 $
// $Release Date: June 16, 2015 $
// $Copyright: Copyright (C) 2011-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//Analog in is pin 65 on the Ti

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
 #include "IQmathLib.h"
//#include "IQmathCPP.h"
//#define PI 3.14159
//iq input, sin_out;

typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;
   Uint16 EPwmSetPoint; //*** this is for POT
   Uint16 SinCos_TICKER;//** MBED
   float  SinCos0_values;//**Start at high on the Uni circle
   Uint8  ULLatch;//upper and lower wave of Sin/Cos
   Uint8 SinORCos; //0=sin 1=cosine

}EPWM_INFO;

// Prototype statements for functions found within this file.
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
__interrupt void epwm1_isr(void);
//__interrupt void epwm2_isr(void);
//__interrupt void epwm3_isr(void);
void update_compare(EPWM_INFO*,EPWM_INFO*);
//void update_compare(EPWM_INFO*);
float map(float x, float in_min,float in_max,float out_max,float out_min);

// Global variables used in this example
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;

// Configure the period for each timer
#define EPWM1_TIMER_TBPRD  2500//2000  // Period register
#define EPWM1_MAX_CMPA     2500//
#define EPWM1_MIN_CMPA       0
#define EPWM1_MAX_CMPB     2500
#define EPWM1_MIN_CMPB       0

#define EPWM2_TIMER_TBPRD  2500  // Period register
#define EPWM2_MAX_CMPA     2500
#define EPWM2_MIN_CMPA       0
#define EPWM2_MAX_CMPB     2500
#define EPWM2_MIN_CMPB       0

#define EPWM3_TIMER_TBPRD  2000  // Period register
#define EPWM3_MAX_CMPA      950
#define EPWM3_MIN_CMPA       50
#define EPWM3_MAX_CMPB     1950
#define EPWM3_MIN_CMPB     1050
#define PI180 0.0174532925
// To keep track of which way the compare value is moving
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0
int SinCos_TICKER =0;//** MBED
int PWMMapping =0;
int16 SinPointer = 0;
int16 CoSPointer =300;
Uint16 WAVETickeer =1;
Uint16 ArrayPinter =0;
float Remainder = 0.0;
Uint16 wholeNum = 0;
Uint16 analogRead_F =0.0;
float Max_SinF =1000.0;
Uint16 MaxWaveFreq_I =600; //maximum sinewave frequency default 300Hz
float MaxPWM_Perc_F = EPWM1_TIMER_TBPRD;//maximum ~ o.5 is 50
float  PotRead_I =0;
float WaveStepSize_f= 0.0;
float VoltageSetpoint_F =0.0;
float WavePerStepSize_f =0.0;
float Delata_f =0.0;
Uint16 StepSize_i =0;
Uint16 FloatToInt_i =0;
Uint8 UpdateStep_Size = 1;
int x=0;
int PWMOffsetPerC = 5;
int OffsetPot =0;
unsigned int Sin_CosP90values[721] = {
		0	,	22	,	44	,	65	,	87	,	109	,	131,	153	,	175,	196,
		218	,	240	,	262	,	284	,	305	,	327	,	349,	371	,	393,	414,
		436	,	458	,	480	,	502	,	523	,	545	,	567,	589	,	610,	632,
		654	,	676	,	698	,	719	,	741,	763,	785,	806	,	828,	850,
		872	,	893	,	915	,	937	,	958,	980,	1002,	1024,	1045,	1067,
		1089,	1110,	1132,	1154,	1175,	1197	,1219,	1240,	1262,	1284,
		1305,	1327,	1349,	1370,	1392,	1413,	1435,	1457,	1478,	1500,
		1521,	1543,	1564,	1586,	1607,	1629,	1650,	1672,	1693,	1715,
		1736,	1758,	1779,	1801,	1822,	1844,	1865,	1887,	1908,	1930,
		1951,	1972,	1994,	2015,	2036,	2058,	2079,	2100,	2122,	2143,
		2164,	2186,	2207,	2228,	2250,	2271,	2292,	2313,	2334,	2356,
		2377,	2398,	2419,	2440,	2462,	2483,	2504,	2525,	2546,	2567,
		2588,	2609,	2630,	2651,	2672,	2693,	2714,	2735,	2756,	2777,
		2798,	2819,	2840,	2861,	2882,	2903,	2924,	2945,	2965,	2986,
		3007,	3028,	3049,	3069,	3090,	3111,	3132,	3152,	3173,	3194,
		3214	,
		3235	,
		3256	,
		3276	,
		3297	,
		3317	,
		3338	,
		3359	,
		3379	,
		3400	,
		3420	,
		3441	,
		3461	,
		3482	,
		3502	,
		3523	,
		3543	,
		3563	,
		3584	,
		3604	,
		3624	,
		3645	,
		3665	,
		3685	,
		3706	,
		3726	,
		3746	,
		3766	,
		3786	,
		3807	,
		3827	,
		3847	,
		3867	,
		3887	,
		3907	,
		3927	,
		3947	,
		3967	,
		3987	,
		4007	,
		4027	,
		4047	,
		4067	,
		4087	,
		4107	,
		4127	,
		4147	,
		4167	,
		4187	,
		4206	,
		4226	,
		4246	,
		4266	,
		4285	,
		4305	,
		4325	,
		4344	,
		4364	,
		4384	,
		4403	,
		4423	,
		4442	,
		4462	,
		4481	,
		4501	,
		4520	,
		4540	,
		4559	,
		4579	,
		4598	,
		4617	,
		4637	,
		4656	,
		4675	,
		4695	,
		4714	,
		4733	,
		4752	,
		4772	,
		4791	,
		4810	,
		4829	,
		4848	,
		4867	,
		4886	,
		4905	,
		4924	,
		4943	,
		4962	,
		4981	,
		5000	,
		5019	,
		5038	,
		5057	,
		5075	,
		5094	,
		5113	,
		5132	,
		5150	,
		5169	,
		5188	,
		5206	,
		5225	,
		5244	,
		5262	,
		5281	,
		5299	,
		5318	,
		5336	,
		5355	,
		5373	,
		5391	,
		5410	,
		5428	,
		5446	,
		5465	,
		5483	,
		5501	,
		5519	,
		5538	,
		5556	,
		5574	,
		5592	,
		5610	,
		5628	,
		5646	,
		5664	,
		5682	,
		5700	,
		5718	,
		5736	,
		5754	,
		5771	,
		5789	,
		5807	,
		5825	,
		5842	,
		5860	,
		5878	,
		5895	,
		5913	,
		5931	,
		5948	,
		5966	,
		5983	,
		6001	,
		6018	,
		6036	,
		6053	,
		6070	,
		6088	,
		6105	,
		6122	,
		6139	,
		6157	,
		6174	,
		6191	,
		6208	,
		6225	,
		6242	,
		6259	,
		6276	,
		6293	,
		6310	,
		6327	,
		6344	,
		6361	,
		6378	,
		6394	,
		6411	,
		6428	,
		6445	,
		6461	,
		6478	,
		6494	,
		6511	,
		6528	,
		6544	,
		6561	,
		6577	,
		6593	,
		6610	,
		6626	,
		6643	,
		6659	,
		6675	,
		6691	,
		6708	,
		6724	,
		6740	,
		6756	,
		6772	,
		6788	,
		6804	,
		6820	,
		6836	,
		6852	,
		6868	,
		6884	,
		6899	,
		6915	,
		6931	,
		6947	,
		6962	,
		6978	,
		6994	,
		7009	,
		7025	,
		7040	,
		7056	,
		7071	,
		7086	,
		7102	,
		7117	,
		7133	,
		7148	,
		7163	,
		7178	,
		7193	,
		7209	,
		7224	,
		7239	,
		7254	,
		7269	,
		7284	,
		7299	,
		7314	,
		7328	,
		7343	,
		7358	,
		7373	,
		7387	,
		7402	,
		7417	,
		7431	,
		7446	,
		7461	,
		7475	,
		7490	,
		7504	,
		7518	,
		7533	,
		7547	,
		7561	,
		7576	,
		7590	,
		7604	,
		7618	,
		7632	,
		7646	,
		7660	,
		7674	,
		7688	,
		7702	,
		7716	,
		7730	,
		7744	,
		7758	,
		7771	,
		7785	,
		7799	,
		7812	,
		7826	,
		7840	,
		7853	,
		7867	,
		7880	,
		7894	,
		7907	,
		7920	,
		7934	,
		7947	,
		7960	,
		7973	,
		7986	,
		7999	,
		8013	,
		8026	,
		8039	,
		8052	,
		8064	,
		8077	,
		8090	,
		8103	,
		8116	,
		8128	,
		8141	,
		8154	,
		8166	,
		8179	,
		8192	,
		8204	,
		8216	,
		8229	,
		8241	,
		8254	,
		8266	,
		8278	,
		8290	,
		8303	,
		8315	,
		8327	,
		8339	,
		8351	,
		8363	,
		8375	,
		8387	,
		8399	,
		8410	,
		8422	,
		8434	,
		8446	,
		8457	,
		8469	,
		8480	,
		8492	,
		8504	,
		8515	,
		8526	,
		8538	,
		8549	,
		8560	,
		8572	,
		8583	,
		8594	,
		8605	,
		8616	,
		8627	,
		8638	,
		8649	,
		8660	,
		8671	,
		8682	,
		8693	,
		8704	,
		8714	,
		8725	,
		8736	,
		8746	,
		8757	,
		8767	,
		8778	,
		8788	,
		8799	,
		8809	,
		8819	,
		8829	,
		8840	,
		8850	,
		8860	,
		8870	,
		8880	,
		8890	,
		8900	,
		8910	,
		8920	,
		8930	,
		8940	,
		8949	,
		8959	,
		8969	,
		8978	,
		8988	,
		8997	,
		9007	,
		9016	,
		9026	,
		9035	,
		9045	,
		9054	,
		9063	,
		9072	,
		9081	,
		9091	,
		9100	,
		9109	,
		9118	,
		9127	,
		9135	,
		9144	,
		9153	,
		9162	,
		9171	,
		9179	,
		9188	,
		9197	,
		9205	,
		9214	,
		9222	,
		9230	,
		9239	,
		9247	,
		9255	,
		9264	,
		9272	,
		9280	,
		9288	,
		9296	,
		9304	,
		9312	,
		9320	,
		9328	,
		9336	,
		9344	,
		9351	,
		9359	,
		9367	,
		9374	,
		9382	,
		9389	,
		9397	,
		9404	,
		9412	,
		9419	,
		9426	,
		9434	,
		9441	,
		9448	,
		9455	,
		9462	,
		9469	,
		9476	,
		9483	,
		9490	,
		9497	,
		9504	,
		9511	,
		9517	,
		9524	,
		9531	,
		9537	,
		9544	,
		9550	,
		9557	,
		9563	,
		9569	,
		9576	,
		9582	,
		9588	,
		9594	,
		9600	,
		9607	,
		9613	,
		9619	,
		9625	,
		9630	,
		9636	,
		9642	,
		9648	,
		9654	,
		9659	,
		9665	,
		9670	,
		9676	,
		9681	,
		9687	,
		9692	,
		9698	,
		9703	,
		9708	,
		9713	,
		9719	,
		9724	,
		9729	,
		9734	,
		9739	,
		9744	,
		9749	,
		9753	,
		9758	,
		9763	,
		9768	,
		9772	,
		9777	,
		9781	,
		9786	,
		9790	,
		9795	,
		9799	,
		9804	,
		9808	,
		9812	,
		9816	,
		9820	,
		9825	,
		9829	,
		9833	,
		9837	,
		9840	,
		9844	,
		9848	,
		9852	,
		9856	,
		9859	,
		9863	,
		9866	,
		9870	,
		9873	,
		9877	,
		9880	,
		9884	,
		9887	,
		9890	,
		9893	,
		9897	,
		9900	,
		9903	,
		9906	,
		9909	,
		9912	,
		9914	,
		9917	,
		9920	,
		9923	,
		9925	,
		9928	,
		9931	,
		9933	,
		9936	,
		9938	,
		9941	,
		9943	,
		9945	,
		9947	,
		9950	,
		9952	,
		9954	,
		9956	,
		9958	,
		9960	,
		9962	,
		9964	,
		9966	,
		9967	,
		9969	,
		9971	,
		9973	,
		9974	,
		9976	,
		9977	,
		9979	,
		9980	,
		9981	,
		9983	,
		9984	,
		9985	,
		9986	,
		9987	,
		9988	,
		9990	,
		9990	,
		9991	,
		9992	,
		9993	,
		9994	,
		9995	,
		9995	,
		9996	,
		9997	,
		9997	,
		9998	,
		9998	,
		9998	,
		9999	,
		9999	,
		9999	,
		10000	,
		10000	,
		10000	,
		10000	,
		10000	,



};


void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2806x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the F2806x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the F2806x_EPwm.c file
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEPwm3Gpio();
   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //***enable XCLOCKOUT through GPIO mux
   SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 2; //***XCLOCKOUT = SYSCLK
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2806x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2806x_DefaultIsr.c.
// This function is found in F2806x_PieVect.c.
   InitPieVectTable();
   InitAdc();//***
   AdcOffsetSelfCal();//***
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.EPWM1_INT = &epwm1_isr;
  // PieVectTable.EPWM2_INT = &epwm2_isr;
   //PieVectTable.EPWM3_INT = &epwm3_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals();  // Not required for this example



   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   InitEPwm1Example();
   InitEPwm2Example();
   InitEPwm3Example();

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;

   EALLOW;
     AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;//***	//Enable non-overlap mode
     //AdcRegs.ADCCTL1.bit.TEMPCONV  = 1;  //***	//**Connect channel A5 internally to the temperature sensor
     AdcRegs.ADCSOC0CTL.bit.CHSEL  = 5;    //***	//Set SOC0 channel select to ADCINA5
     AdcRegs.ADCSOC0CTL.bit.ACQPS  = 25;   //***		//Set SOC0 acquisition period to 26 ADCCLK
     AdcRegs.INTSEL1N2.bit.INT1SEL = 0;    //***	//Connect ADCINT1 to EOC0
     AdcRegs.INTSEL1N2.bit.INT1E  =  1;    //***		//Enable ADCINT1

   // Set the flash OTP wait-states to minimum. This is important
   // for the performance of the temperature conversion function.
     FlashRegs.FOTPWAIT.bit.OTPWAIT = 1;//***
    EDIS;//***

// Step 5. User specific code, enable interrupts:

// Enable CPU INT3 which is connected to EPWM1-3 INT:
   IER |= M_INT3;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
  // PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
  // PieCtrlRegs.PIEIER3.bit.INTx3 = 1;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):
   //***epwm1_info.EPwmSetPoint = 1900;

   //  int BUFFER_SIZE = 360;
     //Golden Test Values

   MaxPWM_Perc_F = MaxPWM_Perc_F*30/100; //30 means 30 percent of EPWM1_TIMER_TBPRD
   PWMOffsetPerC =  MaxPWM_Perc_F*PWMOffsetPerC/100;
   OffsetPot = 4050*PWMOffsetPerC/100;
   for(;;)
   {    // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

	   //Force start of conversion on SOC0
	           AdcRegs.ADCSOCFRC1.all = 0x01;

	           //Wait for end of conversion.
	           while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0){}  //Wait for ADCINT1
	           AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1
	           analogRead_F= AdcResult.ADCRESULT0;// / (4067);
	         //  if (analogRead_F <=3){analogRead_F =0;}
	           analogRead_F =analogRead_F /20;
	           analogRead_F = analogRead_F*20;

	         //  PotRead_I = map(analogRead_F,0,4065,0,MaxWaveFreq_I);//return analog range to desired frequency range 0-4065 to 0-300Hz
	          // PotRead_I = map(analogRead_F,0,4065,0,MaxWaveFreq_I);
	           VoltageSetpoint_F = map(analogRead_F,20+OffsetPot,4050,0+PWMOffsetPerC,MaxPWM_Perc_F+PWMOffsetPerC);//max is scalled from 0-300 to 0-50% since motor is reated 110V/300Hz ~ max voltage is 320
	           WaveStepSize_f = map(analogRead_F,5,4050.0,720.0,1);// number of steps
	          // VoltageSetpoint_F = map(PotRead_I,0,MaxWaveFreq_I,0,MaxPWM_Perc_F);//max is scalled from 0-300 to 0-50% since motor is reated 110V/300Hz ~ max voltage is 320
	          // WaveStepSize_f = map(PotRead_I,0,MaxWaveFreq_I,720,1);// number of steps
	           WavePerStepSize_f =720/WaveStepSize_f;
	           FloatToInt_i = WavePerStepSize_f;

	           while (x==30000){
	           Delata_f = WavePerStepSize_f -FloatToInt_i;
	           StepSize_i = Delata_f >= 0.50? WavePerStepSize_f + 1.0: WavePerStepSize_f;
	          //StepSize_i = 293;
	           //Get temp sensor sample result from SOC0
	          // epwm1_info.EPwmMaxCMPA = WaveStepSize_f;      // Setup min/max CMPA/CMPB values
	       //    epwm1_info.EPwmMaxCMPB = WaveStepSize_f;//EPWM1_MAX_CMPB;
	       //    epwm2_info.EPwmMaxCMPA = WaveStepSize_f;      // Setup min/max CMPA/CMPB values
           //    epwm2_info.EPwmMaxCMPB = WaveStepSize_f;//EPWM1_MAX_CMPB;
	           x =0;
	           }
x++;
      __asm("          NOP");
   }

}


/*********************************
__interrupt void epwm2_isr(void)
{

   // Update the CMPA and CMPB values
   update_compare(&epwm2_info);

   // Clear INT flag for this timer
   EPwm2Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm3_isr(void)
{

   // Update the CMPA and CMPB values
   update_compare(&epwm3_info);

   // Clear INT flag for this timer
   EPwm3Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
*/
void InitEPwm1Example()
{

   // Setup TBCLK
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   // Setup shadow register load on ZERO
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = EPWM1_MIN_CMPA;    // Set compare A value
   EPwm1Regs.CMPB = EPWM1_MIN_CMPB;              // Set Compare B value

   // Set actions
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count

   EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B, up count

   // Interrupt where we will change the Compare Values
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
   epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA & CMPB
   epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_UP;
   epwm1_info.EPwmTimerIntCount = 0;             // Zero the interrupt counter
   epwm1_info.EPwmRegHandle = &EPwm1Regs;        // Set the pointer to the ePWM module
   epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;      // Setup min/max CMPA/CMPB values
   epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
   epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPA;//EPWM1_MAX_CMPB;
   epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
   epwm1_info.EPwmSetPoint=0; //*** this is for POT
   epwm1_info.SinCos0_values =0;
   epwm1_info.ULLatch =1;
   epwm1_info.SinORCos = 0; //0=sine 1=cos
   //***epwm1 was declared for structure;
   //***EPwmMinCMPB are elements in theh structure;
   //***EPWM1_MIN_CMPB defined integer user define to load to the stru eles.
   //***EPWM_CMP_UP & EPWM_CMP_DOWN (This function only for initing and only exectued onece at the begining)
   //***EPwmTimerIntCount = +1 for ever single time an interrupt is interrupted
}

void InitEPwm2Example()
{
   // Setup TBCLK
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;       // Set timer period
   EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   // Setup shadow register load on ZERO
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = EPWM2_MIN_CMPA;       // Set compare A value
   EPwm2Regs.CMPB = EPWM2_MAX_CMPB;                 // Set Compare B value
/*
   // Set actions
   EPwm2Regs.AQCTLA.bit.PRD = AQ_CLEAR;             // Clear PWM2A on Period
   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;               // Set PWM2A on event A, up count

   EPwm2Regs.AQCTLB.bit.PRD = AQ_CLEAR;             // Clear PWM2B on Period
   EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;               // Set PWM2B on event B, up count
*/

   // Set actions
   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count

   EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
   EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B, up count

   // Interrupt where we will change the Compare Values
   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;        // Select INT on Zero event
   EPwm2Regs.ETSEL.bit.INTEN = 1;                   // Enable INT
   EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;              // Generate INT on 3rd event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
   epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;    // Start by increasing CMPA
   epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;  // and decreasing CMPB
   epwm2_info.EPwmTimerIntCount = 0;                // Zero the interrupt counter
   epwm2_info.EPwmRegHandle = &EPwm2Regs;           // Set the pointer to the ePWM module
  /* epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;         // Setup min/max CMPA/CMPB values
   epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
  epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
   epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;*/
   epwm2_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;      // Setup min/max CMPA/CMPB values
   epwm2_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
   epwm2_info.EPwmMaxCMPB = EPWM1_MAX_CMPA;
   epwm2_info.EPwmMinCMPB = EPWM1_MIN_CMPB;

   epwm2_info.EPwmSetPoint=0; //*** this is for POT
   epwm2_info.SinCos0_values =0;
   epwm2_info.ULLatch =1;// initialize to have the upper waveform go first.
   epwm2_info.SinORCos = 1; //0=sine 1=cos

}

void InitEPwm3Example(void)
{


	//(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
   // Setup TBCLK
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm3Regs.TBPRD = EPWM3_TIMER_TBPRD;       // Set timer period
   EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm3Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadow register load on ZERO
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
   EPwm3Regs.CMPA.half.CMPA = EPWM3_MIN_CMPA; // Set compare A value
   EPwm3Regs.CMPB = EPWM3_MAX_CMPB;           // Set Compare B value

   // Set Actions
   EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;         // Set PWM3A on event B, up count
   EPwm3Regs.AQCTLA.bit.CBU = AQ_CLEAR;       // Clear PWM3A on event B, up count

   EPwm3Regs.AQCTLB.bit.ZRO = AQ_TOGGLE;      // Toggle EPWM3B on Zero

   // Interrupt where we will change the Compare Values
   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   // Start by increasing the compare A and decreasing compare B
   epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
   epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;
   // Start the cout at 0
   epwm3_info.EPwmTimerIntCount = 0;
   epwm3_info.EPwmRegHandle = &EPwm3Regs;
   epwm3_info.EPwmMaxCMPA = EPWM3_MAX_CMPA;
   epwm3_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
   epwm3_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
   epwm3_info.EPwmMinCMPB = EPWM3_MIN_CMPB;
   epwm3_info.EPwmSetPoint=0; //*** this is for POT
   epwm3_info.SinCos0_values =0;
   epwm3_info.ULLatch =1;
   epwm3_info.SinORCos = 2;
}

__interrupt void epwm1_isr(void)
{

	//if(epwm1_info.EPwmTimerIntCount >= epwm1_info.EPwmSetPoint) {

    //Update the CMPA and CMPB values
	/*    Remainder = 300 % WaveStepSize_f;
	    wholeNum = 300/WaveStepSize_f;
	    if(Remainder > 0){wholeNum = wholeNum +1; }
	    wholeNum = WaveStepSize_f * wholeNum; */
	    //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	   // float map(float x, float in_min,float in_max,float out_max,float out_min)

	   // SinPointer =map(SinPointer,1, wholeNum, 1, 300);
	  //  SinPointer



		switch (WAVETickeer)
		{
		case 1:
			SinPointer += epwm1_info.EPwmSetPoint;
			CoSPointer -= epwm1_info.EPwmSetPoint;
			break;
		case 2:
			SinPointer -= epwm1_info.EPwmSetPoint;
			CoSPointer += epwm1_info.EPwmSetPoint;
			break;
		case 3:
			SinPointer += epwm1_info.EPwmSetPoint;
			CoSPointer -= epwm1_info.EPwmSetPoint;
			break;
		case 4:
			SinPointer -= epwm1_info.EPwmSetPoint;
			CoSPointer += epwm1_info.EPwmSetPoint;
			break;
		default:
			break;

		}
		if (SinPointer>=720 || CoSPointer <=0){ SinPointer= 720; CoSPointer =0; WAVETickeer +=1;UpdateStep_Size =0;}
			else if (CoSPointer>=720 || SinPointer <=0){SinPointer= 0; CoSPointer =720; WAVETickeer +=1; UpdateStep_Size = 0;}
		if (WAVETickeer == 5){WAVETickeer =1;}
		if (UpdateStep_Size == 0){
				epwm1_info.EPwmSetPoint = StepSize_i;
	            epwm2_info.EPwmSetPoint = StepSize_i;
	            UpdateStep_Size = 1;
			}

			epwm1_info.SinCos0_values = map(Sin_CosP90values[SinPointer], 0, 10000, 0, VoltageSetpoint_F);
			epwm2_info.SinCos0_values = map(Sin_CosP90values[CoSPointer], 0, 10000, 0, VoltageSetpoint_F);
			update_compare(&epwm1_info,&epwm2_info);
			//update_compare(&epwm1_info);
			//epwm1_info.EPwmRegHandle.CMPB= 0;
		//	epwm1_info.EPwmRegHandle.CMPA.half.CMPA= 0.2;//(epwm_info->EPwmMaxCMPA/5)


	EPwm1Regs.ETCLR.bit.INT = 1;
	EPwm2Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
void update_compare(EPWM_INFO * epwm_info1,EPWM_INFO *epwm_info2)
//void update_compare(EPWM_INFO * epwm_info)
{

	//PWMMapping= map(float mjk, float in_min,float in_max,float out_max,float out_min)
	switch (WAVETickeer)
	{
	case 1:
			epwm_info1->EPwmRegHandle->CMPB= 0;
			epwm_info1->EPwmRegHandle->CMPA.half.CMPA= epwm_info1->SinCos0_values;
		epwm_info2->EPwmRegHandle->CMPB= 0;
			epwm_info2->EPwmRegHandle->CMPA.half.CMPA= epwm_info2->SinCos0_values;


			//epwm_info2->EPwmRegHandle->CMPB= 0;
		//epwm_info2->EPwmRegHandle->CMPA.half.CMPA= 0;
			break;
		case 2:
			epwm_info1->EPwmRegHandle->CMPB= 0;
			epwm_info1->EPwmRegHandle->CMPA.half.CMPA= epwm_info1->SinCos0_values;
			epwm_info2->EPwmRegHandle->CMPA.half.CMPA= 0;
			epwm_info2->EPwmRegHandle->CMPB= epwm_info2->SinCos0_values;

			//epwm_info1->EPwmRegHandle->CMPB= 0;
			//epwm_info1->EPwmRegHandle->CMPA.half.CMPA= 0;
			//epwm_info2->EPwmRegHandle->CMPB= 0;
			//epwm_info2->EPwmRegHandle->CMPA.half.CMPA= 0;
			break;
		case 3:
			epwm_info1->EPwmRegHandle->CMPA.half.CMPA= 0;
			epwm_info1->EPwmRegHandle->CMPB= epwm_info1->SinCos0_values;

			epwm_info2->EPwmRegHandle->CMPA.half.CMPA= 0;
			epwm_info2->EPwmRegHandle->CMPB= epwm_info2->SinCos0_values;
			//epwm_info2->EPwmRegHandle->CMPB= epwm_info2->SinCos0_values;

			//epwm_info1->EPwmRegHandle->CMPB= 0;
			//epwm_info1->EPwmRegHandle->CMPA.half.CMPA= 0;
			//epwm_info2->EPwmRegHandle->CMPB= 0;
			//epwm_info2->EPwmRegHandle->CMPA.half.CMPA= 0;
			break;
		case 4:
			epwm_info1->EPwmRegHandle->CMPA.half.CMPA= 0;
			epwm_info1->EPwmRegHandle->CMPB= epwm_info1->SinCos0_values;
			epwm_info2->EPwmRegHandle->CMPB= 0;
			epwm_info2->EPwmRegHandle->CMPA.half.CMPA= epwm_info2->SinCos0_values;

			//epwm_info1->EPwmRegHandle->CMPB= 0;
			//epwm_info1->EPwmRegHandle->CMPA.half.CMPA= 0;
			//epwm_info2->EPwmRegHandle->CMPB= 0;
			//epwm_info2->EPwmRegHandle->CMPA.half.CMPA= 0;
			break;
		default:
			break;
	}


//	if (SinCos_TICKER <=180){
	//	PWMMapping = (epwm_info1->SinCos0_values) * (epwm_info1->EPwmMaxCMPA);
	//	epwm_info->EPwmRegHandle->CMPB= 600;
	//	epwm_info->EPwmRegHandle->CMPA.half.CMPA=600;//(epwm_info->EPwmMaxCMPA/5)
	//}
	//else {
	//	PWMMapping = (epwm_info1->SinCos0_values) * (epwm_info1->EPwmMaxCMPA);
	//	epwm_info1->EPwmRegHandle->CMPA.half.CMPA= 0;
	//	epwm_info1->EPwmRegHandle->CMPB= PWMMapping;
	//	}


    return;
}
//(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
float map(float x, float in_min,float in_max,float out_min,float out_max)
{

return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

	//return 720/(((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)*2250/analogRead_F);

}
//===========================================================================
// No more.
//===========================================================================
