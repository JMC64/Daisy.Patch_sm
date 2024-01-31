
// Based on rg00 code ( a lot of copy paste !), here is a clockable LFO with 8 outputs
// trigger has to be sent to input B10




//Demo code for the FHX-8CV expander, attached to patch.init() with the patch_sm (this also has some code for the FHX-8GT)
//This demo isn't a finished application but is a simple emulation of the Instruo/Divkid Ochd
//The code runs an infinite loop clocking 8x counters at different increments and these form LFOs (RG 2023)

//This demo uses some code that is installed with the Electrosmith Daisy development examples
//This should get loaded on the local HDD at C:\Users\...\Desktop\DaisyExamples
//It uses some code from C:\Users\...\Desktop\DaisyExamples\patch_sm\GettingStarted\Momentary_Switch
//and the SPI code from C:\Users\...\Desktop\DaisyExamples\libDaisy\examples\SPI\SpiBlockingTransmit

//see also:
//SPI REF documemntation - https://electro-smith.github.io/libDaisy/md_doc_2md_2__a8___getting-_started-_s_p_i.html
//SPI REF code notes: https://github.com/electro-smith/libDaisy/blob/master/doc/md/_a8_Getting-Started-SPI.md
//GPIO see https://github.com/electro-smith/libDaisy/blob/master/doc/md/_a1_Getting-Started-GPIO.md
//ADCs see https://github.com/electro-smith/libDaisy/blob/master/doc/md/_a4_Getting-Started-ADCs.md
//patch pin migration: https://github.com/electro-smith/libDaisy/pull/584/commits/f13f0fa23b193a5275a61d0807a7a08ce96ffcd1
//lower level code https://github.com/electro-smith/libDaisy/blob/master/src/daisy_patch_sm.cpp

#include "daisy_patch_sm.h"
#include "daisysp.h"
#include <stdbool.h>

#include <cmath>
#include <cstdint>

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

//============================================
// this is stuff for running 8 LFOs whiuch emulate Ochd

const int LFO_COUNT = 8;


//============================================

//define the voltage scale settings for FHX-8CV.  These are 2-byte values used to set the output volt scale per channel
//N.B these are 2 byte codes, but do not sit on the byte boundary in the SPI packet

#define SCALE_ZERO_TEN 0x7DDC
//#define SCALE_FIVE_FIVE 0x7DDC0 //this isn't needed, we can use the above scale
#define SCALE_ZERO_ONE 0x0C96
#define SCALE_ZERO_FIVE 0x3EEE
#define SCALE_ZERO_EIGHT 0x64AF

//define the FHX-8CV channel mapping: the output jack number in terms of the binary address of the jack
//NB these channel addresses are a single nibble (3 bits with 0b0 padded at left) and this nibble is right padded with 0b0000
//It's for coding ease so we can '|' (bitwise or) the channel into the scale factor.  E.g. ch3 is 0x4 but we define it 0x40 below

#define FHX8CV_CH1 0x00
#define FHX8CV_CH2 0x20
#define FHX8CV_CH3 0x40
#define FHX8CV_CH4 0x60
#define FHX8CV_CH5 0x70
#define FHX8CV_CH6 0x50
#define FHX8CV_CH7 0x30
#define FHX8CV_CH8 0x10

//define some bytes for the various FHX SPI packet headers.  These designate a packet to go to a particular module type

#define SPI_PKT_8GT 0x04            //the packet header for the FHX-8GT
#define SPI_PKT_8CV_VOLTAGE 0x03    //the packet header to send voltage data to a single channel of the  FHX-8CV
#define SPI_PKT_8CV_CONFIG 0x14     //the packet header to send 8-bit channel offset info to the 8 FHX-8CV channels

//============================================

// Hardware object for the patch_sm
DaisyPatchSM hw;

// Switch object - here we use the switch to choose unipolar or bipolar CV outs
Switch button, toggle;

// Create dsy_gpio objects allowing the use of three pins on the Patch-SM port A - we use them for the addess pins on the FHX modules
dsy_gpio patch_A3; //will use for FHX A0
dsy_gpio patch_A8; //will use for FHX A1
dsy_gpio patch_A9; //will use for FHX A2

//GPIO patch_A3; //this approach seems buggy in the patch-sm implementation

// Handle we'll use to interact with SPI
SpiHandle spi_handle;

// Structure to configure the SPI handle
SpiHandle::Config spi_conf;

//Non-blocking SPI: must use a special memory section for DMA (if we indeed decided to use non-blocking...)
//uint8_t DMA_BUFFER_MEM_SECTION buffer[4] = {0, 1, 2, 3};
//BTW we're not actually using this in the demo - TBD future enhancement?
uint8_t DMA_BUFFER_MEM_SECTION dma_buffer[4]; //note this can't be declared as a local


//============================================
// My variable to synchronize the  LFO
u_int32_t elapsed_time, t0, t1,time_step, ts1,ts2,tsled ;
u_int32_t ts0[8];
double LFO_angle[8];
double step_duration[8]= {50,50,50,50,50,50,50,50 };
double LFO_factor[8]= {1,1.5,2,2.5,3,4,5,8 };  // modify the LFO speed relatively to the incoming clock
//============================================

//this is code from one of the Dasiy samples, we're not using it here.  

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	hw.ProcessAllControls();
    //button.Debounce();  //added in while loop for now...
    //toggle.Debounce();
	for (size_t i = 0; i < size; i++)
	{
		OUT_L[i] = IN_L[i];
		OUT_R[i] = IN_R[i];
	}
}

//============================================

//function to initialise the three GPIO pins we are using for addressing the FHX modules

void InitialisePatchGpioFhxAddr() //Set up three header pins for addessing the FHX, uses globals patch_A3, patch_A8, patch_A9
{
	//patch_A3.Init(hw.A3, GPIO::Mode::OUTPUT);  //this approach seems not working, use below instead
	//patch_A3.Init(A3);

	patch_A3.mode = DSY_GPIO_MODE_OUTPUT_PP; //for FHX A0 - coded  here using the old dsy_gpio style
    patch_A3.pull = DSY_GPIO_NOPULL;
    patch_A3.pin = hw.A3;
    dsy_gpio_init(&patch_A3);
    dsy_gpio_write(&patch_A3, 0);

	patch_A8.mode = DSY_GPIO_MODE_OUTPUT_PP; //for FHX A1 - coded  here using the old dsy_gpio style
    patch_A8.pull = DSY_GPIO_NOPULL;
    patch_A8.pin = hw.A8;
    dsy_gpio_init(&patch_A8);
    dsy_gpio_write(&patch_A8, 0);

	patch_A9.mode = DSY_GPIO_MODE_OUTPUT_PP; //for FHX A2 - coded  here using the old dsy_gpio style
    patch_A9.pull = DSY_GPIO_NOPULL;
    patch_A9.pin = hw.A9;
    dsy_gpio_init(&patch_A9);
    dsy_gpio_write(&patch_A9, 0);
}

//function to configure SPI so we can use it

void ConfigurePatchSpi() //configure the SPI bus for FHX on pins D0, D10, D1 - uses global spi_conf (and SpiHandle if we want to scale the clock)
{
	spi_conf.mode = SpiHandle::Config::Mode::MASTER; // The patch.Init is Main on SPI bus
	spi_conf.periph = SpiHandle::Config::Peripheral::SPI_2; // Use the SPI_2 Peripheral

	// Pins to use for SPI. These must be available on the selected peripheral
	spi_conf.pin_config.mosi = DaisyPatchSM::D9; // Use D9 as MOSI
	spi_conf.pin_config.miso = Pin(); // We won't need this
	spi_conf.pin_config.sclk = DaisyPatchSM::D10; // Use pin D10 as SCLK but NB data on risding clock while ESX is on falling edge
	spi_conf.pin_config.nss = DaisyPatchSM::D1; // use D1 as NSS

	// data will flow from main to sub over just the MOSI line
	spi_conf.direction = SpiHandle::Config::Direction::TWO_LINES_TX_ONLY;

	// Main will output on the NSS line
	spi_conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;

	//set some stuff for the SPI mode: ONE_EDGE is when we transition to the clock polarity. TWO_EDGE is when we transition away from the clock polarity
	spi_conf.clock_polarity = SpiHandle::Config::ClockPolarity::LOW; //default is LOW, otherwise can set HIGH
	spi_conf.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE; //default is ONE_EDGE

	//Number of bits to tx. Defaults to 8. Must be in the range [4, 32] inclusive.
	spi_conf.datasize = 32;

	//The clock rate is 25MHz. With a prescaler of 4 the final clock rate is 25MHz / 4 = 6.25MHz (Default SpiHandle::Config::BaudPrescaler::PS_8)
	//SpiHandle::Config::BaudPrescaler::PS_8
}

//function we can use to set an three-bit address on the GPIO pins, we call it with 8 bits but it only uses the three LSBs

void SetFhxAddr(uint8_t FhxAddr)
{
 	uint8_t LsbMask = 0x01; // the mask is 0b00000001 in binary to get the Least Significan Bit

	//mask out the relevant binary bits of the expander address and set up values to send to each GPIO pin
	uint8_t ValAddr0 = FhxAddr & LsbMask;
	uint8_t ValAddr1 = (FhxAddr >> 1) & LsbMask;
	uint8_t ValAddr2 = (FhxAddr >> 2) & LsbMask;

	dsy_gpio_write(&patch_A3, ValAddr0); //patch_A3 pin is wired to FHX A0 address line
	dsy_gpio_write(&patch_A8, ValAddr1); //patch_A8 pin is wired to FHX A1 address line
	dsy_gpio_write(&patch_A9, ValAddr2); //patch_A9 pin is wired to FHX A2 address line
}

//function to fake that a FH2 (FaderHost) is connected to the FHX - it checks on power-up and flashes the LEDs as a warning
//Expert Sleepers sends this 32 bit packet shortly after bootup, it seems to suppress the flashing FHX lights (they flash when it thinhs no FH2 is connected)

void SendFhxBootBytes() //Upon bootup the FHT will send 0x08000001 to the FHX expanders
{
	const uint8_t FhxAddr = 0; //from the ES protocol it looks like this is sent on boot with addresses zero
	SetFhxAddr(FhxAddr);
	//System::Delay(10);  //Time for whatever the expanders need to do with this address data....

	uint8_t spi_buffer[4] = {0x01, 0x00, 0x00, 0x08}; //NB the buffer plays in reverse bytes order...

	// Blocking SPI transmit those 4 bytes
	spi_handle.BlockingTransmit(spi_buffer, 4);
}





//Send volts to FHX-8CV - Blocking SPI transmit to one channel only.
//TBD add expander base number and auto call to the relevant GPIO address lines, if multi expanders in use??

void Sendfhx8cvVolts(uint8_t fhx8cvChannelNum, uint16_t fhx8cvOutputVoltage)
{
	uint8_t spi_buffer[4];  //N.B. buffer[3] sent first and buffer[0] is last
	const uint8_t FhxAddr = 1; //address the 8CV at '0b001' i.e. one expander with its config jumpers at 1/1

	SetFhxAddr(FhxAddr);
	spi_buffer[3] = SPI_PKT_8CV_VOLTAGE;  
	spi_buffer[2] = fhx8cvChannelNum | ((fhx8cvOutputVoltage >> 12) & 0x0F); //shift the upper nibble of the fhx8cvOutputVoltage to LS nibble, cautiously mask (in case crap was shifted in) and bitwise or
	spi_buffer[1] = fhx8cvOutputVoltage >> 4; //we use the middle 2 nibbles of the fhx8cvOutputVoltage value in this byte
	spi_buffer[0] = (fhx8cvOutputVoltage << 4) & 0xF0; //we use the lowest nibble of 'fhx8cvOutputVoltage' in the upper nibble of this byte and causiouly mask in 0x0 as the lower nibble
	spi_handle.BlockingTransmit(spi_buffer, 4);
}


//Send 8x values of voltage to FHX-8CV - Blocking SPI transmit array of 8 voltages to 8 FHX channels.
//It works just like setting one channel but iterates across 8x uint16_t values packed within an 8-d array, and it maps array index to expander channel
//TBD add in expander base number and auto call to the relevant address lines, if multi expanders in use??

void Sendfhx8cvValues(uint16_t fhx8cvOutputValues[8])
{
	uint8_t spi_buffer[4];  //N.B. buffer[3] sent first and buffer[0] is last
	uint8_t fhx8cvChannelNum;
	uint16_t fhx8cvOutputVoltage;
	const uint8_t FhxAddr = 1; //address the 8CV at '0b001' i.e. one expander with its config jumpers at 1/1

	SetFhxAddr(FhxAddr);
	for(uint8_t i=0; i<8; i++)
	{
		fhx8cvOutputVoltage = fhx8cvOutputValues[i];
		fhx8cvChannelNum = i<<4; //N.B. the way we did the counter 'i' means the channel numbers are in the wrong nibble
		spi_buffer[3] = SPI_PKT_8CV_VOLTAGE;  
		spi_buffer[2] = fhx8cvChannelNum | ((fhx8cvOutputVoltage >> 12) & 0x0F); //shift the upper nibble of the fhx8cvOutputVoltage to LS nibble, cautiously mask (in case crap was shifted in) and bitwise or
		spi_buffer[1] = fhx8cvOutputVoltage >> 4; //we use the middle 2 nibbles of the fhx8cvOutputVoltage value in this byte
		spi_buffer[0] = (fhx8cvOutputVoltage << 4) & 0xF0; //we use the lowest nibble of 'fhx8cvOutputVoltage' in the upper nibble of this byte and causiouly mask in 0x0 as the lower nibble
		spi_handle.BlockingTransmit(spi_buffer, 4);
	}
}


//configure all 8 FHX-8CV channels for polarity of voltage
//TBD run this over all 7 (or 8) addressed FHX-8CV expanders, if multi expanders in use??

void SendConfig8cv(uint8_t fhx8cvConfigByte)
{
	uint8_t spi_buffer[4];  //N.B. buffer[3] sent first and buffer[0] is last
	const uint8_t FhxAddr = 1; //address the 8CV at '0b001' i.e. one expander with its config jumpers at 1/1

	SetFhxAddr(FhxAddr);
	System::Delay(1);  //Time for whatever the expanders to receive this address data....
	spi_buffer[3] = SPI_PKT_8CV_CONFIG;  
	spi_buffer[2] = 0x00;
	spi_buffer[1] = fhx8cvConfigByte; //binary - each bit is a flag bipolar or not
	spi_buffer[0] = 0x00; 
	spi_handle.BlockingTransmit(spi_buffer, 4);


}


//Function to set up a 0V to 10V config on all 8 channels of the FHX-8CV

void SetConfig8cv0v10v()
{
	uint16_t fhx8cvOutputValues[8];
	uint8_t fhx8cvConfigByte;
	//pre config - this part first sets the midpoint voltage values, it's what ES does
	fhx8cvOutputValues[0] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[1] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[2] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[3] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[4] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[5] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[6] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[7] = SCALE_ZERO_TEN;
	Sendfhx8cvValues(fhx8cvOutputValues);

	//then this part configures the FHX--8CV voltage polarity, all 8x channels
	fhx8cvConfigByte = 0x00; //this should be all 8 channels unipolar ** CONFIG 00 **
	SendConfig8cv(fhx8cvConfigByte);
}

//Function to set up a -5V to +5V config on the FHX-8CV
void SetConfig8cv5v5v()
{
	uint16_t fhx8cvOutputValues[8];
	uint8_t fhx8cvConfigByte;
	//pre config - this part first sets the midpoint voltage values, it's what ES does
	fhx8cvOutputValues[0] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[1] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[2] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[3] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[4] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[5] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[6] = SCALE_ZERO_TEN;
	fhx8cvOutputValues[7] = SCALE_ZERO_TEN;
	Sendfhx8cvValues(fhx8cvOutputValues);

	//then this part configures the FHX--8CV voltage polarity, all 8x channels
	fhx8cvConfigByte = 0xFF; //this should be all 8 channels bipolar ** CONFIG FF **
	SendConfig8cv(fhx8cvConfigByte);
}


//####################################################

int main(void)
{
	// Initialize the patch_sm hardware object
	hw.Init();

	//this is stuff for the default audio passthrough...which we are not actually using in this demo!
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAudio(AudioCallback);
//	hw.StartLog(true); 
	// Initialize the switches - We'll read the toggle switch on pin B7 and set the outputs unipolar or bipolar accordingly
    button.Init(hw.B7);
    toggle.Init(hw.B8);

	//Initialise 3 GPIO outs on PatchSM pins A3, A8, A9 for addressing the FHX expanders, and set them 0V for now
	InitialisePatchGpioFhxAddr();

	//Configure the SPI on PatchSM pins D0, D10, D1, set up with the ExpertSleepers SPI implementation params
	ConfigurePatchSpi();

	// Initialize the SPI Handle
	spi_handle.Init(spi_conf);
	
	
	uint16_t fhx8cvOutputValues[8];
	bool switchStatus = false;
	bool priorSwitchStatus = true;

	//fake that a FH2 is driving the FHX expanders, it stops the expanders flashing a warning

	SendFhxBootBytes();  //this sends 0x08000001 just like the FH2 would, and then let's just wait a moment
	System::Delay(1000);

	// Init variables
	time_step= 100;
	ts1=50;
	ts2=0;
	//This segment has the Ochd LFO emulation - it's demo code



	// Declare an array to store the 8 instantaneous output values for the LFOs
	uint16_t outputValueArray[8];

	//SetConfig8cv0v10v();
	SetConfig8cv5v5v(); //let's start off -5V to +5V bipolar outs

	t0= System::GetNow();
	for (int i=0; i<LFO_COUNT;i++){
		ts0[i]=t0;
		};
	while(true) //infinite loop
	{
	
		//if the switch status changed then send the FHX-8CV a new config, per the switch, and initialise channel voltages to centre voltage
		if (switchStatus != priorSwitchStatus)
		{
			if(switchStatus == false)
			{
				SetConfig8cv0v10v();
				//initialise all FHX-8CV channel values to 0x7DDC, but we don't need to *send* them yet
				fhx8cvOutputValues[0] = 0x7DDC;
				fhx8cvOutputValues[1] = 0x7DDC;
				fhx8cvOutputValues[2] = 0x7DDC;
				fhx8cvOutputValues[3] = 0x7DDC;
				fhx8cvOutputValues[4] = 0x7DDC;
				fhx8cvOutputValues[5] = 0x7DDC;
				fhx8cvOutputValues[6] = 0x7DDC;
				fhx8cvOutputValues[7] = 0x7DDC;
			}
			else
			{
				SetConfig8cv5v5v();
				//initialise all FHX-8CV channel values to 0x0000, but we don't need to *send* them yet
				fhx8cvOutputValues[0] = 0x0000;
				fhx8cvOutputValues[1] = 0x0000;
				fhx8cvOutputValues[2] = 0x0000;
				fhx8cvOutputValues[3] = 0x0000;
				fhx8cvOutputValues[4] = 0x0000;
				fhx8cvOutputValues[5] = 0x0000;
				fhx8cvOutputValues[6] = 0x0000;
				fhx8cvOutputValues[7] = 0x0000;
			}
			priorSwitchStatus = switchStatus;
		}

		//looks odd to read the switch config here, but we need to set a config first (just above) and then correct it if the switch is different
    	toggle.Debounce();

		//read the switch
        switchStatus = toggle.Pressed();

        //Set the PatchSM rear-mounted onboard led to the current state
        hw.SetLed(switchStatus);
		
	
// get the time 
		if (hw.gate_in_1.Trig()){
			t1 = System::GetNow();
			elapsed_time =t1-t0;
			t0=t1;
			hw.WriteCvOut(CV_OUT_2, 2.5); // blink the led
		};

		ts1 = System::GetNow();
		if (ts1-tsled>=10){
			hw.WriteCvOut(CV_OUT_2, 0.0);
			tsled= ts1;
			};
		
		if ((ts1 - ts2)>=(elapsed_time/ 100)){     // we use 100 points to describe the cycle of each sine
			for (int i=0 ; i < LFO_COUNT;i++){
				LFO_angle[i]+= 2.0 * M_PI * 0.01 /LFO_factor[i];
				 double sineValue = std::sin(LFO_angle[i]);
				 double outputValue = (sineValue + 1.0) * 5.0;
				 outputValueArray[i] = static_cast<uint16_t>(outputValue * (65535.0 / 10.0));
			};
			ts2=ts1;
		};

	
//hw.PrintLine("");
		//unshuffle the outputs so it runs like ochd - this is a bit of a mind melt, but the ESX-8CV has a very weird binary addressing for the channels
		fhx8cvOutputValues[0] = outputValueArray[7];
		fhx8cvOutputValues[2] = outputValueArray[6];
		fhx8cvOutputValues[4] = outputValueArray[5];
		fhx8cvOutputValues[6] = outputValueArray[4];
		fhx8cvOutputValues[7] = outputValueArray[3];
		fhx8cvOutputValues[5] = outputValueArray[2];
		fhx8cvOutputValues[3] = outputValueArray[1];
		fhx8cvOutputValues[1] = outputValueArray[0];

		//now finally we update the FHX-8CV channel values to the actual expander
		Sendfhx8cvValues(fhx8cvOutputValues); 

	


	}
}