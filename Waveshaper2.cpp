/*
WAveshaper for patch.init()

TO add an inflexion point : 
	* Switch B8 = ON (up), and press button B7 once. Each  press adds a new point ( up to 48 then back to 3)
To change randomly the height of the inflexion points: 
	* Switch B8 = OFF (down), and press button B7 once. This change all the value  by a small random amount. 
	This is coupled to the value of the pot CV2 : The closest clockwise, the bigger the change is
	NB : the change can be positive or negative. The sign is inverted when a value goes beyong intervall [-1,1]
To change the spacing of the inflexion points :
	* Turn pot CV1 : full clockwise, equal spacing between the point 

Pitch can be receive (1V/oct) in CV6
An enveloppe or a LFO can be send to CV7 which is coupled to Pot CV4. This  consitutes the initial amplification of the oscillator 
=> therefore running more or less the interpolated curve



*/





#include "daisy_patch_sm.h"
#include "daisysp.h"
#include <vector>

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;
using namespace std;

DaisyPatchSM hw;
static Oscillator osc0;
Switch button,interrupteur;


// Inflexion points for calculating the waveshaping array: Vector for X values, Y values and the sign of the change with random number ( true : increase, false: decrease))
vector <int> inflexionPtX;  
vector <float> inflexionPtY;
vector <bool> inflexionSign;

int nb_inflexion=3;
int max_inflexion_index= nb_inflexion-1;
int max_inflexion = 48;

//waveshaping array
int MAX_POINT = 1024;
vector <float> waveshape_table(MAX_POINT); //[num_points];
vector <float> waveshape_table_tmp(MAX_POINT);//[num_points]; // create an another array for calculation

bool Flg_end_calculation = false;  // used to allow copying the newly calulated waveshape to the readable one for the oscillator


// calculate the absciss of the inflexion point which are controled by a pot => Inflexion points are not necessarily equally spaced
void def_abscisse_vector (float v) {
  float m = (float) (MAX_POINT -1);
  float n = (float)max_inflexion_index;
  for (int i =0 ; i <= max_inflexion_index; i++){
	float q = (float)i / n;
	q= pow(q,v);
	float res = q*m;
	int abs = (int)(res + 0.5);
	inflexionPtX.at(i)=abs;
  };

}

// Used to generated randomly a sign for the Y of the inflexion points
bool randomSign(){
	float p=(float) hw.GetRandomValue()*0.99 /4294967296.0 -0.5 ;
 
    if (p>=0){
        return true;
    }
    else {
        return false;
	};
}



float newrandom( bool &sign, float old_point, float control){
	float y=0.0;
	float p=(float) hw.GetRandomValue() /4294967296.0 * control /10.0 ;

	if (sign){
		y= old_point+ p ;
		}
	else{
		y= old_point- p ;
		};
// if the value is going beyond [-1,1], bring it back to the intervall and invert the sign for further changes 
	if (y >=0.99){
		y= old_point- 2*p ;
		sign=false;
		}
	if (y <=-0.99){
		y= old_point+ 2*p ;
		sign=true;
		};		
	return y;
}

// cosine interpolation : used when necessary.. faster than Lagrange, I think
float cosin_interpol(float x,float y1,float y0){  

// En theorie, on devrait passer les paramètres suivants
//float cosin_interpol(floatx,float y1,float y0,float x1,float x0){
// 0< x < 1 : (frac0), all point are equally space as this routine is called from audio loop 
//Formule generique : 
 // y  =0.5* (cos ( M_PI * (x -x0)/(y1-y0) ) -1) *(y0-y1)  + y0
// on passe directemet x-x0 et y1-y0 =1
return 0.5*(cos( x*M_PI )-1.0)*(y0-y1)+ y0;

}


// Lagrange interpolation interpolation to calulate the waveshaping curve
float Lagrange(int x){

	int i=0;
	float x1;
	float x2;
	float x3;
	float y1;
	float y2;
	float y3;
	
	while(inflexionPtX.at(i)<= x) { 
		i++;
	};

  	if ( i>= 1 && i< max_inflexion_index ){
		x1= inflexionPtX.at(i-1);
		x2= inflexionPtX.at(i);
		x3= inflexionPtX.at(i+1);
		y1= inflexionPtY.at(i-1);
		y2= inflexionPtY.at(i);
		y3= inflexionPtY.at(i+1);	
	};

	if (i ==0) {		
		x1= inflexionPtX.at(0);
		x2= inflexionPtX.at(1);
		x3= inflexionPtX.at(2);
		y1= inflexionPtY.at(0);
		y2= inflexionPtY.at(1);
		y3= inflexionPtY.at(2);
	};

	if (i >=max_inflexion_index) {	
		x1= inflexionPtX.at(max_inflexion_index-2);  
		x2= inflexionPtX.at(max_inflexion_index-1);
		x3= inflexionPtX.at(max_inflexion_index);
		y1= inflexionPtY.at(max_inflexion_index-2);
		y2= inflexionPtY.at(max_inflexion_index-1);
		y3= inflexionPtY.at(max_inflexion_index);
	}; 

	float a = (x1*(y3-y2) + x2*(y1-y3)+ x3*(y2-y1)) / ((x1-x2 )* (x1-x3)*(x2-x3));
	float b = (y2-y1)/(x2-x1) - a* (x1+x2);
	float c = y1 - a*x1*x1- b*x1;
	return a*x*x +b*x +c; 
}

void controls(){
	// variable for the harware reading
	static float oldv1=-1.0;
	static float oldv2=-1.0;
	static float oldv3=-1.0;
	static float oldv4=-1.0;
	static bool state_switch;
	static bool button_getting_pressed ;
	bool flg_calculate_Lagrange=false;
	static bool flagv2chng = false;
	float v1 = hw.GetAdcValue(CV_1);
	float v2 = hw.GetAdcValue(CV_2);
	float v3 = hw.GetAdcValue(CV_3);
	float v4 = hw.GetAdcValue(CV_4);

	static bool temp_sign;
	v1=v1+0.1;
	v1=fclamp(v1,0.1,1.0);
	v4=fclamp(v4,0.1,1.0);

	interrupteur.Debounce();
	button.Debounce();
	state_switch = interrupteur.Pressed();
	button_getting_pressed = button.RisingEdge();
	
	float v6 = hw.GetAdcValue(CV_6);
	float v7 = hw.GetAdcValue(CV_7);
	float pitch = fmap(v6, 0.f, 60.f);
	float midi_nn = fclamp(pitch+24.0f, 0.f, 127.f);
    float freq    = mtof(midi_nn);
	
	osc0.SetAmp(v4*v7); // this is for fun.. need to dig into this  : coupling pot  with CV input
	osc0.SetFreq(freq);
	
//if user press the button and the switch is ON => we add  one inflexion point
	if(state_switch && button_getting_pressed ){
		//hw.PrintLine("condition vraie pour calcul abscisse");
		if (nb_inflexion < max_inflexion){
		//	hw.PrintLine("nb_inflexion < max_inflexion");
					nb_inflexion++;
					max_inflexion_index=nb_inflexion-1;
					// resize the vectors
					inflexionPtX.resize(nb_inflexion);
					inflexionPtY.resize(nb_inflexion);
					inflexionSign.resize(nb_inflexion);
				//	hw.PrintLine("vector resized to %d",nb_inflexion);
					inflexionSign.at(max_inflexion_index-1)= randomSign();  //affect a random sign to the newly created inflexion point
				//	hw.PrintLine("randomsize fait");
					//inflexionPtY.at(max_inflexion_index-1)=(hw.GetRandomFloat(0.0f,1.0f)*0.99)-0.5f;// give a Y random value to newly created point
					inflexionPtY.at(max_inflexion_index-1)=(float) hw.GetRandomValue()*0.99 /4294967296.0 -0.5 ;
				//	hw.PrintLine("randoms value fait");
					inflexionPtY.at(max_inflexion_index)=1.0;	// set the last point at 1.0
				//	hw.PrintLine("maxinflexion à 1  fait");
					//inflexionPtX.at(max_inflexion_index)=1023;
				}
		else {
				nb_inflexion=3;
				max_inflexion_index=nb_inflexion-1;
				inflexionPtX.resize(nb_inflexion);
				inflexionSign.resize(nb_inflexion);          
				inflexionPtY.resize(nb_inflexion);
				inflexionSign.at(max_inflexion_index)=false;
				inflexionPtY.at(max_inflexion_index)=1.0;
			};
		def_abscisse_vector (v1) ;  // recalculate the abscisses 	
		//hw.PrintLine("def abscisse vector fait");
		flg_calculate_Lagrange=true;
	};

//if user press the button and the switch is OFF => we had change the values of the inflexion points
// according to their sign and the value of the pot N°2 ( CV_2)
 	if (!state_switch && button_getting_pressed){
		for  (int i=1 ; i<= max_inflexion_index    ;  i++){
			 bool temp_sign =inflexionSign.at(i);
			 float valuetmp = inflexionPtY.at(i);
			 inflexionPtY.at(i)= newrandom(temp_sign,valuetmp,oldv2);
			 inflexionSign.at(i)=temp_sign;
		};
		inflexionPtY.at(0)=-1.0f; // forcing first and last value
		inflexionSign.at( max_inflexion_index)=false;
 		inflexionPtY.at( max_inflexion_index)=1.0f;
		flg_calculate_Lagrange=true;
		flagv2chng=false;
			//for debug
			//	hw.PrintLine("def ordonnee vector fait avec nouveau point");
			//	for (int i=0; i<=max_inflexion_index;i++){		
			//		hw.PrintLine("%d\t%d\t%f\t%s", i,inflexionPtX.at(i),inflexionPtY.at(i),inflexionSign.at(i) ? "true" : "false");
			//	};
		
	}; 

// CV1 changes the space between inflexion points
	if (fabs(v1-oldv1)>=0.01){
		def_abscisse_vector (v1 ) ;
		oldv1=v1;  //AVOID TO recalulate all the time IF NO SIGNIFICANT CHANGE
		flg_calculate_Lagrange=true;
	};

//CV 2 is used in randomising the Y of the inflexion points
	if (fabs(v2-oldv2)>=0.01){
		flagv2chng = true;
		oldv2=v2; 
	};


	


// if changes : calculate the interpolated curve
// not sure the security for avoiding clicks by copying the  table is efficient...
	if (flg_calculate_Lagrange==true){
		for (int i = 0; i < MAX_POINT-1; i++) {
				waveshape_table_tmp.at(i) = Lagrange(i);
			}; 
		waveshape_table_tmp.at(0) = -1.0;
		waveshape_table_tmp.at(MAX_POINT-1) = 1.0;
		flg_calculate_Lagrange=false;
		waveshape_table = waveshape_table_tmp;
	};


}




void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	float tmp0,tmp1, sig0,sig1;
	hw.ProcessAllControls();
	for (size_t i = 0; i < size; i++){
		tmp0 = (osc0.Process() +1.0)* 512.0;
		int index_below = floorf(tmp0);
		int index_above = index_below+1;
		if (index_above>=1023){index_above=0;};
		float frac0 = tmp0 -index_below;

		// for now, linear interpolation 
		sig0 =frac0 * waveshape_table[index_below] + (1-frac0)* waveshape_table[index_above];
		// to try : cosinus interpolation with 2 points
		//sig0 = cosin_interpol(frac0,waveshape_table.at(index_above),waveshape_table.at(index_below));	

		OUT_L[i] = sig0;
		OUT_R[i] = sig0;		
	}
}

int main(void)
{
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	float sample_rate = hw.AudioSampleRate();
	button.Init(hw.B7);
	interrupteur.Init(hw.B8);
	//hw.StartLog(true);
	inflexionPtX.resize(nb_inflexion );
    inflexionPtY.resize(nb_inflexion);
	inflexionSign.resize(nb_inflexion);
	def_abscisse_vector (1.0) ;
	inflexionPtY.at(0)=-1.0;
// ugly but necessary to go around libdaisy bug !!
	inflexionPtY.at(1)=(float) hw.GetRandomValue()*0.99 /4294967296.0 -0.5 ;

	inflexionSign.at(0)=randomSign();
	inflexionPtY.at(2)=1.0;
	//hw.PrintLine("Init vaut %d\t%f\t%s", inflexionPtX.at(1),inflexionPtY.at(1),inflexionSign.at(1)? "true" : "false");
	hw.StartAudio(AudioCallback);
	osc0.Init(sample_rate);
	osc0.SetWaveform(osc0.WAVE_TRI);
	osc0.SetFreq(110);

    osc0.SetAmp(0.5);

	while(1) {
		controls();
	}
}
