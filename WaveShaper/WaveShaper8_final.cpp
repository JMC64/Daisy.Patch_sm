#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "dev/oled_ssd130x.h"
#include "vector"
#include <string>
#include "fatfs.h"
#include <sstream>


using namespace daisy;
using namespace patch_sm;
using namespace daisysp;
using namespace std;

DaisyPatchSM hw;
Encoder encoder;
Oscillator osc0,osc1,osc2;
bool osc_mode_IN_L =false;
float myCV1=0.0f,myCV2=0.0f,myCV3=0.0f,myCV4=0.0f,myCV5=0.0f,myCV6=0.0f ;

using MyDisplay = OledDisplay<SSD130x4WireSoftSpi128x64Driver>;
MyDisplay display;

vector <string> menu {"EDT", "OSC", "FILE","RST"};
vector <string> menu_osc {"SINE", "TRI", "SAW", "IN_L","EXIT"};
vector <string> menu_file {"LOAD", "SAVE","EXIT"};
vector <string> menu_reset {"RESET","EXIT"};


vector <int> inflexionPtX_display          { 1   ,   7    ,  15   ,23      ,31    ,39      ,47     ,55      ,63   ,71     ,79    ,87     ,95   ,103   ,111    ,119    ,126};  //cheat on first and last to see them
vector <int> inflexionPtY_display          {62,59,56,53,50,47,44,41,38,35,32,29,26,23,20,17,14,11};   //cheat on first and last to see them
vector <int> Old_inflexionPtY_display      {62,59,56,53,50,47,44,41,38,35,32,29,26,23,20,17,14,11}; 
// keep the CV
vector <float> Old_inflexionPtY           { 0.0f ,  0.0625f,0.125f,0.1875f ,0.25f , 0.3125f, 0.375f, 0.4375f, 0.5f,0.5625f,0.625f,0.6875f,0.75f,0.8125f,0.875f,0.9375f, 1.0f}; 
vector <int> inflexionPtX                 {     0,      64,    128,     192,   256,     320,    384,     448,  512,    576,   640,    704,   768,   832,   896,    960,  1024}; 
// keep inflexion point for waveshape
vector <float> inflexionPtY               {-1.0f , -0.875f, -0.75f, -0.625f, -0.5f, -0.375f, -0.25f, -0.125f, 0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, 1.0f };  


int MAX_POINT = 1025;
vector <float> waveshape_table(MAX_POINT); //[num_points];
vector <float> waveshape_table_display(128); //[num_points];
float min_y_val=0.0f,max_y_val=0.0f;  //used for normalisation
vector<string>  FilenamesVector {"EXIT"};

FIL    DSY_SDRAM_BSS         	myfile; /**< Can't be made on the stack (DTCMRAM) */
FatFSInterface   DSY_SDRAM_BSS  	fsi;
SdmmcHandler     sdcard;
FILINFO 	 DSY_SDRAM_BSS   	fileInfo;
FRESULT 	 DSY_SDRAM_BSS   	fr;
DIR 		 DSY_SDRAM_BSS   	dir;                    // Directory


void manage_encoder(int32_t inc,int32_t &x,int m  ){
	x+=inc;
	if (x<0){
		x =m-1;
		}
	else{
		x=x % m;
		};
};

// Function to perform cubic spline interpolation
float CubicSplineInterpolation(float x) {
    int n = inflexionPtX.size();
    if (n != inflexionPtY.size() || n < 2) {
      //  std::cerr << "Error: Invalid input data." << std::endl;
        return 0.0f;
    }

    // Find the interval [x_k, x_{k+1}] containing x
    int k = 0;
    while (k < n - 1 && x > inflexionPtX[k + 1]) {
        ++k;
    }

    // Compute the parameters for the cubic spline
    float h = inflexionPtX[k + 1] - inflexionPtX[k];
    float t = (x - inflexionPtX[k]) / h;
    float t2 = t * t;
    float t3 = t2 * t;
    float y_k = inflexionPtY[k];
    float y_k1 = inflexionPtY[k + 1];
    float m_k = (k == 0) ? 0.0f : (y_k1 - inflexionPtY[k - 1]) / (inflexionPtX[k + 1] - inflexionPtX[k - 1]);
    float m_k1 = (k == n - 2) ? 0.0f : (inflexionPtY[k + 2] - y_k) / (inflexionPtX[k + 2] - inflexionPtX[k]);

    // Cubic spline interpolation
    float a = 2 * t3 - 3 * t2 + 1;
    float b = -2 * t3 + 3 * t2;
    float c = t3 - 2 * t2 + t;
    float d = t3 - t2;
    float interpolatedY = a * y_k + b * y_k1 + c * h * m_k + d * h * m_k1;

    // Update max and min values
    if (interpolatedY >  max_y_val ) {
       max_y_val  = interpolatedY;
    }
    if (interpolatedY < min_y_val) {
      	min_y_val = interpolatedY;
    }

    return interpolatedY;
}


float normalize(float x, float minVal, float maxVal) {
    return 2.0f * (x - minVal) / (maxVal - minVal) - 1.0f;
}

void calculate_wave_shape(){
	for (int i =1 ; i<=1023; i++){
		waveshape_table.at(i)=CubicSplineInterpolation(i);
	};
	for (int i =1 ; i<=1023; i++){
		waveshape_table.at(i) = normalize( waveshape_table.at(i),min_y_val,max_y_val);
	};
};

void show_calculated_spleen(){
	int y=0;
 for (int i = 0 ; i <1024; i+=8){
	y = (int)(waveshape_table.at(i)*-26.5f+36.5f);
	display.DrawPixel(i/8,y,true);
 };

}


void load_file(string filename){
	char line[30];
	int i=0;
	float foo;
	//hw.PrintLine("File to open : %s",filename);

	if(f_mount(&fsi.GetSDFileSystem(), "/", 0) == FR_OK)  {
		f_open(&myfile,filename.c_str(),FA_READ);
		
	while (f_gets(line,36,&myfile)) {
			foo=stof(line);
			inflexionPtY.at(i)=foo;
			inflexionPtY_display.at(i)=(int)(-23.5f*(foo+1.0f)+63.5f);
			Old_inflexionPtY.at(i)=(foo+1.0)*0.5f;
			//hw.PrintLine(" Lu dans fic Display : %d Value :%f  CV:%f", inflexionPtY_display.at(i),  inflexionPtY.at(i), Old_inflexionPtY.at(i)  )   ;
			i++;
		}
	f_close(&myfile);
	};
	f_mount(0, "", 0); //umount drive
};

void load_file_names(){
string test_string1;
	if (FilenamesVector.size()!=0){
		FilenamesVector.clear();
		FilenamesVector.push_back("EXIT");
		};
// hw.PrintLine( "load_file_names");
  // Mount SD Card
 	if (f_mount(&fsi.GetSDFileSystem(), "/", 1) != FR_OK) {
  //	  hw.PrintLine( "mount error", fileInfo,"\n");
 	 }
	else  { 
	//	hw.PrintLine( "sd mounted");
		f_opendir(&dir, "/");   // Open Root
		do{
			f_readdir(&dir, &fileInfo);
			if (fileInfo.fname[0] != 0){
				test_string1= std::string(fileInfo.fname);
				if (test_string1.substr(test_string1.size() - 4)==".WSP"){
					FilenamesVector.push_back(std::string(fileInfo.fname));
				};
			};
		} while(fileInfo.fname[0] != 0);
		f_closedir(&dir);
	};
	for (int i =0; i < FilenamesVector.size(); i++){
		//hw.PrintLine("file %s",FilenamesVector[i].c_str()) ;
	};

};




int  display_load (int inc) {
static int32_t pos;
int pos1=0;
	display.DrawRect(0,16,127,63,false,true);																//redraw the frame
	display.DrawRect(0,16,127,63,false,false);
	display.SetCursor(8,17);
	display.WriteChar('*',Font_7x10,true);
	manage_encoder(inc,pos ,FilenamesVector.size()) ;
	//hw.PrintLine("pos in list %d",pos);
	for (int i =0 ; i<5;i++){
		display.SetCursor(14,17+i*8);
		pos1=(pos+i)%FilenamesVector.size();
		display.WriteString(FilenamesVector[pos1].c_str(),Font_6x8,true);
	};
	return pos;	
};


void save_file(){
	const char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
	char myfilename [13];
	uint32_t  rand;
	float x;
	int xi;
	UINT   bw  = 0;
	string tmpstr ;
  	myfilename[0]='F';
	for (int i =1 ;i <8; i++){			
			rand= hw.GetRandomValue() 	;
			x=(float)( rand)*61.0f/ 4294967295.0f+0.5f;
			xi= (int) (x+0.5f);
			myfilename [i]= alphanum[xi];	
		//	hw.PrintLine("%f %d %d %f %c ",x, xi, strlen(alphanum), (float) rand ,alphanum[xi]);
	};
	myfilename[8]='.';
	myfilename[9]='W';
	myfilename[10]='S';
	myfilename[11]='P'; 
	myfilename[12]='\0';
	//hw.PrintLine("filename %s", myfilename);
	
	
	
	display.DrawRect(0,10,127,63,false, true);
	display.DrawRect(0,10,127,63,false, false);
	display.SetCursor(0,30);
	display.WriteString(tmpstr.c_str(),Font_6x8,true);
	tmpstr = "Saving in: ";
	display.SetCursor(25,40);
	tmpstr = string(myfilename);
	display.WriteString(tmpstr.c_str(),Font_6x8,true);
	 if(f_mount(&fsi.GetSDFileSystem(), "/", 0) == FR_OK)
    {
		if(f_open(&myfile, myfilename, (FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK) {
		//	hw.PrintLine("write 1");
			for (int i = 0 ; i <inflexionPtY.size(); i++ ){
				string test_string1= to_string (inflexionPtY.at(i) ) +"\n";
				f_write(&myfile,test_string1.c_str(),strlen(test_string1.c_str()) , &bw);
			//	hw.PrintLine("Bytes %d",bw);
			//	hw.PrintLine("%s",test_string1.c_str());
			};	
		}
		else{
		//	hw.PrintLine("Error opening file for writing %d " , f_open(&myfile, myfilename, (FA_CREATE_ALWAYS | FA_WRITE)));
			};
		f_close(&myfile);
	};
	f_mount(0, "", 0);  // umount the drive
	
	display.SetCursor(80,50);
	display.WriteString("Done!",Font_6x8,true);
};


void reset(){
	int arr[17]=  { 1   ,   7    ,  15   ,23      ,31    ,39      ,47     ,55      ,63   ,71     ,79    ,87     ,95   ,103   ,111    ,119    ,126};  //cheat on first and last to see them
	int arr1[17] =    {62,59,56,53,50,47,44,41,38,35,32,29,26, 23, 20, 17, 15}; 
	float arr3[17] =     {-1.0f , -0.875f, -0.75f, -0.625f, -0.5f, -0.375f, -0.25f, -0.125f, 0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, 1.0f };  
	float arr4[17] =  { 0.0f ,  0.0625f,0.125f,0.1875f ,0.25f , 0.3125f, 0.375f, 0.4375f, 0.5f,0.5625f,0.625f,0.6875f,0.75f,0.8125f,0.875f,0.9375f, 1.0f}; 
	int arr5[17] =  {     0,      64,    128,     192,   256,     320,    384,     448,  512,    576,   640,    704,   768,   832,   896,    960,  1024}; 

	for (int i=0; i <17;i++){
		inflexionPtX_display.at(i)=arr[i];
		inflexionPtY_display.at(i)=arr1[i];
		Old_inflexionPtY_display.at(i)=arr1[i];
		inflexionPtX.at(i)=arr5[i];
		inflexionPtY.at(i)  =arr3[i];
		Old_inflexionPtY.at(i) =arr4[i];
	};
};
void draw_cross(int x_center, int y_center, int width, bool flg){
	int pix_left, pix_right, pix_top, pix_bottom;
	pix_left   = max(x_center- width, 0);
	pix_right  = min(x_center+ width, 127);
	pix_top    = max(y_center-width,0);
	pix_bottom = min(y_center+width,63);
	display.DrawLine(pix_left,y_center,pix_right, y_center ,flg);
	display.DrawLine(x_center,pix_top,x_center ,pix_bottom, flg);
};

void display_edit_screen(int actual_point){
	display.DrawRect(0,0,127,9,false,true);	
	display.DrawRect(0,0,127,9,false,false);	
	display.SetCursor(50,0);
	display.WriteString("EXIT",Font_6x8,true);
	display.DrawRect(0,10,127,63,false,true);																//redraw the frame
	display.DrawRect(0,10,127,63,false,false);	 
	if  (actual_point<=16){
		for (int i=0;i <=16 ;i++){
			draw_cross(inflexionPtX_display.at(i),inflexionPtY_display.at(i),2,true);
		};
		draw_cross(inflexionPtX_display.at(actual_point),Old_inflexionPtY_display.at(actual_point),2,false); 	// erase  cross at previous position
		draw_cross(inflexionPtX_display.at(actual_point),inflexionPtY_display.at(actual_point),2,true);      	// new cross at new position
		if (actual_point>1){																					// erase circle at previous selected point
			display.DrawCircle(((actual_point-1)*8) -1, 37,3,false);									
		} 
		else{
			display.DrawCircle((15*8) -1, 37,3,false);
		};

		display.DrawCircle((actual_point*8) -1, 37,3,true);														// draw circle at new position
		Old_inflexionPtY_display.at(actual_point)=inflexionPtY_display.at(actual_point);	
	}					
	else{
		display.SetCursor(50, 0);
		display.WriteString("EXIT",Font_6x8,false);
		for (int i=0;i <=16 ;i++){
			draw_cross(inflexionPtX_display.at(i),inflexionPtY_display.at(i),2,true);
		};	
	};
	display.DrawRect(0,10,127,63,true,false);																//redraw the frame
	display.DrawLine(0,37,127,37,true); 																	//redraw the axis for y = 0


};

void display_submenu(int menu_pos,vector  <string> m){
	display.DrawRect(0,10,127,63,false, true);
	display.DrawRect(0,10,127,63,false, false);
	for (int i =0; i < m.size();i++){
		if (i ==  menu_pos){
			display.SetCursor(10, i * 9 +16);
			display.WriteString(m[i].c_str(),Font_6x8,false);
		} 
		else{
			display.SetCursor(10, i * 9 +16);
			display.WriteString(m[i].c_str(),Font_6x8,true);
		};		
	};

};

void display_menu(int menu_pos){
	int pixel_sum=0;;
	display.DrawRect(0,0,127,9,false,true);	
	display.DrawRect(0,0,127,9,false,false);
	for (int i =0; i < menu.size();i++){
		display.SetCursor(pixel_sum,0);
		if (i== menu_pos){
		//	display.DrawRect( pixel_sum,0, pixel_sum + 6*menu[i].length(),6,true,true);
			display.WriteString(menu[i].c_str(),Font_6x8,false);
			}
		else{
			display.WriteString(menu[i].c_str(),Font_6x8,true);
		};
		pixel_sum+= (menu[i].length()+1)*6;
		
	};
 }


void control_menu(){
	static int32_t count=0,increment , count_submenu_osc=0, count_submenu_file=0,count_submenu_rst=0,actual_point=0;
	bool click=false;
	char tmpstr[15];
	static int menu_lvl = 0;
	encoder.Debounce();
	increment=encoder.Increment();
	 
	myCV1 = fclamp(hw.GetAdcValue(CV_1), 0.0f, 1.0f);
	myCV2 = fclamp(hw.GetAdcValue(CV_2), 0.1f, 0.99f);
	myCV3 = fclamp(hw.GetAdcValue(CV_3), 0.0f, 1.0f);
	myCV4 = fclamp(hw.GetAdcValue(CV_4), 0.0f, 1.0f);
	myCV5 = fclamp(hw.GetAdcValue(CV_5), 0.0f, 1.0f);
	myCV5 =fmap(myCV5*myCV2,0.1f,1.0f);
	
	myCV6= hw.GetAdcValue(CV_6);
	float pitch = fmap(myCV6 ,0.f, 60.f);
	float note0 = fclamp(pitch+24.0f, 0.f, 127.f);
	float note1=  note0 + (0.05 *  note0  * myCV1);
	float note2=  note0 - (0.05 *  note0  * myCV1);
    float freq0    = mtof(note0);
	float freq1    = mtof(note1);
	float freq2    = mtof(note2);
	osc0.SetFreq(freq0);
	osc1.SetFreq(freq1);
	osc2.SetFreq(freq2);
	float amp = fclamp(myCV5 *myCV2+myCV2,0.1f,0.99f);
	osc0.SetAmp(amp );
	osc1.SetAmp(amp );
	osc2.SetAmp(amp );

	if(encoder.FallingEdge() ){click = true;};

	switch (menu_lvl){
		case 0 : 
			manage_encoder(increment,count,menu.size());
			display_menu(count);
			if (click){
				menu_lvl++;
				click=false;
			};
		break;
		case 1 :
			switch(count){
				case 0 : // EDT menu
					manage_encoder(increment,actual_point,18);
					//if (++actual_point>15){actual_point=1;};
					display_edit_screen(actual_point);
					show_calculated_spleen();
					if (click){
						if (actual_point==17){ //select menu exit
							menu_lvl--; 
							click=false;
							}
						else{
						menu_lvl++;
						click=false;
						};
					};
			
				break;
				case 1 : // OSC menu
					manage_encoder(increment,count_submenu_osc,menu_osc.size());
					display_submenu(count_submenu_osc,menu_osc);
					if (click){
						switch (count_submenu_osc){
							case 0 :
							osc0.SetWaveform(osc0.WAVE_SIN);
							osc1.SetWaveform(osc1.WAVE_SIN);
							osc2.SetWaveform(osc2.WAVE_SIN);
							osc_mode_IN_L =false;
							break;
							case 1 :
							osc0.SetWaveform(osc0.WAVE_TRI);
							osc1.SetWaveform(osc1.WAVE_TRI);
							osc2.SetWaveform(osc2.WAVE_TRI);
							osc_mode_IN_L =false;
							break;
							case 2 :
							osc0.SetWaveform(osc0.WAVE_SAW);
							osc1.SetWaveform(osc1.WAVE_SAW);
							osc2.SetWaveform(osc2.WAVE_SAW);
							osc_mode_IN_L =false;
							break;
							case 3 :
							osc_mode_IN_L =false;
							break;
							case 4 :
								// do nothing
							break;
						};
						menu_lvl--;
						display.DrawRect(0,10,127,63,false, true);
						display.DrawRect(0,10,127,63,false, false);
						click=false;
					}
				break;
				case 2 : // FILE menu
					manage_encoder(increment,count_submenu_file,menu_file.size());
					display_submenu(count_submenu_file,menu_file);
					if ( click && count_submenu_file== (menu_file.size()-1)){
						menu_lvl--;
						display.DrawRect(0,10,127,63,false, true);
						display.DrawRect(0,10,127,63,false, false);
						click=false;};
					if ( click && count_submenu_file== 0){	
							load_file_names();
							menu_lvl++;
							click=false;
						
					};
					if ( click && count_submenu_file== 1){	
						//hw.PrintLine("Enter savefile");
							save_file();
							menu_lvl--;
							click=false;
						
					};
				break;
				case 3 : // RST menu
					manage_encoder(increment,count_submenu_rst,menu_reset.size());
				 	display_submenu(count_submenu_rst,menu_reset);
					if (click && count_submenu_rst== (menu_reset.size()-1)){
						menu_lvl--;
						display.DrawRect(0,10,127,63,false, true);
						display.DrawRect(0,10,127,63,false, false);
						click=false;
					}
					else{
						if (click){
							reset();
							calculate_wave_shape();
							click=false;
						};
					};
			
				break;
			};
		break;
		case 2:  // modifying the points on edit menu
			switch (count){
				case 0://in edit mode
					inflexionPtY.at(actual_point)  += increment/100.0f;
					if (inflexionPtY.at(actual_point) >1.0f){ inflexionPtY.at(actual_point) =-1.0f;};
					if (inflexionPtY.at(actual_point) < -1.0f){ inflexionPtY.at(actual_point) =1.0f;};
					display_edit_screen(actual_point);
					Old_inflexionPtY_display.at(actual_point) =inflexionPtY_display.at(actual_point);

					if (click){  // && encoder.TimeHeldMs()> 500.0f
								calculate_wave_shape();
								menu_lvl--;
								click=false;
					};
					inflexionPtY_display.at(actual_point) = (int)(inflexionPtY.at(actual_point)*-26.5f+36.5f);
					//display.SetCursor(10, 20);
				//	sprintf(tmpstr,"%.2f %d",inflexionPtY.at(actual_point),inflexionPtY_display.at(actual_point) );
					//display.WriteString(tmpstr,Font_6x8,true);
				break;
				case 1 ://in osc mode nothing to do
				break;
				case 2 : //in file mode
				 	if (count_submenu_file== 0){ // load file 
						int choice = display_load(increment);
						if (click){
							if ( choice == 0 ){
								menu_lvl--;
								click=false;
							}
							else
							{
								// load the file 
								load_file(FilenamesVector[choice]);
								calculate_wave_shape();
								menu_lvl--;
								click=false;
							};
						};
					};
				break;
			};

		break;
			
	}

};

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{float tmp0,tmp1, sig0,sig1;
int index_below, index_above;

	hw.ProcessAllControls();
if (osc_mode_IN_L ){
		for (size_t i = 0; i < size; i++)
		{
			tmp0 =( IN_L[i]* myCV2 +1.0f)* 512.0f;
			 index_below = floorf(tmp0);
			 index_above = index_below+1;
			if (index_above>=1023){index_above=0;};
			float frac0 = tmp0 -index_below;
			// for now, linear interpolation 
			sig0 =frac0 * waveshape_table[index_below] + (1.f-frac0)* waveshape_table[index_above];
			OUT_L[i] = sig0;
			OUT_R[i] = sig0;
		}
	}
	else{
		for (size_t i = 0; i < size; i++)
		{
			tmp0 = (osc0.Process() +1.0f)* 512.0f;
			float tmp1 = (osc1.Process() +1.0f)* 512.0f;
			float tmp2 = (osc2.Process() +1.0f)* 512.0f;
			 index_below = floorf(tmp0);
			 index_above = index_below+1;
			if (index_above>=1023){index_above=0;};
			float frac0 = tmp0 -index_below;
			// for now, linear interpolation 
			sig0 =frac0 * waveshape_table[index_below] + (1.f-frac0)* waveshape_table[index_above];

			 index_below = floorf(tmp1);
			 index_above = index_below+1;
			if (index_above>=1023){index_above=0;};
			float frac1 = tmp1 -index_below;
			// for now, linear interpolation 
			sig0 +=frac1 * waveshape_table[index_below] + (1.f-frac1)* waveshape_table[index_above];

			 index_below = floorf(tmp2);
			 index_above = index_below+1;
			if (index_above>=1023){index_above=0;};
			float frac2 = tmp2 -index_below;
			// for now, linear interpolation 
			sig0 +=frac2 * waveshape_table[index_below] + (1.f-frac2)* waveshape_table[index_above];
			OUT_L[i] = sig0*0.3333f;
			OUT_R[i] = sig0*0.3333f;
			
		};
	};



}

int main(void)
{  int k=0,j=0;
	//char tmpstr[15];

	hw.Init();
	//hw.StartLog(false);
	encoder.Init(hw.A8, hw.A9, hw.D8); // These pins need to be defined

MyDisplay::Config display_config;
    display_config.driver_config.transport_config.pin_config.sclk = DaisyPatchSM::D10;
    display_config.driver_config.transport_config.pin_config.sclk_delay = 0;
    display_config.driver_config.transport_config.pin_config.mosi = DaisyPatchSM::D9;
    display_config.driver_config.transport_config.pin_config.dc = DaisyPatchSM::A2;//DaisyPatchSM::D2;
    display_config.driver_config.transport_config.pin_config.reset = DaisyPatchSM::A3;//DaisyPatchSM::D3;
    display.Init(display_config);
	
 	SdmmcHandler::Config sd_config;
    SdmmcHandler         sdcard;
    sd_config.Defaults();
    sdcard.Init(sd_config);
    fsi.Init(FatFSInterface::Config::MEDIA_SD);

hw.StartLog(false);
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAudio(AudioCallback);

	float sample_rate   = hw.AudioSampleRate();
	osc0.Init(sample_rate);
	osc0.SetWaveform(osc0.WAVE_SIN);
	osc0.SetFreq(110.0f);
    osc0.SetAmp(0.5f);
	osc1.Init(sample_rate);
	osc1.SetWaveform(osc1.WAVE_SIN);
	osc1.SetFreq(109.0f);
    osc1.SetAmp(0.5f);
	osc2.Init(sample_rate);
	osc2.SetWaveform(osc2.WAVE_SIN);
	osc2.SetFreq(111.0f);
    osc2.SetAmp(0.5f);


	display.Fill(false);
	while(1) {
		
		control_menu();
		display.Update();

	};
}
