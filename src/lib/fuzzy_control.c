/**********************************************
* Copyright(C) �quipe Funambule
*  Projet Plug & Stay
*-------------------------------------------
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
* ----------------------------------------
* Membres:
*  Dany Joly
*  Patrice Gagn�
*  Francois Pomerleau
*  Francois Boucher-Genesse
* ----------------------------------------
* Fichier : fuzzy_logic_ctl.c
* Responsable : FBG
* ----------------------------------------
* Historique : 
* DJ | 2004-04-20 | Creation du fichier
*********************************************/

#include <stdio.h>

#include "fuzzy_control.h"

#define FL_DEBUG 0

#define cal_weight(output, input, left_limit, middle_limit, right_limit, saturation)\
	if(saturation != 0)\
	{\
		if (input < left_limit || input > right_limit)\
			output = 0;\
		else\
		{\
			if (input < middle_limit)\
			{\
				output = (input - left_limit)/(middle_limit - left_limit);\
			}\
			else \
			{\
				output = 1+(input-middle_limit)/(middle_limit - right_limit);\
			}\
			if (saturation >= 0 && output > saturation)\
				output = saturation;\
		}\
	}else{\
	  output = 0;\
	}\

#define max_value(output, value1, value2, value3, value4, value5, value6, value7)\
	if (value1 > value2)\
		output = value1;\
	else \
		output = value2;\
	if (output < value3)\
		output = value3;\
	if (output < value4)\
		output = value4;\
	if (output < value5)\
		output = value5;\
	if (output < value6)\
		output = value6;\
	if (output < value7)\
		output = value7;

void fuzzy_acc_init ()
{
	
	tri_lr[0][0]=OUT_NL_L;
	tri_lr[0][1]=OUT_NL_M;
	tri_lr[0][2]=OUT_NL_R;

	tri_lr[1][0]=OUT_NS_L;
	tri_lr[1][1]=OUT_NS_M;
	tri_lr[1][2]=OUT_NS_R;

	tri_lr[2][0]=OUT_ZE_L;
	tri_lr[2][1]=OUT_ZE_M;
	tri_lr[2][2]=OUT_ZE_R;

	tri_lr[3][0]=OUT_PS_L;
	tri_lr[3][1]=OUT_PS_M;
	tri_lr[3][2]=OUT_PS_R;

	tri_lr[4][0]=OUT_PL_L;
	tri_lr[4][1]=OUT_PL_M;
	tri_lr[4][2]=OUT_PL_R;
}

//La sortie est en INT8, input1 en FLOAT32 et la valeur de vitesse est en INT16
FLOAT32 fuzzy_acc_ctl (const FLOAT32 input1, const FLOAT32 input2){

  //printf("fuzzy_in: %lf %lf\n",input1,input2);
	FLOAT32 SommeA;
	FLOAT32 SommeB;
	
	//#ifdef USE_FUZZY_CONTROL

	FLOAT32 weight1;				//Buffer for the weight of the decision fired by input1
	FLOAT32 weight2;				//Buffer for the weight of the decision fired by input2
	FLOAT32	x;						//Use to parse matrix and to calculate centroid
	FLOAT32	y;						//Use to parse matrix and to calculate centroid
	FLOAT32 output[MAX_DECISION];	//Array for the weight of all output value to be calcaculated
	FLOAT32 d_matrix [MAX_DECISION][MAX_DECISION];		//input1 is the first argument of the matrix and input2 the second
	FLOAT32 saturation_values[5];
	FLOAT32 limit_high=OUT_MAX;
	FLOAT32 limit_low=OUT_MIN;
	INT8 i;
	INT8 j;

	//input1 = acceleration
	//input2 = velocity error

	//Ensure that all decisions are reseted
	for (i=0; i<MAX_DECISION; i++){
		for (j=0; j<MAX_DECISION; j++){
			d_matrix[i][j]=0;
		}	
	}
	SommeA = 0;
	SommeB = 0;
	
	//Rules selection:

	//RULE 1:if input1 is NL and input2 is NL
	if (((input1 > IN1_NL_L) && (input1 < IN1_NL_R)) && ((input2 > IN2_NL_L) && (input2 < IN2_NL_R)))
	{
		cal_weight(weight1, input1, IN1_NL_L, IN1_NL_M, IN1_NL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NL_L, IN2_NL_M, IN2_NL_R, MAX_NEG);
		d_matrix [NL][NL]= min_value(weight1, weight2);
	}
	//RULE 2:if input1 is NL and input2 is NS
	if ((input1 > IN1_NL_L && input1 < IN1_NL_R) && (input2 > IN2_NS_L && input2 < IN2_NS_R))
	{
		cal_weight(weight1, input1, IN1_NL_L, IN1_NL_M, IN1_NL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NS_L, IN2_NS_M, IN2_NS_R, MAX_NEG);
		d_matrix [NL][NS]= min_value(weight1, weight2);	
	}
	//RULE 3:if input1 is NL and input2 is ZE
	if ((input1 > IN1_NL_L && input1 < IN1_NL_R) && (input2 > IN2_ZE_L && input2 < IN2_ZE_R))
	{
		cal_weight(weight1, input1, IN1_NL_L, IN1_NL_M, IN1_NL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_ZE_L, IN2_ZE_M, IN2_ZE_R, MAX_NEG);
		d_matrix [NL][ZE]= min_value(weight1, weight2);
	}
	//RULE 4:if input1 is NL and input2 is PS
	if ((input1 > IN1_NL_L && input1 < IN1_NL_R) && (input2 > IN2_PS_L && input2 < IN2_PS_R))
	{
		cal_weight(weight1, input1, IN1_NL_L, IN1_NL_M, IN1_NL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PS_L, IN2_PS_M, IN2_PS_R, MAX_NEG);
		d_matrix [NL][PS]= min_value(weight1, weight2);
	}
	//RULE 5:if input1 is NL and input2 is PL
	if ((input1 > IN1_NL_L && input1 < IN1_NL_R) && (input2 > IN2_PL_L && input2 < IN2_PL_R))
	{
		cal_weight(weight1, input1, IN1_NL_L, IN1_NL_M, IN1_NL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PL_L, IN2_PL_M, IN2_PL_R, MAX_NEG);
		d_matrix [NL][PL]= min_value(weight1, weight2);
	}
	//********************
	//RULE 6:if input1 is NS and input2 is NL
	if (input1 > IN1_NS_L && input1 < IN1_NS_R && input2 > IN2_NL_L && input2 < IN2_NL_R)
	{
		cal_weight(weight1, input1, IN1_NS_L, IN1_NS_M, IN1_NS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NL_L, IN2_NL_M, IN2_NL_R, MAX_NEG);
		d_matrix [NS][NL]= min_value(weight1, weight2);
	}
	//RULE 7:if input1 is NS and input2 is NS
	if ((input1 > IN1_NS_L && input1 < IN1_NS_R) && (input2 > IN2_NS_L && input2 < IN2_NS_R))
	{
		cal_weight(weight1, input1, IN1_NS_L, IN1_NS_M, IN1_NS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NS_L, IN2_NS_M, IN2_NS_R, MAX_NEG);
		d_matrix [NS][NS]= min_value(weight1, weight2);
	}
	//RULE 8:if input1 is NS and input2 is ZE
	if ((input1 > IN1_NS_L && input1 < IN1_NS_R) && (input2 > IN2_ZE_L && input2 < IN2_ZE_R))
	{
		cal_weight(weight1, input1, IN1_NS_L, IN1_NS_M, IN1_NS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_ZE_L, IN2_ZE_M, IN2_ZE_R, MAX_NEG);
		d_matrix [NS][ZE]= min_value(weight1, weight2);
	}
	//RULE 9:if input1 is NS and input2 is PS
	if ((input1 > IN1_NS_L && input1 < IN1_NS_R) && (input2 > IN2_PS_L && input2 < IN2_PS_R))
	{
		cal_weight(weight1, input1, IN1_NS_L, IN1_NS_M, IN1_NS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PS_L, IN2_PS_M, IN2_PS_R, MAX_NEG);
		d_matrix [NS][PS]= min_value(weight1, weight2);
	}
	//RULE 10:if input1 is NS and input2 is PL
	if ((input1 > IN1_NS_L && input1 < IN1_NS_R) && (input2 > IN2_PL_L && input2 < IN2_PL_R))
	{
		cal_weight(weight1, input1, IN1_NS_L, IN1_NS_M, IN1_NS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PL_L, IN2_PL_M, IN2_PL_R, MAX_NEG);
		d_matrix [NS][PL]= min_value(weight1, weight2);
	}
	//********************
	//RULE 11:if input1 is ZE and input2 is NL
	if (input1 > IN1_ZE_L && input1 < IN1_ZE_R && input2 > IN2_NL_L && input2 < IN2_NL_R)
	{
		cal_weight(weight1, input1, IN1_ZE_L, IN1_ZE_M, IN1_ZE_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NL_L, IN2_NL_M, IN2_NL_R, MAX_NEG);
		d_matrix [ZE][NL]= min_value(weight1, weight2);
	}
	//RULE 12:if input1 is ZE and input2 is NS
	if ((input1 > IN1_ZE_L && input1 < IN1_ZE_R) && (input2 > IN2_NS_L && input2 < IN2_NS_R))
	{
		cal_weight(weight1, input1, IN1_ZE_L, IN1_ZE_M, IN1_ZE_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NS_L, IN2_NS_M, IN2_NS_R, MAX_NEG);
		d_matrix [ZE][NS]= min_value(weight1, weight2);
	}
	//RULE 13:if input1 is ZE and input2 is ZE
	if ((input1 > IN1_ZE_L && input1 < IN1_ZE_R) && (input2 > IN2_ZE_L && input2 < IN2_ZE_R))
	{
		cal_weight(weight1, input1, IN1_ZE_L, IN1_ZE_M, IN1_ZE_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_ZE_L, IN2_ZE_M, IN2_ZE_R, MAX_NEG);
		d_matrix [ZE][ZE]= min_value(weight1, weight2);
	}
	//RULE 14:if input1 is ZE and input2 is PS
	if ((input1 > IN1_ZE_L && input1 < IN1_ZE_R) && (input2 > IN2_PS_L && input2 < IN2_PS_R))
	{
		cal_weight(weight1, input1, IN1_ZE_L, IN1_ZE_M, IN1_ZE_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PS_L, IN2_PS_M, IN2_PS_R, MAX_NEG);
		d_matrix [ZE][PS]= min_value(weight1, weight2);
	}
	//RULE 15:if input1 is ZE and input2 is PL
	if ((input1 > IN1_ZE_L && input1 < IN1_ZE_R) && (input2 > IN2_PL_L && input2 < IN2_PL_R))
	{
		cal_weight(weight1, input1, IN1_ZE_L, IN1_ZE_M, IN1_ZE_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PL_L, IN2_PL_M, IN2_PL_R, MAX_NEG);
		d_matrix [ZE][PL]= min_value(weight1, weight2);
	}
	//********************
	//RULE 16:if input1 is PS and input2 is NL
	if (input1 > IN1_PS_L && input1 < IN1_PS_R && input2 > IN2_NL_L && input2 < IN2_NL_R)
	{
		cal_weight(weight1, input1, IN1_PS_L, IN1_PS_M, IN1_PS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NL_L, IN2_NL_M, IN2_NL_R, MAX_NEG);
		d_matrix [PS][NL]= min_value(weight1, weight2);
	}
	//RULE 17:if input1 is PS and input2 is NS
	if ((input1 > IN1_PS_L && input1 < IN1_PS_R) && (input2 > IN2_NS_L && input2 < IN2_NS_R))
	{
		cal_weight(weight1, input1, IN1_PS_L, IN1_PS_M, IN1_PS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NS_L, IN2_NS_M, IN2_NS_R, MAX_NEG);
		d_matrix [PS][NS]= min_value(weight1, weight2);
	}
	//RULE 18:if input1 is PS and input2 is ZE
	if ((input1 > IN1_PS_L && input1 < IN1_PS_R) && (input2 > IN2_ZE_L && input2 < IN2_ZE_R))
	{
		cal_weight(weight1, input1, IN1_PS_L, IN1_PS_M, IN1_PS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_ZE_L, IN2_ZE_M, IN2_ZE_R, MAX_NEG);
		d_matrix [PS][ZE]= min_value(weight1, weight2);
	}
	//RULE 19:if input1 is PS and input2 is PS
	if ((input1 > IN1_PS_L && input1 < IN1_PS_R) && (input2 > IN2_PS_L && input2 < IN2_PS_R))
	{
		cal_weight(weight1, input1, IN1_PS_L, IN1_PS_M, IN1_PS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PS_L, IN2_PS_M, IN2_PS_R, MAX_NEG);
		d_matrix [PS][PS]= min_value(weight1, weight2);
	}
	//RULE 20:if input1 is PS and input2 is PL
	if ((input1 > IN1_PS_L && input1 < IN1_PS_R) && (input2 > IN2_PL_L && input2 < IN2_PL_R))
	{
		cal_weight(weight1, input1, IN1_PS_L, IN1_PS_M, IN1_PS_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PL_L, IN2_PL_M, IN2_PL_R, MAX_NEG);
		d_matrix [PS][PL]= min_value(weight1, weight2);
	}
	//********************
	//RULE 21:if input1 is PL and input2 is NL
	if (input1 > IN1_PL_L && input1 < IN1_PL_R && input2 > IN2_NL_L && input2 < IN2_NL_R)
	{
		cal_weight(weight1, input1, IN1_PL_L, IN1_PL_M, IN1_PL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NL_L, IN2_NL_M, IN2_NL_R, MAX_NEG);
		d_matrix [PL][NL]= min_value(weight1, weight2);
	}
	//RULE 22:if input1 is PL and input2 is NS
	if ((input1 > IN1_PL_L && input1 < IN1_PL_R) && (input2 > IN2_NS_L && input2 < IN2_NS_R))
	{
		cal_weight(weight1, input1, IN1_PL_L, IN1_PL_M, IN1_PL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_NS_L, IN2_NS_M, IN2_NS_R, MAX_NEG);
		d_matrix [PL][NS]= min_value(weight1, weight2);
	}
	//RULE 23:if input1 is PL and input2 is ZE
	if ((input1 > IN1_PL_L && input1 < IN1_PL_R) && (input2 > IN2_ZE_L && input2 < IN2_ZE_R))
	{
		cal_weight(weight1, input1, IN1_PL_L, IN1_PL_M, IN1_PL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_ZE_L, IN2_ZE_M, IN2_ZE_R, MAX_NEG);
		d_matrix [PL][ZE]= min_value(weight1, weight2);
	}
	//RULE 24:if input1 is PL and input2 is PS
	if ((input1 > IN1_PL_L && input1 < IN1_PL_R) && (input2 > IN2_PS_L && input2 < IN2_PS_R))
	{
		cal_weight(weight1, input1, IN1_PL_L, IN1_PL_M, IN1_PL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PS_L, IN2_PS_M, IN2_PS_R, MAX_NEG);
		d_matrix [PL][PS]= min_value(weight1, weight2);
	}
	//RULE 25:if input1 is PL and input2 is PL
	if ((input1 > IN1_PL_L && input1 < IN1_PL_R) && (input2 > IN2_PL_L && input2 < IN2_PL_R))
	{
		cal_weight(weight1, input1, IN1_PL_L, IN1_PL_M, IN1_PL_R, MAX_NEG);
		cal_weight(weight2, input2, IN2_PL_L, IN2_PL_M, IN2_PL_R, MAX_NEG);
		d_matrix [PL][PL]= min_value(weight1, weight2);
	}

	// Calcul max value for output NL
	max_value(saturation_values[0], d_matrix[NS][NL], d_matrix[PL][NL], d_matrix[PL][NS], d_matrix[PL][ZE], d_matrix[PL][PS], d_matrix[PL][PL], MAX_NEG);
	// Calcul max value for output NS
	max_value(saturation_values[1], d_matrix[NS][NS], d_matrix[ZE][NL], d_matrix[ZE][NS], d_matrix[PS][NL], d_matrix[PS][NS], d_matrix[PS][ZE], MAX_NEG);
	// Calcul max value for output ZE
	max_value(saturation_values[2], d_matrix[ZE][ZE], MAX_NEG, MAX_NEG, MAX_NEG, MAX_NEG, MAX_NEG, MAX_NEG);
	// Calcul max value for output PS
	max_value(saturation_values[3], d_matrix[NS][ZE], d_matrix[NS][PS], d_matrix[NS][PL], d_matrix[ZE][PS], d_matrix[ZE][PL], d_matrix[PS][PS], MAX_NEG);
	// Calcul max value for output PL
	max_value(saturation_values[4], d_matrix[NL][NL], d_matrix[NL][NS], d_matrix[NL][ZE], d_matrix[NL][PS], d_matrix[NL][PL], d_matrix[PS][PL],MAX_NEG);

// SATURATION NOT USED
/*	for (i=0;i<5;++i) 
	{
		if (saturation_values[i]!=0) 
		{
			if (limit_low>tri_lr[i][0])
				limit_low=tri_lr[i][0];		
		}
	}

	for (i=4;i>=0;--i) 
	{
		if (saturation_values[i]!=0) 
		{
			if (limit_high<tri_lr[i][2])
				limit_high=tri_lr[i][2];
		}
	}
*/

	for (x=OUT_MIN; x<=OUT_MAX; x += (OUT_MAX-OUT_MIN)/PRECISION)
	{
		//Link the decision matrix with the weight of each rules fired
		cal_weight(output[NL], (FLOAT32)x, tri_lr[0][0], tri_lr[0][1], tri_lr[0][2], saturation_values[0]);
		cal_weight(output[NS], (FLOAT32)x, tri_lr[1][0], tri_lr[1][1], tri_lr[1][2], saturation_values[1]);
		cal_weight(output[ZE], (FLOAT32)x, tri_lr[2][0], tri_lr[2][1], tri_lr[2][2], saturation_values[2]);
		cal_weight(output[PS], (FLOAT32)x, tri_lr[3][0], tri_lr[3][1], tri_lr[3][2], saturation_values[3]);
		cal_weight(output[PL], (FLOAT32)x, tri_lr[4][0], tri_lr[4][1], tri_lr[4][2], saturation_values[4]);

		//Calculate the centroide for the deffuzziliation
		max_value (y, output[NL], output[NS], output[ZE], output[PS], output[PL], MAX_NEG, MAX_NEG);


		SommeA += y*x;
		SommeB += y;
	}
	
	//#endif // USE_FUZZY_CONTROL

if(FL_DEBUG){
    printf("%lf %lf %lf %lf %lf\n",d_matrix[NL][NL],d_matrix[NL][NS],d_matrix[NL][ZE],d_matrix[NL][PS],d_matrix[NL][PL]);
	  printf("%lf %lf %lf %lf %lf\n",d_matrix[NS][NL],d_matrix[NS][NS],d_matrix[NS][ZE],d_matrix[NS][PS],d_matrix[NS][PL]);
	  printf("%lf %lf %lf %lf %lf\n",d_matrix[ZE][NL],d_matrix[ZE][NS],d_matrix[ZE][ZE],d_matrix[ZE][PS],d_matrix[ZE][PL]);
	  printf("%lf %lf %lf %lf %lf\n",d_matrix[PS][NL],d_matrix[PS][NS],d_matrix[PS][ZE],d_matrix[PS][PS],d_matrix[PS][PL]);
	  printf("%lf %lf %lf %lf %lf\n\n",d_matrix[PL][NL],d_matrix[PL][NS],d_matrix[PL][ZE],d_matrix[PL][PS],d_matrix[PL][PL]);
	
	printf("out : \nNL NS ZE PS PL: %lf %lf %lf %lf %lf\n",saturation_values[0],saturation_values[1],saturation_values[2],saturation_values[3],saturation_values[4]);
}
	return SommeA/SommeB;
} 

FLOAT32 min_value (const FLOAT32 value1, const FLOAT32 value2){
	if (value1 < value2)
		return value1;
	else 
		return value2;
}
