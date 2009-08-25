/**********************************************
* Copyright(C) Équipe Funambule  
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
*  Patrice Gagné
*  Francois Pomerleau
*  Francois Boucher-Genesse
* ----------------------------------------
* Fichier : fuzzy_membership.h
* Responsable : FP
* ----------------------------------------
* Description :
* - Contain definition for the fuzzy 
*   logique membership function.
* - This controler has been developed for
*   a specific application: satellite control
* ----------------------------------------
* Utilisation :
* - The controler used must have 2 inputs and one 
*   output.
* - You must adjust the value for your specific
*   controler.
* - Limites are normalize (between -1 and 1) with
*   the maximum value 
* - Define all the membership limits in this file
* ----------------------------------------
*********************************************/

#ifndef SMART_FUZZY_MEMBERSHIP_H
#define SMART_FUZZY_MEMBERSHIP_H

/*
* Definitions for the decision matrix
* Do not change
*/

#define NL 0		//Negative Large
#define NS 1		//Negative Small
#define ZE 2		//Zero
#define PS 3		//Positive Small
#define PL 4		//Positive Large

//#define INFINIT (65535.0f)
#define INFINIT (100.0f)
/*
* This section define the shape of the membership 
* function for the first input. 
* In this case: ACCELERATION
*/
#define factor_acc 1
//Since the membership are symetric,we use those define to avoid redondancy
#define IN1_LL (3.75f)*factor_acc
#define IN1_LS (1.67f)*factor_acc
#define IN1_SL (3.33f)*factor_acc 
#define IN1_SS (2.50f)*factor_acc
#define IN1_ZE (1.63f)*factor_acc
 
//Negative large triangle definition
#define IN1_NL_L (-INFINIT)
#define IN1_NL_M (-IN1_LL)
#define IN1_NL_R (-IN1_LS)

//Negative small triangle definition
#define IN1_NS_L (-IN1_SL)
#define IN1_NS_M (-IN1_SS)
#define IN1_NS_R ( 0.00f)

//Zero triangle definition
#define IN1_ZE_L (-IN1_ZE)
#define IN1_ZE_M ( 0.00f)
#define IN1_ZE_R ( IN1_ZE)

//Positive small triangle definition
#define IN1_PS_L (0.00f)
#define IN1_PS_M (IN1_SS)
#define IN1_PS_R (IN1_SL)

//Positive large triangle definition
#define IN1_PL_L (IN1_LS)
#define IN1_PL_M (IN1_LL)
#define IN1_PL_R (INFINIT)

/*
* This section define the shape of the membership 
* function for the second input. 
* In this case: VELOCITY ERROR
*/
//Since the membership are symetric,we use those define to avoid redondancy
#define factor_vel 1.2
#define IN2_LL (43.08f)*factor_vel
#define IN2_LS (12.92f)*factor_vel
#define IN2_SL (43.8f)*factor_vel
#define IN2_SS (7.00f)*factor_vel
#define IN2_ZE (14.0f)*factor_vel
 
//Negative large triangle definition
#define IN2_NL_L (-INFINIT)
#define IN2_NL_M (-IN2_LL)
#define IN2_NL_R (-IN2_LS)

//Negative small triangle definition
#define IN2_NS_L (-IN2_SL)
#define IN2_NS_M (-IN2_SS)
#define IN2_NS_R ( 0.00f)

//Zero triangle definition
#define IN2_ZE_L (-IN2_ZE)
#define IN2_ZE_M ( 0.00f)
#define IN2_ZE_R ( IN2_ZE)

//Positive small triangle definition
#define IN2_PS_L (0.00f)
#define IN2_PS_M (IN2_SS)
#define IN2_PS_R (IN2_SL)

//Positive large triangle definition
#define IN2_PL_L (IN2_LS)
#define IN2_PL_M (IN2_LL)
#define IN2_PL_R (INFINIT)

/*
* This section define the shape of the membership 
* function for the output. 
* In this case: DELTA ACC. PEDAL VALUE 
*/	
#define OUT_MIN -4
#define OUT_MAX 4

//Since the membership are symetric,we use those define to avoid redondancy
#define OUT_LL (2.00f)
#define OUT_LS (1.00f)
#define OUT_SL (2.00f) 
#define OUT_SS (0.80f)
#define OUT_ZE (0.80f)
 
//Negative large triangle definition
#define OUT_NL_L (-INFINIT)
#define OUT_NL_M (-OUT_LL)
#define OUT_NL_R (-OUT_LS)

//Negative small triangle definition
#define OUT_NS_L (-OUT_SL)
#define OUT_NS_M (-OUT_SS)
#define OUT_NS_R ( 0.00f)

//Zero triangle definition
#define OUT_ZE_L (-OUT_ZE)
#define OUT_ZE_M ( 0.00f)
#define OUT_ZE_R ( OUT_ZE)

//Positive small triangle definition
#define OUT_PS_L (0.00f)
#define OUT_PS_M (OUT_SS)  
#define OUT_PS_R (OUT_SL)

//Positive large triangle definition
#define OUT_PL_L (OUT_LS)
#define OUT_PL_M (OUT_LL)
#define OUT_PL_R (INFINIT)

float tri_lr[5][3];

#endif
