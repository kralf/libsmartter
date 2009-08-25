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
* Fichier : fuzzy_ctl.h
* Responsable : FP
* ----------------------------------------
* Description :
* - Contain useful function for the fuzzy 
*   logique controler.
* - This controler has been developed for
*   a specific application: satellite control
* ----------------------------------------
* Utilisation :
* - The controler must have 2 inputs and one 
*   output.
* - Make sure that the membership limits are
*   define for your specific application in 
*   the fuzzy_membership.h file.
* ----------------------------------------
* Historique : 
* FP | 2004-04-10 | Creation du fichier
*********************************************/

#ifndef SMART_FUZZY_CONTROL_H
#define SMART_FUZZY_CONTROL_H

#define INT8 signed char
#define UINT8 unsigned char
#define FLOAT32 float

#define MAX_DECISION 5
#define PRECISION 100.0f // TODO: find the smallest value we can use
#define MAX_NEG -32768

void fuzzy_acc_init (void);

/** 
 * Fuzzy logic controler.
 * This function take two inputs and make a decision to produce a specific output. This fuzzy logic controler is a mamdani type and have those caracteristics: And methode: min, Implication: min, Defuzziliation: centroide and the membership function are define with a triangular form.  
 * @author Francois Pomerleau
 * @date 2004-04-10
 * @param input1 Input to the controler
 * @param input2 Input to the controler
 * @return Return the controled value
 * @note Before use this function, we recommande to make test a simulation. MatLab/Simulink could be useful. 
*/
FLOAT32 fuzzy_acc_ctl (const FLOAT32 input1, const FLOAT32 input2);

/**   
 * Calculate the weight of an input.
 * This function calculate the weight of an input in relation with the me mbership function selected.
 * @author Francois Pomerleau
 * @date 2004-04-10
 * @param input Value of the input to be evaluate
 * @param left_limit Left limit of the membership function
 * @param middle_limit Highest point of the membership function
 * @param right_limit Right limit of the membership function
 * @param saturation Value that the result can't exceed
 * @return Return the weight of the input
*/
//FLOAT32 cal_weight(const FLOAT32 input, const FLOAT32 left_limit, 
				   //const FLOAT32 middle_limit, const FLOAT32 right_limit, 
				   //const FLOAT32 saturation);


/** 
 * Find the smallest value between two.
 * This function take two values and return the smallest one.
 * @author Francois Pomerleau
 * @date 2004-04-10
 * @param value1 Value to be compared
 * @param value2 Value to be compared
 * @return Return the smallest value
*/
FLOAT32 min_value (const FLOAT32 value1, const FLOAT32 value2);

/** 
 * Find the maximum value between the seven inputs.
 * This function take seven values and return the biggest one.
 * @author Francois Pomerleau
 * @date 2004-04-10
 * @param value1 Value to be compared
 * @param value2 Value to be compared
 * @param value3 Value to be compared
 * @param value4 Value to be compared
 * @param value5 Value to be compared
 * @param value6 Value to be compared
 * @param value7 Value to be compared
 * @return Return the smallest value
*/
//FLOAT32 max_value (const FLOAT32 value1, const FLOAT32 value2, 
//			 const FLOAT32 value3, const FLOAT32 value4, 
//			 const FLOAT32 value5, const FLOAT32 value6, 
//			 const FLOAT32 value7);


#endif
