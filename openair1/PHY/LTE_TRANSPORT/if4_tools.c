/*******************************************************************************
    OpenAirInterface
    Copyright(c) 1999 - 2014 Eurecom

    OpenAirInterface is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.


    OpenAirInterface is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with OpenAirInterface.The full GNU General Public License is
   included in this distribution in the file called "COPYING". If not,
   see <http://www.gnu.org/licenses/>.

  Contact Information
  OpenAirInterface Admin: openair_admin@eurecom.fr
  OpenAirInterface Tech : openair_tech@eurecom.fr
  OpenAirInterface Dev  : openair4g-devel@lists.eurecom.fr

  Address      : Eurecom, Campus SophiaTech, 450 Route des Chappes, CS 50193 - 06904 Biot Sophia Antipolis cedex, FRANCE

 *******************************************************************************/

/*! \file PHY/LTE_TRANSPORT/if4_tools.c
* \brief 
* \author Mauricio Gunther, S. Sandeep Kumar, Raymond Knopp
* \date 2016
* \version 0.1
* \company Eurecom
* \email: knopp@eurecom.fr 
* \note
* \warning
*/

#include "PHY/LTE_TRANSPORT/if4_tools.h"

// Define how data blocks are stored and transferred


void send_IF4(PHY_VARS_eNB *eNB, int subframe){
	eNB_proc_t *proc = &eNB->proc;
	//int frame=proc->frame_tx;
	//int subframe=proc->subframe_tx;

	LTE_DL_FRAME_PARMS *fp=&eNB->frame_parms;

	uint32_t i,j;
	float *data_block = malloc(length*sizeof(long));

  // Generate IF4 packet (for now DL) with frame status information
  dl_packet = gen_IF4_dl_packet( /* ADD INFO and data_block pointer */ );
	
	for(i=0; i<fp->symbols_per_tti; i++) {
    
    // Do compression of the two parts and generate data blocks
    
    symbol = eNB->common_vars.txdataF[0][0 /*antenna number*/][subframe*fp->ofdm_symbol_size*(fp->symbols_per_tti)]
    data_block[j] = Atan(symbol[fp->ofmd_symbol_size - NrOfNonZeroValues + j -1])<<16 + Atan(symbol[fp->ofmd_symbol_size - NrOfNonZeroValues + j]);
    data_block[j+NrOfNonZeroValues] = Atan(subframe[i][j+1])<<16 + Atan(subframe[i][j+2]);
    
    // Set data blocks and update subframe no./other information to generated packet
    
    
    // Write the packet(s) to the fronthaul
    
  }
  		    
}

void recv_IF4( /* ADD INFO and data_block pointer */ ) {

  // Read packet(s) from the fronthaul
  
  // Apply reverse processing - decompression
  
  // Generate and return the OFDM symbols (txdataF)
  
}

IF4_dl_packet gen_IF4_dl_packet( /* ADD INFO and data_block pointer */ ) {  
  IF4_dl_packet dl_packet;
  
  // Set destination and source address
  
  // Set Type and Sub-Type
  dl_packet.type = ; //08_0A ? 
  dl_packet.sub_type = 0x0020;

  // Leave reserved as it is 
  //dl_packet.rsvd = ;
  
  // Set frame status
  dl_packet.frame_status.ant_num = ;
  dl_packet.frame_status.ant_start = ;
  dl_packet.frame_status.rf_num = ;
  dl_packet.frame_status.sf_num = ;
  dl_packet.frame_status.sym_num = ;
  //dl_packet.frame_status.rsvd = ;
    
  // Set data blocks if sent
  if (data_block != NULL) {
    
  } else {
    
  }
  
  // Set frame check sequence
  dl_packet.fcs = ;
  
  return dl_packet;
}

IF4_ul_packet gen_IF4_ul_packet( /* ADD INFO and data_block pointer */ ) {
  IF4_ul_packet ul_packet;

  // Set destination and source address
  
  // Set Type and Sub-Type
  ul_packet.type = ; //08_0A ? 
  ul_packet.sub_type = 0x0019;

  // Leave reserved as it is 
  //ul_packet.rsvd = ;
  
  // Set frame status
  ul_packet.frame_status.ant_num = ;
  ul_packet.frame_status.ant_start = ;
  ul_packet.frame_status.rf_num = ;
  ul_packet.frame_status.sf_num = ;
  ul_packet.frame_status.sym_num = ;
  //ul_packet.frame_status.rsvd = ;
    
  // Set antenna specific gain
  ul_packet.gain0.exponent = ;
  //ul_packet.gain0.rsvd = ;
  
  // Set data blocks if sent
  if (data_block != NULL) {
    
  } else {
    
  }
  
  // Set frame check sequence
  ul_packet.fcs = ;
  
  return ul_packet;
}

IF4_prach_packet gen_IF4_prach_packet( /* ADD INFO and data_block pointer */ ) {
  IF4_prach_packet prach_packet;

  // Set destination and source address
  
  // Set Type and Sub-Type
  prach_packet.type = ; //08_0A ? 
  prach_packet.sub_type = 0x0021;

  // Leave reserved as it is 
  //prach_packet.rsvd = ;
  
  // Set LTE Prach configuration
  //prach_packet.prach_conf.rsvd = ;
  prach_packet.prach_conf.ant = ;
  prach_packet.prach_conf.rf_num = ;
  prach_packet.prach_conf.sf_num = ;
  prach_packet.prach_conf.exponent = ;  
      
  // Set data blocks if sent
  if (data_block != NULL) {
    
  } else {
    
  }
  
  // Set frame check sequence
  prach_packet.fcs = ;
  
  return prach_packet;
} 
