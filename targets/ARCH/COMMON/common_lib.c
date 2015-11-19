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
/*! \file common_lib.c 
 * \brief common APIs for different RF frontend device 
 * \author HongliangXU, Navid Nikaein
 * \date 2015
 * \version 0.2
 * \company Eurecom
 * \maintainer:  navid.nikaein@eurecom.fr
 * \note
 * \warning
 */
#include <stdio.h>
#include <strings.h>
#include <dlfcn.h>
#include <errno.h>
#include <string.h>

#include "common_lib.h"

/* 
   FT: The device interface is implemented in a shared library, the softmodem knows which kind
   of harware is used by parsing the device name returned when calling the trx_getinfo_func function
   provided by this shared library
*/
int set_device(openair0_device *device) {

  switch (device->type) {
    
  case EXMIMO_DEV:
    printf("[%s] has loaded EXPRESS MIMO device.\n",((device->host_type == BBU_HOST) ? "BBU": "RRH"));
    return 0;     
    break;
  case USRP_DEV:
    printf("[%s] has loaded USRP device.\n",((device->host_type == BBU_HOST) ? "BBU": "RRH")); 
    return 0;
    break;
  case BLADERF_DEV:
    printf("[%s] has loaded BLADERF device.\n",((device->host_type == BBU_HOST) ? "BBU": "RRH")); 
    return 0;
    break;
  case NONE_DEV:
    printf("[%s] has not loaded a HW device.\n",((device->host_type == BBU_HOST) ? "BBU": "RRH"));
    return 0; 
    break;    
  default:
    printf("[%s] invalid HW device.\n",((device->host_type == BBU_HOST) ? "BBU": "RRH")); 
    return -1;
    break;
  }
  
}

int set_transport(openair0_device *device) {

  switch (device->transp_type) {
    
  case ETHERNET_TP:
    printf("[%s] has loaded ETHERNET trasport protocol.\n",((device->host_type == BBU_HOST) ? "BBU": "RRH"));
    return 0;     
    break;
  case NONE_TP:
    printf("[%s] has not loaded a transport protocol.\n",((device->host_type == BBU_HOST) ? "BBU": "RRH"));
    return 0; 
    break;    
  default:
    printf("[%s] invalid transport protocol.\n",((device->host_type == BBU_HOST) ? "BBU": "RRH")); 
    return -1;
    break;
  }
  
}

/* FT: looking for the rh interface library and load it */
int load_lib(openair0_device *device, openair0_config_t *openair0_cfg, char *cfgfile, uint8_t flag) {
  
  void *lib_handle;
  char *OAI_RF_LIBNAME;
  char *OAI_TP_LIBNAME;
  oai_device_initfunc_t fp ;

  if (device->host_type==BBU_HOST) {
    OAI_RF_LIBNAME= "/home/guepe/openairinterface5g/cmake_targets/lte_noS1_build_oai/build/liboai_device.so";
    OAI_TP_LIBNAME= "/home/guepe/openairinterface5g/cmake_targets/lte_noS1_build_oai/build/liboai_transpro.so";
  } else {
    OAI_RF_LIBNAME= "liboai_device.so";
    OAI_TP_LIBNAME= "liboai_transpro.so";
  }
    
    if (flag == RF_DEVICE) {
      lib_handle = dlopen(OAI_RF_LIBNAME, RTLD_LAZY);
      if (!lib_handle) {
	printf( "Unable to locate %s: HW device set to NONE_DEV.\n", OAI_RF_LIBNAME);
	return 0;
      } 
      
      fp = dlsym(lib_handle,"device_init");
      
      if (fp != NULL ) {
	fp(device,openair0_cfg,cfgfile);
      } else {
	fprintf(stderr, "%s %d:oai device intializing function not found %s\n", __FILE__, __LINE__, dlerror());
	return -1;
      }
    } else {
      lib_handle = dlopen(OAI_TP_LIBNAME, RTLD_LAZY);
      if (!lib_handle) {
	printf( "Unable to locate %s: transport protocol set to NONE_TP.\n", OAI_TP_LIBNAME);
	return 0;
      } 
      
      fp = dlsym(lib_handle,"transport_init");
      
      if (fp != NULL ) {
	fp(device,openair0_cfg,cfgfile);
      } else {
	fprintf(stderr, "%s %d:oai device intializing function not found %s\n", __FILE__, __LINE__, dlerror());
	return -1;
      }
    } 
    
  return 0; 	       
}



int openair0_device_load(openair0_device *device, openair0_config_t *openair0_cfg) {
  
  int rc;
  static char   *cfgfile;
  uint8_t       flag=RF_DEVICE;
  /* FT: rewritten for shared library, common, radio head interface implementation */
  rc=load_lib(device, openair0_cfg, NULL,flag);
  if ( rc >= 0) {       
    if ( set_device(device) < 0) {
      fprintf(stderr, "%s %d:Unsupported radio head\n",__FILE__, __LINE__);
      return -1;		   
    }   
  }
  
  return 0;
}

int openair0_transport_load(openair0_device *device, openair0_config_t *openair0_cfg) {
  
  int rc;
  static char   *cfgfile;
  uint8_t       flag=TRANSPORT_PROTOCOL;

  /* FT: rewritten for shared library, common, radio head interface implementation */
  rc=load_lib(device, openair0_cfg, NULL,flag);
  if ( rc >= 0) {       
    if ( set_transport(device) < 0) {
      fprintf(stderr, "%s %d:Unsupported radio head\n",__FILE__, __LINE__);
      return -1;		   
    }   
  }
  
  return 0;
}
