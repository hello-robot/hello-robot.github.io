/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/
#include "AnalogManager.h"
#include "Common.h"

AnalogManager analog_manager;

//////////////////////// ANALOG READ ISR //////////////////////////////////////////////////

// Use the SAMD21's ISR to transfer ADC results to buffer array in memory


#define IDX_ANA_V_BATT 0
#define IDX_ANA_CLIFF_0 2
#define IDX_ANA_CLIFF_1 3
#define IDX_ANA_CLIFF_2 4
#define IDX_ANA_CLIFF_3 5
#define IDX_ANA_TEMP 6
#define IDX_ANA_CURRENT 7


AnalogManager::AnalogManager(){
  cliff_LPFa = 1.0; 
  cliff_LPFb = 0.0;
  voltage_LPFa = 1.0; 
  voltage_LPFb = 0.0;
  current_LPFa = 1.0; 
  current_LPFb = 0.0;
  temp_LPFa = 1.0; 
  temp_LPFb = 0.0;
  first_filter=1;
  first_read_done=0;
  first_config=1;
  adc_input_id=0;
  }
  
void AnalogManager::update_config(Pimu_Config * cfg_new, Pimu_Config * cfg_old)
{
  if (cfg_new->cliff_LPF!=cfg_old->cliff_LPF) 
  {
    cliff_LPFa = exp(cfg_new->cliff_LPF*-2*3.14159/FS); // z = e^st pole mapping
    cliff_LPFb = (1.0-cliff_LPFa);
  }
  if (cfg_new->voltage_LPF!=cfg_old->voltage_LPF) 
  {
    voltage_LPFa = exp(cfg_new->voltage_LPF*-2*3.14159/FS); // z = e^st pole mapping
    voltage_LPFb = (1.0-voltage_LPFa);
  }
  if (cfg_new->current_LPF!=cfg_old->current_LPF) 
  {
    current_LPFa = exp(cfg_new->current_LPF*-2*3.14159/FS); // z = e^st pole mapping
    current_LPFb = (1.0-current_LPFa);
  }
  if (cfg_new->temp_LPF!=cfg_old->temp_LPF) 
  {
    temp_LPFa = exp(cfg_new->temp_LPF*-2*3.14159/FS); // z = e^st pole mapping
    temp_LPFb = (1.0-temp_LPFa);
  }
  if (first_config)
  {
    voltage = adcResult[IDX_ANA_V_BATT];
    current = adcResult[IDX_ANA_CURRENT];
    temp =    adcResult[IDX_ANA_TEMP];
    cliff[0] = adcResult[IDX_ANA_CLIFF_0];
    cliff[1] = adcResult[IDX_ANA_CLIFF_1];
    cliff[2] = adcResult[IDX_ANA_CLIFF_2];
    cliff[3] = adcResult[IDX_ANA_CLIFF_3];
    first_config=0;
  }
}

    
void AnalogManager::step(Pimu_Status * stat, Pimu_Config * cfg)
{
  if (!first_read_done)
    return;

  if (first_filter)
  {
    voltage = adcResult[IDX_ANA_V_BATT];
    current = adcResult[IDX_ANA_CURRENT];
    temp =    adcResult[IDX_ANA_TEMP];
    cliff[0] = adcResult[IDX_ANA_CLIFF_0];
    cliff[1] = adcResult[IDX_ANA_CLIFF_1];
    cliff[2] = adcResult[IDX_ANA_CLIFF_2];
    cliff[3] = adcResult[IDX_ANA_CLIFF_3];
    first_filter=false;
  }

  voltage = voltage * voltage_LPFa +  voltage_LPFb* adcResult[IDX_ANA_V_BATT];
  current = current * current_LPFa +  current_LPFb* adcResult[IDX_ANA_CURRENT];
  temp =    temp *    temp_LPFa +     temp_LPFb*    adcResult[IDX_ANA_TEMP];
  cliff[0]= cliff_LPFa*cliff[0] +  cliff_LPFb*adcResult[IDX_ANA_CLIFF_0];
  cliff[1]= cliff_LPFa*cliff[1] +  cliff_LPFb*adcResult[IDX_ANA_CLIFF_1];
  cliff[2]= cliff_LPFa*cliff[2] +  cliff_LPFb*adcResult[IDX_ANA_CLIFF_2];
  cliff[3]= cliff_LPFa*cliff[3] +  cliff_LPFb*adcResult[IDX_ANA_CLIFF_3];


  stat->cliff_range[0]=cliff[0]-cfg->cliff_zero[0];
  stat->cliff_range[1]=cliff[1]-cfg->cliff_zero[1];
  stat->cliff_range[2]=cliff[2]-cfg->cliff_zero[2];
  stat->cliff_range[3]=cliff[3]-cfg->cliff_zero[3];
  at_cliff[0] = stat->cliff_range[0]<cfg->cliff_thresh; //Neg is dropoff
  at_cliff[1] = stat->cliff_range[1]<cfg->cliff_thresh;
  at_cliff[2] = stat->cliff_range[2]<cfg->cliff_thresh;
  at_cliff[3] = stat->cliff_range[3]<cfg->cliff_thresh;
  
}

void AnalogManager::setupADC()
{
  //ADC ISR setup
  ADC->INPUTCTRL.bit.MUXPOS = adc_input_id;                   // Set the analog input to A0
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  //SAMPLEN 5:0 bits, sample time =(samplen+1)*(clk_adc/2)
  ADC->SAMPCTRL.bit.SAMPLEN = 0x07;                  // Set max Sampling Time Length to 4x ADC clock pulse (42us)
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |      // Divide Clock ADC GCLK by 512 (48MHz/512 = 93.7kHz)
                   ADC_CTRLB_RESSEL_10BIT |          // Set the ADC resolution to 10 bits (use ADC_CTRLB_RESSEL_12BIT for 12)
                   ADC_CTRLB_FREERUN;                // Set the ADC to free run
 
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization 
  NVIC_SetPriority(ADC_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for the ADC to 0 (highest)
  NVIC_EnableIRQ(ADC_IRQn);         // Connect the ADC to Nested Vector Interrupt Controller (NVIC)
  ADC->INTENSET.reg = ADC_INTENSET_RESRDY;           // Generate interrupt on result ready (RESRDY)
  ADC->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  ADC->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
}



//ISR
void ADC_Handler()
{
  if (ADC->INTFLAG.bit.RESRDY)                       // Check if the result ready (RESRDY) flag has been set
  {
    ADC->INTFLAG.bit.RESRDY = 1;                     // Clear the RESRDY flag
    while(ADC->STATUS.bit.SYNCBUSY);                 // Wait for read synchronization
    analog_manager.adcResult[analog_manager.adc_input_id] = ADC->RESULT.reg;          // Read the result;
    analog_manager.adc_input_id++;
    if (analog_manager.adc_input_id==NUM_ADC_INPUTS)
    {
      analog_manager.adc_input_id=0;
      analog_manager.first_read_done=1;
    }
    ADC->CTRLA.bit.ENABLE = 0;                     // Disable the ADC
    while(ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization
    ADC->INPUTCTRL.bit.MUXPOS = analog_manager.adc_input_id;         // Set the analog input channel
    while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
    ADC->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
    while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  }
}
