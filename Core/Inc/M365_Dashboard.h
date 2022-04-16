/*
 * M365_Dashboard.h
 *
 *  Created on: Nov 27, 2021
 *      Author: stancecoke
 */

#ifndef INC_M365_DASHBOARD_H_
#define INC_M365_DASHBOARD_H_

#include "motor.h"


void M365Dashboard_init();

void search_DashboardMessage(M365State_t *M365State, UART_HandleTypeDef huart1);
void process_DashboardMessage(M365State_t* p_M365State, uint8_t *message, uint8_t length, UART_HandleTypeDef huart1);
void addCRC(uint8_t * message, uint8_t size);
int16_t checkCRC(uint8_t * message, uint8_t size);

#endif /* INC_M365_DASHBOARD_H_ */
