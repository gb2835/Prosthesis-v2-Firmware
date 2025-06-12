/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_PROSTHESIS_V2_H_
#define INC_PROSTHESIS_V2_H_

#include <stdint.h>

typedef enum
{
	Ankle,
	Combined,
	Knee
} Joint_e;

typedef enum
{
	Left,
	Right
} Side_e;

typedef struct
{
	Side_e Side;
	Joint_e Joint;
} Prosthesis_Init_t;

extern uint8_t isProsthesisControlRequired;

void InitProsthesisControl(Prosthesis_Init_t *Device_Init);
void RunProsthesisControl(void);


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_PROSTHESIS_V2_H_ */
