
#ifndef INCLUDE_BUSCOMMUNICATION_H_
#define INCLUDE_BUSCOMMUNICATION_H_
#include <stdint.h>
#include <stddef.h>
// typedef struct master_state_feedback{
// 	uint32_t slaves_responding; /**< Sum of responding slaves on all Ethernet devices. */
// 	uint32_t al_states; /**< Application-layer states of all slaves.
//                                   The states are coded in the lower 4 bits.
//                                   If a bit is set, it means that at least one
//                                   slave in the bus is in the corresponding
//                                   state:
//                                   - Bit 0: \a INIT
//                                   - Bit 1: \a PREOP
//                                   - Bit 2: \a SAFEOP
//                                   - Bit 3: \a OP */
// 	uint32_t link_up; /**< \a true, if at least one Ethernet link is up. */
// 	uint32_t working_counter; /**< Value of the last working counter. */
// 	uint32_t wc_state; /**< Working counter interpretation.
// 							EC_WC_ZERO(0): no registered process data were exchanged
// 							EC_WC_INCOMPLETE(1): some of the registered process data were exchanged
// 							EC_WC_COMPLETE(2): all registered process data were exchanged*/
// 	int reference_clock_state; /**< DC synchronous clock state, only the master shift mode is valid
// 									0: master reference clock time was acquired
// 									-ENXIO(-6): No reference clock found.
// 									-EIO(-5): Slave synchronization datagram was not received*/
// }master_state_feedback;

#ifdef __cplusplus
	extern "C" {
#endif

typedef struct master_state_feedback master_state_feedback;


int InitMaster(size_t in_size,size_t out_size,uint32_t master_index);

void CloseMaster(uint32_t master_index);

master_state_feedback GetMasterState(uint32_t master_index);

void GetMasterState2(uint32_t master_index,
					uint32_t* slaves_responding,
					uint32_t* al_states,
					uint32_t* link_up,
					uint32_t* working_counter,
					uint32_t* wc_state,
					int* reference_clock_state);

int GetMasterPdo(uint32_t master_index,uint32_t bitoffs,uint8_t bitsize,void* value);

int SetMasterPdo(uint32_t master_index,uint32_t bitoffs,uint8_t bitsize,void* value);

#ifdef __cplusplus
}
#endif

#endif /*INCLUDE_BUSCOMMUNICATION_H_*/