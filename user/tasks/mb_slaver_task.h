#ifndef MB_SLAVER_TASK_H__
#define MB_SLAVER_TASK_H__

void hmi_event_set(sys_evtcode_mask_e event);
void hmi_event_clear(sys_evtcode_mask_e event);
bool hmi_event_get(sys_evtcode_mask_e event);

int mb_slaver_task(void);

#endif

